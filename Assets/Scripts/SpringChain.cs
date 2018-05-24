using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpringChain : MonoBehaviour
{
    public List<Vector3> x_cur;
    public List<Vector3> x_pre;
    public List<Vector3> f_drag;
    public float drag = 0.5f;
    public Vector3 gravity = new Vector3(0, -0.05f);

    public float timeFactor = 100;
    public float iterationCount = 10; // 迭代次数，提升稳定性减少震荡
    public float iterationResetCount = 5;
    public List<Transform> vertice;
    public List<Vector3> vertSpringColl;

    public float edgeStiffnessFactor;
    public AnimationCurve edgeStiffnessCurve;
    public List<EdgeSetting> edgeSettings;

    public int spaceCount = 5;
    public float lenChainRest;
    public float resetRotForce = 0.5f;
    [Serializable]
    public class EdgeSetting
    {
        public float stiffness = 0.5f;
        public float mass = 1;
        public List<Vector3> spring;
        public List<Vector3> dir;
        public List<float> springForce;
        public List<float> len;
        public List<float> lenRest;
        public float lenFactor = 1;
        public List<float> lenDiff;
        public List<Edge> edges;
        public int space;
        public void InitEdgeSetting(List<Transform> vertice)
        {
            lenRest = new List<float>();
            edges = new List<Edge>();
            for (int i = 0; i < vertice.Count - space; i++)
            {
                edges.Add(new Edge() { V1 = vertice[i], V2 = vertice[i + space] }); // nV = nBx + space 
            }
            foreach (var edge in edges)
            {
                var dist = Vector3.Distance(edge.V1.position, edge.V2.position) * lenFactor;
                lenRest.Add(dist);
            }
        }
        internal void Update(List<Vector3> listColl, float stiffnessFactor)
        {
            len = new List<float>();
            lenDiff = new List<float>();
            dir = new List<Vector3>();
            springForce = new List<float>(); // nBx * 1
            int e = 0;
            foreach (var edge in edges)
            {
                len.Add(Vector3.Distance(edge.V1.position, edge.V2.position));
                lenDiff.Add(len[e] - lenRest[e]);
                springForce.Add(stiffnessFactor * stiffness * lenDiff[e]);
                dir.Add((edge.V2.position - edge.V1.position).normalized);

                listColl[e] += springForce[e] * dir[e] / mass; // v1
                listColl[e + space] += springForce[e] * -dir[e] / mass; // v2

                e++;
            }
        }
    }
    [Serializable]
    public struct Edge
    {
        public Transform V1; // Vertex1
        public Transform V2;
    }
    void Start()
    {
        InitPosition();
        InitEdges();
    }
    void InitPosition()
    {
        x_pre = new List<Vector3>();
        foreach (var vert in vertice)
        {
            x_pre.Add(vert.position);
        }
    }
    private void InitEdges()
    {
        if (edgeSettings == null || edgeSettings.Count == 0)
        {
            AddEdgesSettings();
        }
        foreach (var es in edgeSettings)
        {
            es.InitEdgeSetting(vertice);
        }
        lenChainRest = 0f;
        foreach (var lenRest in edgeSettings[0].lenRest)
        {
            lenChainRest += lenRest;
        }
    }
    void LateUpdate()
    {
        var dt = Time.deltaTime * timeFactor;
        dt /= iterationCount;
        var end = vertice[vertice.Count - 1]; // 末端骨骼
        var endPos = end.position;
        var start = vertice[0];
        var lenChain = Vector3.Distance(endPos, start.position);
        var dirChain = (endPos - start.position).normalized;
        var endPosRest = start.position + dirChain * lenChainRest; // rest chain len

        for (int i = 0; i < iterationCount; i++)
        {
            UpdateSpring(dt);
            if (lenChain > lenChainRest) end.position = Vector3.Lerp(endPos, endPosRest, (i + 1) / iterationCount);
        }
        for (int i = 0; i < iterationResetCount; i++)
        {
            ResetRestLen();
            //ResetRestRotation();
        }
        FixLookAt();
        DrawDir();
    }

    private void ResetRestRotation()
    {
        throw new NotImplementedException();
    }
    [ContextMenu("AddEdgesSettings")]
    void AddEdgesSettings()
    {
        edgeSettings = new List<EdgeSetting>();
        for (int i = 1; i <= spaceCount; i++)
        {
            var es = new EdgeSetting();
            es.space = i;
            edgeSettings.Add(es);
        }
    }
    void UpdateSpring(float time)
    {
        vertSpringColl = new List<Vector3>(); // nV * 1
        f_drag = new List<Vector3>();
        x_cur = new List<Vector3>();
        foreach (var vert in vertice)
        {
            x_cur.Add(vert.position);
            vertSpringColl.Add(Vector3.zero);
        }
        for (int i = 0; i < spaceCount; i++)
        {
            if (i < edgeSettings.Count)
                edgeSettings[i].Update(vertSpringColl, edgeStiffnessFactor *
                    edgeStiffnessCurve.Evaluate((float)i / spaceCount));
        }
        { // 内外力合力
            int v = 0;
            var x_new = new List<Vector3>();
            var acc_wind = Vector3.zero;
            foreach (var vert in vertice)
            {
                // 伴随一个简单阻尼模型的 Verlet 迭代
                f_drag.Add(drag * (x_cur[v] - x_pre[v]));
                var force = f_drag[v] + vertSpringColl[v] + acc_wind + gravity;
                force *= time;
                x_new.Add(x_cur[v] + force);
                v++;
            }
            x_pre = new List<Vector3>(x_cur);
            x_cur = new List<Vector3>(x_new);
        }
        { // 固定点
            int v = 0;
            var pins = new List<int>();
            //pins.Add(0);
            foreach (var vert in vertice)
            {
                if (!pins.Contains(v) && !vert.GetComponent<Rigidbody>().isKinematic)
                {
                    vert.position = x_cur[v];
                }
                v++;
            }
        }
        FixLookAt();
    }
    void ResetRestLen()
    { // 瞬间还原长度
        for (int v = 1; v < vertice.Count; v++)
        {
            if (!vertice[v].GetComponent<Rigidbody>().isKinematic)
            {
                var dir = (vertice[v].position - vertice[v - 1].position).normalized;
                vertice[v].position = vertice[v - 1].position + dir * edgeSettings[0].lenRest[v - 1];
            }
        }
        for (int v = 1; v < vertice.Count; v++)
        {
            var boneAxis = -vertice[v].up;
            if (!vertice[v].GetComponent<Rigidbody>().isKinematic)
            {
                var localPos = vertice[v].position - vertice[v - 1].position;
                var diffRot = Vector3.Distance(boneAxis, localPos.normalized);
                localPos += boneAxis * resetRotForce * diffRot;
                var dir = localPos.normalized;
                vertice[v].position = vertice[v - 1].position + dir * edgeSettings[0].lenRest[v - 1];
            }
        }
        FixLookAt();
        for (int i = vertice.Count - 2; i >= 0; i--)
        {
            if (!vertice[i].GetComponent<Rigidbody>().isKinematic)
            {
                var dir = (vertice[i].position - vertice[i + 1].position).normalized;
                vertice[i].position = vertice[i + 1].position + dir * edgeSettings[0].lenRest[i];
            }
        }
        for (int i = vertice.Count - 2; i >= 0; i--)
        {
            var boneAxis = vertice[i].up;
            if (!vertice[i].GetComponent<Rigidbody>().isKinematic)
            {
                var localPos = vertice[i].position - vertice[i + 1].position;
                var diffRot = Vector3.Distance(boneAxis, localPos.normalized);
                localPos += boneAxis * resetRotForce * diffRot;
                var dir = localPos.normalized;
                vertice[i].position = vertice[i + 1].position + dir * edgeSettings[0].lenRest[i];
            }
        }
        FixLookAt();
        foreach (var vert in vertice)
        {
            x_pre.Add(vert.position);
        }
        x_cur = new List<Vector3>(x_pre);
    }
    void FixLookAt()
    { // 更改弹簧朝向
        int v = 0;
        foreach (var vert in vertice)
        {
            if (v < vertice.Count - 1)
                vert.up = -(vertice[v + 1].position - vert.position).normalized;
            v++;
            continue;
            if (!vert.GetComponent<Rigidbody>().isKinematic)
            {
                if (v == 0)
                    vert.up = -(vert.position - vertice[v + 1].position).normalized;
                else if (v == vertice.Count - 1)
                    vert.up = -(vertice[v - 1].position - vert.position).normalized;
                else
                {
                    vert.up = (vertice[v - 1].position - vertice[v + 1].position).normalized;
                }
            }
            v++;
        }
    }
    private void OnDrawGizmos()
    {
        if (enabled) DrawSpring();
    }
    public float rayLength = 0.2f;
    public Color rayColor = Color.red;
    public float sphereRadius = 5f;
    public Color sphereColor = Color.blue;
    void DrawSpring()
    {
        if (vertSpringColl == null) return;
        int v = 0;
        Gizmos.color = sphereColor;
        foreach (var vert in vertice)
        {
            if (v < vertSpringColl.Count)
                Gizmos.DrawWireSphere(vert.position, vertSpringColl[v].magnitude * sphereRadius);
            v++;
        }
    }
    void DrawDir()
    {
        int i = 0;
        foreach (var edge in edgeSettings[0].edges)
        {
            if (i >= edgeSettings[0].dir.Count) continue;
            Debug.DrawRay(edge.V1.position, edgeSettings[0].dir[i] * rayLength, rayColor, 0, false);
            Debug.DrawRay(edge.V2.position, -edgeSettings[0].dir[i] * rayLength, rayColor, 0, false);
            i++;
        }
    }
}
