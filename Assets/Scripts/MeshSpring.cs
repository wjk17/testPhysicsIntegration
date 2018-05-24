using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
[CustomEditor(typeof(MeshSpring))]
public class MeshSpringEditor : Editor
{
    public override void OnInspectorGUI()
    {
        var o = (MeshSpring)target;
        base.OnInspectorGUI();
        if (GUILayout.Button("Change"))
        {
            o.Change();
        }
    }
}
#endif
public class MeshSpring : MonoBehaviour
{
    public Matrix x_pre;
    public Matrix x_cur;
    public Matrix vs; // 顶点矩阵
    public Matrix es; // 边矩阵
    public Matrix F; // 面矩阵
    //public Matrix center;
    public float stiffness;
    public float drag;
    //public float radius;
    public Transform wind_dir;
    public float wind_strength; // 风强度
    public float mass;
    public int[] pin_idx;
    public Vector3 gravity;

    // constrains option
    public bool wind = false;
    //public bool ball = true;
    public bool pins = true;
    //public float skinThickness = 0.01f;
    public Matrix L0;
    public Matrix colEdgeV1;
    public Matrix colEdgeV2;

    public Transform[][] points;
    public GameObject prefab;
    public int rowsCount;
    public int columnsCount;
    int _rowsCount;
    int _columnsCount;
    private Mesh mesh;
    public float space;
    public float range = 10f;
    public bool updatePhysics;

    public bool log;
    public bool drawFaces;
    public bool depthTest;
    public Color color = Color.blue;

    void UpdatePhysics()
    {
        GetV();
        // n*1 当前每条边（弹簧）的长度
        var edgeLength = (x_cur.GetRows(colEdgeV1) - x_cur.GetRows(colEdgeV2)); //.RowVectorLength;
        if (log) Debug.Log(edgeLength.ToString("matrix: edgeLength"));
        edgeLength = edgeLength.rowVectorLength;
        if (log) Debug.Log(edgeLength.ToString("matrix: edgeLength RowVectorLength"));
        // 弹力 spring force  
        // n*1 当前每条边（弹簧）与静止长度的差值
        var diff_rest_cur = edgeLength - L0;
        // fs = n*1列矩阵 力的总和（弹力+硬度）
        // 元素是标量，硬度越高弹性越低，变形会越快被重置
        var force_spring = stiffness * diff_rest_cur; // 弹力与长度差值（变形程度）成正比

        // dir = n*3 储存每条边 质点1 到 质点2 的方向
        // 即将 质点1 拉向 质点2 的方向
        Matrix dir = (x_cur.GetRows(colEdgeV2) - x_cur.GetRows(colEdgeV1)).rowVectorNormalized;
        // dir1=dir2=dir3= n*1 分别储存方向的x y z 的列矩阵
        var dir1 = dir.GetColumn(1);
        var dir2 = dir.GetColumn(2);
        var dir3 = dir.GetColumn(3);

        //Debug.Log(force_spring.ToString("matrix: force_spring"));
        //Debug.Log(dir1.ToString("matrix: dir1"));
        // c = 2n*1
        var c = es.GetColumn(1).ColumnExtend(es.GetColumn(2));

        var cs = Matrix.ColumnExtend((force_spring * dir1), -force_spring * dir1);
        //Debug.Log(cs.ToString("matrix: cs"));

        SparseMatrix M1 = Sparse(c, c, Matrix.ColumnExtend((force_spring * dir1), -force_spring * dir1));
        SparseMatrix M2 = Sparse(c, c, Matrix.ColumnExtend((force_spring * dir2), -force_spring * dir2));
        SparseMatrix M3 = Sparse(c, c, Matrix.ColumnExtend((force_spring * dir3), -force_spring * dir3));
        if (log) Debug.Log(M1.ToString("matrix: m1"));
        // M1~3 = nV*1
        // M1~3 三个包含了对角元素的列矩阵合并为一个 nV*3矩阵，再除以质量mass
        // f = ma   to  a = f/m
        // acceleration 弹力的加速度 = 受力 / 质量
        // acc_spring = 质点在这帧的相对移动
        Matrix acc_spring = Matrix.Columns(Diag(M1), Diag(M2), Diag(M3)) / mass;

        // wind force
        var acc_wind = zeros(vs.rowLength, 3); // nV*3的矩阵，每个顶点的风力
        if (wind)
        {
            // f*3 每个面的归一化法线
            var N = Normals(x_cur, F);
            // f*3
            var windM = new Matrix();
            windM.elements = new float[F.rowLength][];
            for (int i = 0; i < F.rowLength; i++)
            {
                var dot = Vector3.Dot(wind_dir.forward, N.GetVector3(i));
                windM.SetRow(i, 1 - Mathf.Abs(dot));//朝向/背向风向时受力最大
            }
            // f*3 每个面受到的风力
            var Fw = (windM * wind_strength);
            // Fw 并排重复三列
            // 因为 F = f*3，要把Fw同时给予每个面的三个顶点
            // 那么共享点会受到每个使用这个点的面所受到的力
            // 一般情况最多同时被6个面使用
            M1 = Sparse(F, F, RepmatColumns(Fw * N.GetColumn(1), 3));
            M2 = Sparse(F, F, RepmatColumns(Fw * N.GetColumn(2), 3));
            M3 = Sparse(F, F, RepmatColumns(Fw * N.GetColumn(3), 3));
            // 最后得出 M1~3 = nV*1 每个顶点受到的风力
            acc_wind = Matrix.Columns(Diag(M1), Diag(M2), Diag(M3)) / mass;
            //Debug.Log(acc_wind.ToString("matrix: acc_wind"));
        }
        var dt = Time.deltaTime;
        dt *= dt;
        // 伴随一个简单阻尼模型的 Verlet 迭代
        var dragM = drag * (x_cur - x_pre);
        var x_new = dragM + x_cur + acc_spring + acc_wind + Matrix.Row(gravity) * dt;

        x_pre = x_cur; // 对下一帧来说，当前帧的结果就是 “上一帧的结果”
        x_cur = x_new; // 当前的结果更新为应用verlet后的 “新结果”

        //center = Matrix.Row(transform.position);
        //// ball constrains
        //if (ball)
        //{
        //    var diff = x_cur - center; // 计算出的质点的新位置（球体本地坐标）
        //    // 检查顶点是否穿越球体表面（距离少于半径+表面厚度）
        //    // index = 1*n列矩阵 保存有穿越（碰撞）的顶点索引
        //    Matrix indexes = diff.RowVectorLength < radius + skinThickness;
        //    // 将穿越的顶点拉回到球体的表面
        //    // surface = n*3 拉回到表面的坐标
        //    Matrix surface = diff.GetRows(indexes).RowVectorNormalized * (radius + skinThickness);
        //    x_cur.SetRows(indexes, center + surface);
        //}
        // 固定的顶点 (pin constrains)
        pin_idx[1] = _columnsCount - 1;
        if (pins)
        {
            x_pre.SetRows(pin_idx, vs.GetRows(pin_idx));
            x_cur.SetRows(pin_idx, vs.GetRows(pin_idx));
        }
        SetV();
    }
    private void Update()
    {
        if (drawFaces) DrawFaces();
        if (updatePhysics) UpdatePhysics();
    }
    private void DrawFaces()
    {
        for (int i = 0; i < F.rowLength; i++)
        {
            Debug.DrawLine(x_cur.GetVector3((int)F[i][0]), x_cur.GetVector3((int)F[i][1]), color, 0, depthTest);
            Debug.DrawLine(x_cur.GetVector3((int)F[i][1]), x_cur.GetVector3((int)F[i][2]), color, 0, depthTest);
            Debug.DrawLine(x_cur.GetVector3((int)F[i][2]), x_cur.GetVector3((int)F[i][0]), color, 0, depthTest);
        }
    }

    public void Change()
    {
        Start();
    }
    public void Start()
    {
        _rowsCount = rowsCount;
        _columnsCount = columnsCount;
        // 创建r*c个顶点（质点）
        mesh = new Mesh();
        Vector3[] verts = new Vector3[_rowsCount * _columnsCount];
        for (int i = 0; i < _rowsCount; i++)
        {
            for (int j = 0; j < _columnsCount; j++)
            {
                var v = new Vector2(-j * space, -i * space);
                verts[i * _rowsCount + j] = v;
            }
        }
        mesh.vertices = verts;
        var mf = GetComponent<MeshFilter>();
        mf.sharedMesh = mesh; //mesh.triangles

        Init();
    }
    void Init()
    {
        // 所有质点的原本位置
        GetV();

        // nV*3矩阵，储存所有顶点位置
        vs = x_cur;
        x_pre = x_cur;

        // n = 边数
        // n*2矩阵 储存所有边的两个顶点索引
        GetEdges();

        GetFaces();

        GetRestLength();
    }

    private void GetFaces()
    {
        var faces = new List<int[]>();
        // f*3 每个面包含的三个顶点
        F = new Matrix();
        for (int i = 0; i < _rowsCount; i++)
        {
            for (int j = 0; j < _columnsCount; j++)
            {
                if ((i + 1 < _rowsCount) && (j + 1 < _columnsCount))
                {
                    faces.Add(new int[] { i * _rowsCount + j, i * _rowsCount + (j + 1), (i + 1) * _rowsCount + j });
                }
                if ((i + 1 < _rowsCount) && (j - 1 >= 0))
                {
                    faces.Add(new int[] { i * _rowsCount + j, (i + 1) * _rowsCount + j, (i + 1) * _rowsCount + (j - 1) });
                }
            }
        }
        //faces
        F = new Matrix();
        F.elements = new float[faces.Count][];
        var list = new List<int>();
        for (int i = 0; i < faces.Count; i++)
        {
            F.elements[i] = new float[] { faces[i][0], faces[i][1], faces[i][2] };
            list.Add(faces[i][0]);
            list.Add(faces[i][1]);
            list.Add(faces[i][2]);
        }
        mesh.triangles = list.ToArray();
        if (log) Debug.Log(F.ToString("matrix: Faces"));
    }

    void GetEdges()
    {
        //1.连接质点[i, j]与[i + 1, j]，[i, j] 与[i, j + 1] 的弹簧，称为“结构弹簧”；
        //2.连接质点[i, j] 与[i + 1, j + 1]，[i+1, j] 与[i, j + 1] 的弹簧，称为“剪切弹簧”；
        //3.连接质点[i, j] 与[i + 2, j]，[i, j] 与[i, j + 2] 的弹簧，称为“弯曲弹簧”。
        //这三种弹簧分别用于与结构力（拉力或压力）、剪力和弯矩相关的计算。
        var edges = new List<int[]>();
        // 横向结构
        for (int i = 0; i < _rowsCount; i++)
        {
            for (int j = 0; j < _columnsCount - 1; j++)
            {
                edges.Add(new int[] { i * _rowsCount + j, i * _rowsCount + j + 1 });
            }
        }
        // 竖向结构
        for (int j = 0; j < _columnsCount; j++)
        {
            for (int i = 0; i < _rowsCount - 1; i++)
            {
                edges.Add(new int[] { i * _rowsCount + j, (i + 1) * _rowsCount + j });
            }
        }
        //edges
        es = new Matrix();
        es.elements = new float[edges.Count][];
        for (int i = 0; i < edges.Count; i++)
        {
            es.elements[i] = new float[] { edges[i][0], edges[i][1] };
        }
    }
    // 弹簧在静止时的长度
    void GetRestLength()
    {
        // 所有边的 顶点1 和 顶点2 的索引
        colEdgeV1 = es.GetColumn(1);
        colEdgeV2 = es.GetColumn(2);

        if (log) Debug.Log(es.ToString("matrix: es"));
        if (log) Debug.Log(colEdgeV1.ToString("matrix: col1"));
        if (log) Debug.Log(colEdgeV2.ToString("matrix: col2"));
        if (log) Debug.Log(vs.ToString("matrix: vs"));

        // difEdge1_2 = n*3 顶点2 到 顶点1 的向量（距离和方向）
        var difEdge1_2 = vs.GetRows(colEdgeV1) - vs.GetRows(colEdgeV2);
        // L0 = n*1列矩阵 当前每条边的长度
        if (log) Debug.Log(difEdge1_2.ToString("matrix: L0"));
        L0 = difEdge1_2.rowVectorLength; // 向量的长度（模）
        if (log) Debug.Log(L0.ToString("matrix: L0 RowVectorLength"));
    }
    private void SetV()
    {
        var verts = new Vector3[_rowsCount * _columnsCount];
        var o = mesh.vertices;
        int ind;
        for (int i = 0; i < _rowsCount; i++)
        {
            for (int j = 0; j < _columnsCount; j++)
            {
                ind = i * _rowsCount + j;
                for (int k = 0; k < pin_idx.Length; k++)
                {
                    if (ind == pin_idx[k])
                    {
                        verts[ind] = o[ind];
                        goto next;
                    }
                }
                var x = x_cur.elements[ind][0];
                var y = x_cur.elements[ind][1];
                var z = x_cur.elements[ind][2];
                x = float.IsNaN(x) ? 0 : x;
                y = float.IsNaN(y) ? 0 : y;
                z = float.IsNaN(z) ? 0 : z;
                x = Mathf.Clamp(x, -range, range);
                y = Mathf.Clamp(y, -range, range);
                z = Mathf.Clamp(z, -range, range);
                //points[i][j].position = new Vector3(x, y, z);
                var v = new Vector3(x, y, z);
                v = transform.InverseTransformPoint(v);
                verts[ind] = v;
                next:
                continue;
            }
        }
        mesh.vertices = verts;
    }
    private void GetV()
    {
        x_cur = new Matrix();
        x_cur.elements = new float[_rowsCount * _columnsCount][];
        var verts = mesh.vertices;
        for (int i = 0; i < _rowsCount; i++)
        {
            for (int j = 0; j < _columnsCount; j++)
            {
                var v = verts[i * _rowsCount + j];
                v = transform.TransformPoint(v);
                x_cur.elements[i * _rowsCount + j] = new float[] { v.x, v.y, v.z };
            }
        }
    }
    public class SparseMatrix
    {
        public string ToString(string title)
        {
            string s = title + " len:" + RowLength.ToString();
            for (int i = 0; i < RowLength; i++)
            {
                s += "\r\n";
                for (int j = 0; j < ColumnLength; j++)
                {
                    s += " " + GetValue(i, j).ToString();
                }
            }
            return s;
        }
        public SparseMatrix()
        {
            points = new List<SparsePoint>();
        }
        public struct SparseKey
        {
            public int i;
            public int j;
            public SparseKey(int i, int j)
            {
                this.i = i; this.j = j;
            }
            public static bool operator ==(SparseKey left, SparseKey right)
            {
                return left.i == right.i && left.j == right.j;
            }
            public static bool operator !=(SparseKey left, SparseKey right)
            {
                return left.i != right.i || left.j != right.j;
            }
        }
        public class SparsePoint // class
        {
            public SparseKey key = default(SparseKey);
            public float value = 0;
        }
        public List<SparsePoint> points;
        public int RowLength;
        public int ColumnLength;
        public float GetValue(int i, int j)
        {
            return GetValue(new SparseKey(i, j));
        }
        public float GetValue(SparseKey key)
        {
            foreach (var point in points)
            {
                if (point.key == key)
                    return point.value;
            }
            return 0f;
        }
        public void AddValue(int i, int j, float value)
        {
            AddValue(new SparseKey(i, j), value);
        }
        public void AddValue(SparseKey key, float value)
        {
            foreach (var point in points)
            {
                if (point.key == key)
                {
                    point.value += value;
                    return;
                }
            }
            // 没有这个点的话添加到列表里
            var p = new SparsePoint();
            p.key = key;
            p.value = value;
            RowLength = max(RowLength, p.key.i);
            ColumnLength = max(ColumnLength, p.key.j);
            points.Add(p);
        }
        int max(int a, int b)
        {
            if (a >= b) return a;
            else return b;
        }
    }
    // 创建稀疏矩阵 i j v 均为同长度的列矩阵
    private SparseMatrix Sparse(Matrix i, Matrix j, Matrix v)
    {
        SparseMatrix m = new SparseMatrix();
        for (int k = 0; k < i.rowLength; k++)
        {
            m.AddValue((int)i.elements[k][0], (int)j.elements[k][0], v.elements[k][0]);
        }
        return m;
    }
    private Matrix Wind_force(float p)
    {
        throw new NotImplementedException();
    }
    public Vector3 faToV3(params float[] fa)
    {
        return new Vector3(fa[0], fa[1], fa[2]);
    }
    // 获取法线
    private Matrix Normals(Matrix verts, Matrix face)
    {
        var m = new Matrix();
        m.elements = new float[face.rowLength][];
        for (int i = 0; i < face.rowLength; i++)
        {
            var v1 = faToV3(verts[(int)face[i][0]]);
            var v2 = faToV3(verts[(int)face[i][1]]);
            var v3 = faToV3(verts[(int)face[i][2]]);
            var a1 = (v1 - v2).normalized;
            var a2 = (v1 - v3).normalized;
            var n = Vector3.Cross(a1, a2).normalized;
            m[i] = new float[] { n.x, n.y, n.z };
        }
        return m;
    }
    private Matrix Transpose(Matrix v)
    {
        throw new NotImplementedException();
    }
    private Matrix RepmatColumns(Matrix p, int repeat)
    {
        var m = new Matrix();
        m.elements = new float[p.rowLength][];
        for (int i = 0; i < p.rowLength; i++)
        {
            m.elements[i] = new float[p.columnLength * repeat];
            for (int r = 0; r < repeat; r++)
            {
                Array.Copy(p[i], 0, m.elements[i], r * p.columnLength, p.columnLength);
            }
        }
        return m;
    }
    //// 对角矩阵相关
    // 获取矩阵的对角元素
    private Matrix Diag(SparseMatrix sm)
    {
        if (sm.RowLength != sm.ColumnLength) throw new Exception("对角矩阵行列数不相等");
        Matrix n = new Matrix(); // 获取对角元素
        // 返回列矩阵
        n.elements = new float[sm.RowLength][];
        for (int i = 0; i < sm.RowLength; i++)
        {
            n.elements[i] = new float[] { sm.GetValue(i, i) };
        }
        return n;
    }
    private Matrix Diag(Matrix m)
    {
        if (m.rowLength != m.columnLength) throw new Exception("对角矩阵行列数不相等");
        Matrix n = new Matrix(); // 获取对角元素
        // 返回列矩阵
        n.elements = new float[m.rowLength][];
        for (int i = 0; i < m.rowLength; i++)
        {
            n.elements[i] = new float[] { m.elements[i][i] };
        }
        return n;
    }
    // 范数
    private Matrix zeros(int rowLength, int columnLength)
    {
        var m = new Matrix();
        m.elements = new float[rowLength][];
        for (int i = 0; i < rowLength; i++)
        {
            m.elements[i] = new float[columnLength];
            for (int j = 0; j < columnLength; j++)
            {
                m.elements[i][j] = 0;
            }
        }
        return m;
    }
}
