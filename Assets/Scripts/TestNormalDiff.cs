using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestNormalDiff : MonoBehaviour
{
    public Transform dir1;
    public Transform dir2;
    public Transform dir3;
    public float proj;
    public float dot;
    public float distance;
    public float a1l = 1;
    public float a2l = 1;

    // Use this for initialization
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        distance = Vector3.Distance(dir1.up, dir2.up);
        proj = Vector3.Project(dir1.up, dir2.up).magnitude;
        // proj.magnitude 似乎能无视长度 得出1~0的值（正对/背对~侧对）
        dot = Vector3.Dot(dir1.up, dir2.up);
        // 而Dot需要是两个归一化向量才能得出 1~0~-1 （正对~侧对~背对）的值
        dir3.up = Vector3.Cross(dir1.up * a1l, dir2.up * a2l); // 从 dir3 正方向观察 dir1和2组成的平面，dir1到dir2是顺时针方向
        Debug.Log(Vector3.Cross(dir1.up * a1l, dir2.up * a2l));
        // Cross的结果的长度似乎也跟dir1和2的长度有关
    }
}
