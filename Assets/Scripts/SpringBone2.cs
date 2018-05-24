//
//SpringBone.cs for unity-chan!
//
//Original Script is here:
//ricopin / SpringBone.cs
//Rocket Jump : http://rocketjump.skr.jp/unity3d/109/
//https://twitter.com/ricopin416
//
//Revised by N.Kobayashi 2014/06/20
//
using UnityEngine;
using System.Collections;

namespace UnityChan
{
    public class SpringBone2 : MonoBehaviour
    {
        //次のボーン
        public Transform child;

        //ボーンの向き
        public Vector3 boneAxis = new Vector3(-1.0f, 0.0f, 0.0f);
        public float radius = 0.05f;

        //各SpringBoneに設定されているstiffnessForceとdragForceを使用するか？
        public bool isUseEachBoneForceSettings = false;

        public float pullForce = 1f;

        //バネが戻る力
        public float stiffnessForce = 0.01f;

        //力の減衰力
        public float dragForce = 0.4f;
        public Vector3 springForce = new Vector3(0.0f, -0.0001f, 0.0f);
        public SpringCollider[] colliders;
        public bool debug = true;
        //Kobayashi:Thredshold Starting to activate activeRatio
        public float threshold = 0.01f;
        private float springLength;
        private Quaternion localRotation;
        private Transform trs;
        private Vector3 currTipPos;
        private Vector3 prevTipPos;
        //Kobayashi
        private Transform org;
        //Kobayashi:Reference for "SpringManager" component with unitychan 
        private SpringManager managerRef;

        private void Awake()
        {
            trs = transform;
            localRotation = transform.localRotation;
            //Kobayashi:Reference for "SpringManager" component with unitychan
            // GameObject.Find("unitychan_dynamic").GetComponent<SpringManager>();
            managerRef = GetParentSpringManager(transform);
        }

        private SpringManager GetParentSpringManager(Transform t)
        {
            var springManager = t.GetComponent<SpringManager>();

            if (springManager != null)
                return springManager;

            if (t.parent != null)
            {
                return GetParentSpringManager(t.parent);
            }

            return null;
        }
        public float springLengthOrigin;

        private void Start()
        {
            springLength = Vector3.Distance(trs.position, child.position); // 弹簧长度
            springLengthOrigin = springLength; // 弹簧本来的长度
            currTipPos = child.position; // 当前子对象的位置
            prevTipPos = child.position; // 上一帧子对象的位置
        }
        
        public void UpdateSpring()
        {
            //更新下当前子位置，此函数在LateUpdate执行的话，方便接收Update里的外力影响。
            currTipPos = child.position;
            //Kobayashi
            org = trs;
            // 将旋转重置
            trs.localRotation = Quaternion.identity * localRotation;
            // delta时间平方

            // 硬度 stiffness
            Vector3 force = trs.rotation * (boneAxis * stiffnessForce);

            // 阻力 drag
            // 向上一帧的相对移动量（反向）(prevTipPos - currTipPos)
            var drag = (prevTipPos - currTipPos) * dragForce;

            force += drag; // 阻力 = 还原到前一帧位置的力

            //force += springForce ; // RandomWind 的 Update 里添加的外力，风力、重力

            //force += pullForce * boneAxis * springLength; 


            // 为了不和和前一帧的值相同
            Vector3 temp = currTipPos;

            // Verlet
            // 与上一帧的相对移动量 (currTipPos - prevTipPos)
            // v(-dt) 也就是上一帧的加速度
            var toCurrent = currTipPos - prevTipPos;
            // 

            // 第一帧时toCurrent为零
            // r(t) += v(-dt) + f
            currTipPos += toCurrent * managerRef.toCurrentFactor + force;

            // （立即）还原到原本的长度
            // 子对象本地坐标
            // 第一帧时 currTipPos 是子对象上一帧的位置 trs.position 是当前帧自身的位置
            // tipLocal 就是自身移动后，上一帧的子相对自身的位置
            // 新的transform的位置
            var tipLocal = currTipPos - trs.position;
            // pullForce = 弹簧长度还原的速度
            var len = tipLocal.magnitude + pullForce;
            len *= springLengthOrigin - tipLocal.magnitude;
            var sign = Mathf.Sign(springLengthOrigin - tipLocal.magnitude);
            len = tipLocal.magnitude + sign * pullForce;
            springLength = springLengthOrigin;// len;

            Vector3 pull = tipLocal.normalized * sign * pullForce;

            tipLocal = tipLocal.normalized * springLength;
            // 当前子的虚拟位置
            currTipPos = trs.position + tipLocal;// + pull;

            // 计算前的子位置
            prevTipPos = temp;

            // 将旋转应用
            Vector3 aimVector = trs.TransformDirection(boneAxis);
            // 从轴向旋转至子对象的本地方向
            Quaternion aimRotation = Quaternion.FromToRotation(aimVector, currTipPos - trs.position);
            Quaternion secondaryRotation = aimRotation * trs.rotation;
            trs.rotation = Quaternion.Lerp(org.rotation, secondaryRotation, managerRef.dynamicRatio);

            child.position = Vector3.Lerp(child.position, currTipPos, managerRef.dynamicRatio);
        }

        private void OnDrawGizmos()
        {
            if (debug)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(currTipPos, radius);
            }
        }
    }
}
