using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FixedLenChain : MonoBehaviour
{
    public float spring = 500f;
    public float drag = 0f;
    void Start()
    {
        var rbs = GetComponentsInChildren<Rigidbody>();
        var prefab = rbs[0].GetComponent<HingeJoint>();
        for (int i = 1; i < rbs.Length - 1; i++)
        {
            var hinge = rbs[i].gameObject.AddComponent<HingeJoint>();
            hinge.connectedBody = rbs[i + 1];
            hinge.useSpring = prefab.useSpring;

            hinge.axis = prefab.axis;
            hinge.autoConfigureConnectedAnchor = prefab.autoConfigureConnectedAnchor;
            hinge.anchor = prefab.anchor;
            hinge.connectedAnchor = prefab.connectedAnchor;
        }
    }
    void Update()
    {
        var hjs = GetComponentsInChildren<HingeJoint>();
        foreach (var hj in hjs)
        {
            var springSetting = hj.spring; // get
            springSetting.spring = spring;
            springSetting.damper = drag;
            hj.spring = springSetting; // apply
        }
    }
}
