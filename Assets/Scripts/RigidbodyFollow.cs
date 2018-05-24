using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RigidbodyFollow : MonoBehaviour
{
    public Transform obj;
    public Transform target;
    void FixedUpdate()
    {
        obj.GetComponent<Rigidbody>().MovePosition(target.position);
    }
}
