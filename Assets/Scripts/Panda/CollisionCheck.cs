using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionCheck : MonoBehaviour
{

    private int collisions;

    private void Start()
    {
        collisions = 0;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.tag == "robot" /*|| collision.collider.tag == "piece"*/)
        {
            collisions += 1;
        }
    }

    public int GetCollisions()
    {
        return collisions;
    }

    public void ResetCollisions()
    {
        collisions = 0;
    }
}
