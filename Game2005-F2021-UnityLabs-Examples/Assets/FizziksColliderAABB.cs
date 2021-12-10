using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksColliderAABB : FizziksColliderBase
{
    public Vector3 dimensions = new Vector3(1, 1, 1);

    public Vector3 GetMin()
    {
        return transform.position - GetHalfSize();
    }
    public Vector3 GetMax()
    {
        return transform.position + GetHalfSize();
    }
    public Vector3 GetSize()
    {
        return Vector3.Scale(dimensions, transform.lossyScale);
    }
    public Vector3 GetHalfSize()
    {
        return Vector3.Scale(dimensions, transform.lossyScale) * 0.5f;
    }
    public override CollisionShape GetCollisionShape()
    {
        return CollisionShape.AABB;
    }
}
