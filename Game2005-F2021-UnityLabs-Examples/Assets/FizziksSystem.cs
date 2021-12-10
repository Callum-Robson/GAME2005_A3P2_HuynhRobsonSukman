using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksSystem : MonoBehaviour
{
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public List<FizziksObject> fizziksObjects = new List<FizziksObject>();

    //Smallest distance before things get FUNKY
    public float minimumDistance = 0.0001f;

    //void Start()
    //{
    //    FizziksObject[] objects = FindObjectsOfType<FizziksObject>();
    //    fizziksObjects.AddRange(objects);
    //}

    void FixedUpdate()
    {
        //Velocity update
        for (int i = 0; i < fizziksObjects.Count; i++)
        {
            FizziksObject obj = fizziksObjects[i];
            if (!obj.lockPosition)
            {
                obj.velocity += gravity * obj.gravityScale * Time.fixedDeltaTime;
            }
        }

        //Position update
        for (int i = 0; i < fizziksObjects.Count; i++)
        {
            FizziksObject obj = fizziksObjects[i];

            if (!obj.lockPosition)
            {
                obj.transform.position += obj.velocity * Time.fixedDeltaTime;
            }
        }


        CollisionUpdate();
    }

    void CollisionUpdate()
    {
        for (int objectIndexA = 0; objectIndexA < fizziksObjects.Count; objectIndexA++)
        {
            for (int objectIndexB = objectIndexA + 1; objectIndexB < fizziksObjects.Count; objectIndexB++)
            {
                FizziksObject objectA = fizziksObjects[objectIndexA];
                FizziksObject objectB = fizziksObjects[objectIndexB];

                //If one does not have a collider...
                if (objectA.shape == null || objectB.shape == null)
                {
                    continue;
                }

                //If both are spheres...
                //GetCollisionShape is defined in the base class to allow us to determine what derived classes to cast to
                if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                {
                    //FizziksObject.shape is a base class reference to FizziksColliderBase,
                    //but to do specific things with it, we need to cast to our derived class FizziksColliderSphere
                    SphereSphereCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderSphere)objectB.shape);
                }

                if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Plane)
                {
                    SpherePlaneCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderPlane)objectB.shape);
                }

                if (objectA.shape.GetCollisionShape() == CollisionShape.Plane &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                {
                    SpherePlaneCollision((FizziksColliderSphere)objectB.shape, (FizziksColliderPlane)objectA.shape);
                }

                if (objectA.shape.GetCollisionShape() == CollisionShape.AABB &&
                    objectB.shape.GetCollisionShape() == CollisionShape.AABB)
                {
                    AABBAABBCollision((FizziksColliderAABB)objectA.shape, (FizziksColliderAABB)objectB.shape);
                }

                //if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                //  objectB.shape.GetCollisionShape() == CollisionShape.AABB)
                //{
                //    SphereAABBCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderAABB)objectB.shape);
                //}

                //if (objectA.shape.GetCollisionShape() == CollisionShape.AABB &&
                // objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                //{
                //    SphereAABBCollision((FizziksColliderSphere)objectB.shape, (FizziksColliderAABB)objectA.shape);
                //}
            }
        }
    }

    //In C++ we can return more than one thing at a time using reference parameters with &
    //In C#, we can define "out" parameters, which allows us to return more than one thing
    void GetLockedMovementScalars(FizziksObject a, FizziksObject b, out float movementScalarA, out float movementScalarB)
    {
        //If A is locked and B is not
        // A*0
        // B*1
        if (a.lockPosition && !b.lockPosition)
        {
            movementScalarA = 0.0f;
            movementScalarB = 1.0f;
            return;
        }

        //If B is locked and A is not
        // A*1
        // B*0
        if (!a.lockPosition && b.lockPosition)
        {
            movementScalarA = 1.0f;
            movementScalarB = 0.0f;
            return;
        }

        //If neither are locked
        // A*0.5
        // B*0.5
        if (!a.lockPosition && !b.lockPosition)
        {
            movementScalarA = 0.5f;
            movementScalarB = 0.5f;
            return;
        }

        //If both are locked
        // A*0.0
        // B*0.0
        movementScalarA = 0.0f;
        movementScalarB = 0.0f;
    }

    void AABBAABBCollision(FizziksColliderAABB a, FizziksColliderAABB b)
    {
        Vector3 halfSizeA = a.GetHalfSize();
        Vector3 halfSizeB = b.GetHalfSize();

        //Get displacement between the boxes
        Vector3 displacementAB = b.transform.position - a.transform.position;

        //Find the length of their projections along each axis
        //Compare the distance between them along each axis with half their size in that axis
        float penetrationX = (halfSizeA.x + halfSizeB.x - Mathf.Abs(displacementAB.x));
        float penetrationY = (halfSizeA.y + halfSizeB.y - Mathf.Abs(displacementAB.y));
        float penetrationZ = (halfSizeA.z + halfSizeB.z - Mathf.Abs(displacementAB.z));

        //If for all axes the distance(along that axis) is less than the sum of their half-sizes(along that axis) then they must be overlapping
        if (penetrationX < 0 || penetrationY < 0 || penetrationZ < 0)
        {
            return; // no collision!
        }

        Vector3 normal = new Vector3(Mathf.Sign(displacementAB.x), 0, 0);
        Vector3 minimumTranslationVectorAtoB = normal * penetrationX;


        //Find the shortest penetration to move along
        //if (penetrationX < penetrationY && penetrationX < penetrationZ)
        //{
        //	normal = new Vector3(Mathf.Sign(displacementAB.x), 0, 0);
        //	minimumTranslationVectorAtoB = normal * penetrationX;
        //}

        if (penetrationY < penetrationX && penetrationY < penetrationZ)
        {
            normal = new Vector3(0, Mathf.Sign(displacementAB.y), 0);
            minimumTranslationVectorAtoB = normal * penetrationY;
        }
        else if (penetrationZ < penetrationX && penetrationZ < penetrationY)
        {
            normal = new Vector3(0, 0, Mathf.Sign(displacementAB.z));
            minimumTranslationVectorAtoB = normal * penetrationZ;
        }
        Vector3 contactPoint = a.transform.position + minimumTranslationVectorAtoB;

        //Find the minimum translation vector to move them
        //Apply displacement to separate them along the shortest path we can
        ApplyMinimumTranslationVector(a.kinematicsObject,
        b.kinematicsObject,
        minimumTranslationVectorAtoB,
        normal,
        contactPoint);
    }

    void SphereSphereCollision(FizziksColliderSphere a, FizziksColliderSphere b)
    { 
        // do sphere-sphere collision detection
        // note: sphere-sphere collision detection is the same as circle-circle.
        // If the distance between spheres is less than the sum of their radii, then they are overlapping
        Vector3 displacment = b.transform.position - a.transform.position;
        float distance = displacment.magnitude;
        float sumRadii = a.radius + b.radius;
        float penetrationDepth = sumRadii - distance;
        bool isOverlapping = penetrationDepth > 0;

        if (!isOverlapping)
        {
            return;
        }

        //Mix colors (just to make it easier to see when objects are touching)
        {
            Debug.Log(a.name + " collided with: " + b.name);
            Color colorA = a.GetComponent<Renderer>().material.color;
            Color colorB = b.GetComponent<Renderer>().material.color;
            a.GetComponent<Renderer>().material.color = Color.Lerp(colorA, colorB, 0.05f);
            b.GetComponent<Renderer>().material.color = Color.Lerp(colorB, colorA, 0.05f);
        }

        Vector3 collisionNormalAtoB;
        if (distance < minimumDistance)
        {
           // distance = minimumDistance;
            collisionNormalAtoB = new Vector3(0, penetrationDepth, 0);
        }
        else
        {
            collisionNormalAtoB = displacment / distance;
        }

        //Our minimum translation vector is the vector we have to move along so the objects no longer overlap
        Vector3 minimumTranslationVectorAtoB = penetrationDepth * collisionNormalAtoB;
        Vector3 contactPoint = a.transform.position + collisionNormalAtoB * a.radius;
        ApplyMinimumTranslationVector(a.kinematicsObject, b.kinematicsObject, minimumTranslationVectorAtoB,
            collisionNormalAtoB, contactPoint);
    }

    void SphereAABBCollision(FizziksColliderSphere a, FizziksColliderAABB b)
    {
        Vector3 displacementAB = b.transform.position - a.transform.position;
        Vector3 halfSizeB = b.GetHalfSize();

        float penetrationX = (halfSizeB.x + a.radius - Mathf.Abs(displacementAB.x));
        float penetrationY = (halfSizeB.y + a.radius - Mathf.Abs(displacementAB.y));
        float penetrationZ = (halfSizeB.z + a.radius - Mathf.Abs(displacementAB.z));

        //If for all axes the distance(along that axis) is less than the sum of their half-sizes(along that axis) then they must be overlapping
        if (penetrationX < 0 || penetrationY < 0 || penetrationZ < 0)
        {
            return; // no collision!
        }

        Vector3 normal = new Vector3(Mathf.Sign(displacementAB.x), 0, 0); ;
        Vector3 minimumTranslationVectorAtoB = normal * penetrationX;


        //Find the shortest penetration to move along
        //if(penetrationX < penetrationY && penetrationX < penetrationZ)
        //{
        //	normal = new vector3(Mathf.Sign(displacementAB.x), 0, 0);
        //	minimumTranslationVectorAtoB = normal * penetrationX;
        //}

        if (penetrationY < penetrationX && penetrationY < penetrationZ)
        {
            normal = new Vector3(0, Mathf.Sign(displacementAB.y), 0);
            minimumTranslationVectorAtoB = normal * penetrationY;
        }
        else if (penetrationZ < penetrationX && penetrationZ < penetrationY)
        {
            normal = new Vector3(0, 0, Mathf.Sign(displacementAB.z));
            minimumTranslationVectorAtoB = normal * penetrationZ;
        }
        Vector3 contactPoint = a.transform.position + minimumTranslationVectorAtoB;

        //Find the minimum translation vector to move them
        //Apply displacement to separate them along the shortest path we can
        ApplyMinimumTranslationVector(a.kinematicsObject,
        b.kinematicsObject,
        minimumTranslationVectorAtoB,
        normal,
        contactPoint);
    }

    void SpherePlaneCollision(FizziksColliderSphere a, FizziksColliderPlane b)
    {

        Vector3 somePointOnThePlane = b.transform.position;
        Vector3 centerOfSphere = a.transform.position;

        //Construct any vector from the plane to the center of the sphere
        Vector3 fromPlaneToSphere = centerOfSphere - somePointOnThePlane;

        //Use dot product to find the length of the projection of the center of the sphere sphere onto the plane normal
        //This gives the shortest distance from the plane to the center of the sphere.
        //The sign of this dot product indicates which side of the normal this fromPlaneToSphere vector is on.
        //If the sign is negative, they point in opposite directions
        //If the sign is positive, they are at least somewhat in the same direction
        float dot = Vector3.Dot(fromPlaneToSphere, b.GetNormal());
       
        //float distance = Mathf.Abs(dot);
        float distance = dot; // Abs(dot) will do plane collision.
                              // Removing the Absolute value calculation technically makes it a collision of a "half-space"

        //If the distance is less than the radius of the sphere, they are overlapping
        float penetrationDepth = a.radius - distance;
        bool isOverlapping = penetrationDepth > 0;

        if (!isOverlapping)
        {
            return;
        }

        //Mix colors (just to make it easier to see when objects are touching)
        {
            Debug.Log(a.name + " collided with: " + b.name);
            Color colorA = a.GetComponent<Renderer>().material.color;
            Color colorB = b.GetComponent<Renderer>().material.color;
            a.GetComponent<Renderer>().material.color = Color.Lerp(colorA, colorB, 0.05f);
            b.GetComponent<Renderer>().material.color = Color.Lerp(colorB, colorA, 0.05f);
        }

        Vector3 normal = -b.GetNormal();
        Vector3 mtv = normal * penetrationDepth;
        Vector3 contact = centerOfSphere + (dot * normal);

        Debug.DrawLine(centerOfSphere, contact);
        Debug.DrawRay(contact, normal, Color.red);

        ApplyMinimumTranslationVector(
            a.kinematicsObject,
            b.kinematicsObject,
            mtv,
            normal,
            contact);
    }
  
    private void ApplyMinimumTranslationVector(FizziksObject a, FizziksObject b, Vector3 minimumTranslationVectorAtoB, Vector3 normal, Vector3 contactPoint)
    {
        GetLockedMovementScalars(a, b, out float movementScalarA, out float movementScalarB);

        Vector3 translationVectorA = -minimumTranslationVectorAtoB * movementScalarA;
        Vector3 translationVectorB = minimumTranslationVectorAtoB * movementScalarB;

        a.transform.position += translationVectorA;
        b.transform.position += translationVectorB;

        CollisionInfo collisionInfo;
        collisionInfo.objectA = a.shape;
        collisionInfo.objectB = b.shape;
        collisionInfo.collisionNormalAtoB = normal;
        collisionInfo.contactPoint = contactPoint;

        ApplyKinematicsCollisionResponse(collisionInfo);
    }

    void ApplyKinematicsCollisionResponse(CollisionInfo collision)
    {
        FizziksObject objectA = collision.objectA.kinematicsObject;
        FizziksObject objectB = collision.objectB.kinematicsObject;

        //Find relative velocity between objects
        //Velocity of B relative to A
        Vector3 relativeVelocityAtoB = objectB.velocity - objectA.velocity;

        float relativeNormalVelocityAtoB = Vector3.Dot(relativeVelocityAtoB, collision.collisionNormalAtoB);

        //Determine coefficient of restitution

        float restitution = 0.5f * (objectA.bounciness + objectB.bounciness);
        float changeInVelocity = -relativeNormalVelocityAtoB * (1.0f + restitution);

        //Early exit if the objects are already moving away from each other
        //-->N
        //-	  +
        //A	  B 	    relativeNormalVelocityAtoB = 0  no bounce
        //A       B-->  relativeNormalVelocityAtoB = 2  no bounce
        //<--A    B     relativeNormalVelocityAtoB = 2  no bounce
        //<--A B-->	    relativeNormalVelocityAtoB = 4  no bounce
        //A->  B--->	relativeNormalVelocityAtoB = 2  no bounce
        //A--> <--B	    relativeNormalVelocityAtoB =-4 bounce
        //A-->    B	    relativeNormalVelocityAtoB =-2 bounce 
        //A--->   B->	relativeNormalVelocityAtoB =-2 bounce  
        //<-A   <--B	relativeNormalVelocityAtoB =-1 bounce
        if (relativeNormalVelocityAtoB >= 0.0f)
        {
            return; // no bounce
        }
        //Handle different cases based on which objects are locked
        //Determine impulse (Force * time) = kg * m/sec^2 * sec = kg * m/sec
        //Apply the impulse to each object

        if (objectB.lockPosition && !objectA.lockPosition)
        {
            //(impulse = changeInVelocity * objectA.mass
            float impulse = changeInVelocity * objectA.mass;
            //only A changes velocity
            //Debug.Log("ObjectA velocity = " + objectA.velocity);
            objectA.velocity -= collision.collisionNormalAtoB * (impulse / objectA.mass);
            //Debug.Log("ObjectA velocity = " + objectA.velocity);
            //Debug.Log("ObjectA velocity = " + objectA.velocity);
        }
        else if (!objectB.lockPosition && objectA.lockPosition)
        {
            float impulse = changeInVelocity * objectB.mass;
            //only B changes velocity
            objectB.velocity += collision.collisionNormalAtoB * (impulse / objectB.mass);
        }
        else if (!objectB.lockPosition && !objectA.lockPosition)
        {
            float impulse = changeInVelocity / (1.0f / objectB.mass + 1.0f / objectA.mass);
            objectA.velocity -= collision.collisionNormalAtoB * (impulse / objectA.mass); // ???
            objectB.velocity += collision.collisionNormalAtoB * (impulse / objectB.mass); // ???
            //both change velocity
        }

        Vector3 relativeSurfaceVelocity = relativeVelocityAtoB - (relativeNormalVelocityAtoB * collision.collisionNormalAtoB);
        ApplyFriction(collision, relativeSurfaceVelocity);
    }

    void ApplyFriction(CollisionInfo collision, Vector3 relativeSurfaceVelocity)
    {
       
            //Need two objects
            FizziksObject a = collision.objectA.kinematicsObject;
            FizziksObject b = collision.objectB.kinematicsObject;

            float relativeSpeed = relativeSurfaceVelocity.magnitude;
            float minSpeedToApplyFriction = 0.001f;

            if (relativeSpeed < minSpeedToApplyFriction)
            {
                return;
            }

            //Normal
            //Force along the Normal (using only gravity)
            //Relative Velocity along the common surfaces
            Vector3 directionOfFriction = relativeSurfaceVelocity / relativeSpeed;
            //Debug.Log("Relative Surfave Velocity.Magnitude: " + relativeSurfaceVelocity.magnitude);
            //Debug.Log("Relative Surfave Velocity: " + relativeSurfaceVelocity);
            //Debug.Log("Relative Speed: " + relativeSpeed);

            //Choose a coefficient of Friction (can choose in different ways as long as it depends on both objects)
            //Can choose based on some average(), or a min(), or max(), or lookup from a table
            float kFrictionCoefficient = 0.5f * (a.frictioniness + b.frictioniness);

            //Magnitude of friction force is coefficient of friction times the normal force
            float gravityDotNormal = Vector3.Dot(gravity, collision.collisionNormalAtoB); // Equivalent to force / mass
            Vector3 accelerationFriction = (gravityDotNormal * kFrictionCoefficient * directionOfFriction);
           

            //Apply the frictional force to objects (probably also take into account the "Lock Position"

            //Technically not correct for all cases, but should look okay

            //if (!a.lockPosition && a.shape.GetCollisionShape() == CollisionShape.Sphere)
            //{
            //    a.velocity += accelerationFriction * Time.fixedDeltaTime;
            //    Debug.Log("Direction Friction " + directionOfFriction);
            //    Debug.Log("Acceleration Friction " + accelerationFriction);
            //}
            if (!a.lockPosition)
            {
               a.velocity += accelerationFriction * Time.fixedDeltaTime;
               Debug.Log("Friction updating velocity A");
            }
            if (!b.lockPosition)
            {
                b.velocity += accelerationFriction * Time.fixedDeltaTime;
                Debug.Log("Friction updating velocity B");
            }
    } 
}