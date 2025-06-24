using UnityEngine;

public class MassPoint
{
    private static int nextId = 0;

    public int id;
    public Vector3 position;
    public Vector3 velocity;
    public float mass = 1f;
    public float damping = 0.98f;
    public bool isFixed = false;

    private Vector3 forceAccumulator;
    private object forceLock = new object();

    public MassPoint(Vector3 pos)
    {
        id = nextId++;
        position = pos;
        velocity = Vector3.zero;
        forceAccumulator = Vector3.zero;
    }

    public void AddForce(Vector3 force)
    {
        if (isFixed) return;
        lock (forceLock)
        {
            forceAccumulator += force;
        }
    }

    public void UpdatePhysics(float deltaTime, float groundY, float restitution)
    {
        if (isFixed)
        {
            velocity = Vector3.zero;
            forceAccumulator = Vector3.zero;
            return;
        }

        velocity += (forceAccumulator / mass) * deltaTime;

        velocity *= damping;

        position += velocity * deltaTime;

        if (position.y < groundY)
        {
            position.y = groundY;

            if (velocity.y < 0)
            {
                velocity.y = -velocity.y * restitution;

                velocity.x *= 0.9f;
                velocity.z *= 0.9f;
            }
        }
        forceAccumulator = Vector3.zero;
    }


    public override int GetHashCode()
    {
        return id.GetHashCode();
    }
}
