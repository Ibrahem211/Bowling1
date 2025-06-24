using UnityEngine;

public enum SpringType
{
    Structural,
    Shear,
    Bending
}

public class Spring
{
    public MassPoint a, b;
    public float restLength;
    public float stiffness;
    public float damping;
    public SpringType type;

    public Spring(MassPoint a, MassPoint b, float stiffness, float damping, SpringType type)
    {
        this.a = a;
        this.b = b;
        this.stiffness = stiffness;
        this.damping = damping;
        this.type = type;
        this.restLength = Vector3.Distance(a.position, b.position);
    }

    public void ApplySpringForce(float deltaTime)
    {
        Vector3 delta = b.position - a.position;
        float currentLength = delta.magnitude;

        if (currentLength == 0f)
            return;

        Vector3 direction = delta / currentLength;
        float displacement = currentLength - restLength;

        Vector3 springForce = direction * (displacement * stiffness);

        Vector3 relativeVelocity = b.velocity - a.velocity;
        float dampingForceMagnitude = Vector3.Dot(relativeVelocity, direction) * damping;
        Vector3 dampingForce = direction * dampingForceMagnitude;

        Vector3 totalForce = springForce + dampingForce * -1f;

        a.AddForce(totalForce);
        b.AddForce(-totalForce);

    }
}
