using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class SurfaceSpringSystem : MonoBehaviour
{
    public float pointSize = 0.05f;
    public Material pointMaterial;

    [Header("SerializeField")]
    public float springMaxDistance = 0.3f;
    public float internalSpringMaxDistance = 0.5f;
    public int surfacePointStep = 3;
    public int maxNeighbors = 6;




    [Header("Physics")]
    public float gravity = 9.81f;
    public float structuralSpringStiffness = 100f;
    public float shearSpringStiffness = 50f;
    public float bendingSpringStiffness = 25f;
    public float springDamping = 2f;


    [Header("Voxelization Settings")]
    public float voxelSize = 0.2f;
    public bool visualizeVoxels = false;
    public float groundY = 0f;
    public float restitution = 0.5f;

    private Mesh pointMesh;
    private Material defaultMat;

    private List<MassPoint> internalPoints = new List<MassPoint>();
    private List<MassPoint> massPoints = new List<MassPoint>();
    private List<Spring> springs = new List<Spring>();
    private HashSet<string> springSet = new HashSet<string>();
    Dictionary<Vector3Int, float> surfaceDistanceCache = new Dictionary<Vector3Int, float>();
    List<Triangle> cachedWorldTriangles = null;


    private Vector3[] forcesA;
    private Vector3[] forcesB;

    private OctreeNode octree;
    private OctreeNode surfaceOctree;



    void Start()
    {
        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        if (mesh == null)
        {
            Debug.LogError("❌ لا يوجد Mesh.");
            return;
        }

        GenerateSurfacePointsAndSprings(mesh);

        Destroy(GetComponent<MeshRenderer>());
        Destroy(GetComponent<MeshFilter>());

        Debug.Log($"✅ Created {massPoints.Count} points and {springs.Count} springs.");

        GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        pointMesh = temp.GetComponent<MeshFilter>().sharedMesh;
        Destroy(temp);

        defaultMat = new Material(Shader.Find("Standard"));

        if (visualizeVoxels)
        {
            GenerateInternalPoints(mesh);
        }


        Debug.Log("Total vertices in mesh: " + mesh.vertexCount);
        Debug.Log("Total mass points created: " + massPoints.Count);

        forcesA = new Vector3[springs.Count];
        forcesB = new Vector3[springs.Count];
    }

    void GenerateSurfacePointsAndSprings(Mesh mesh)
    {
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        List<Vector3> weldedVertices;
        List<int> mapping = MeshUtils.WeldVertices(vertices, 0.001f, out weldedVertices);

        massPoints.Clear();
        HashSet<int> usedVertexIndices = new HashSet<int>();
        foreach (int index in triangles)
            usedVertexIndices.Add(mapping[index]);

        int id = 0;
        foreach (int i in usedVertexIndices)
        {
            Vector3 worldPos = transform.TransformPoint(weldedVertices[i]);
            MassPoint mp = new MassPoint(worldPos) { id = id++ };
            massPoints.Add(mp);
        }

        if (massPoints.Count == 0)
        {
            Debug.LogWarning("No surface points generated from mesh.");
            return;
        }

        Bounds bounds = new Bounds(massPoints[0].position, Vector3.zero);
        foreach (var mp in massPoints) bounds.Encapsulate(mp.position);
        bounds.Expand(springMaxDistance);
        surfaceOctree = new OctreeNode(bounds, 8, 6);
        foreach (var mp in massPoints) surfaceOctree.Insert(mp);

        foreach (var mp in massPoints)
        {
            Bounds searchBounds = new Bounds(mp.position, Vector3.one * springMaxDistance * 2f);
            var neighbors = surfaceOctree.Query(searchBounds);
            List<(MassPoint, float)> sortedNeighbors = new();

            foreach (var other in neighbors)
            {
                if (mp.id >= other.id) continue;
                float distSqr = (mp.position - other.position).sqrMagnitude;
                if (distSqr < springMaxDistance * springMaxDistance)
                    sortedNeighbors.Add((other, distSqr));
            }

            sortedNeighbors.Sort((a, b) => a.Item2.CompareTo(b.Item2));
            int count = Mathf.Min(maxNeighbors, sortedNeighbors.Count);
            for (int i = 0; i < count; i++)
                AddUniqueSpring(mp, sortedNeighbors[i].Item1, SpringType.Structural);
        }

        Debug.Log($"🔢 Total unique surface points created: {massPoints.Count}");
    }





    void GenerateInternalPoints(Mesh mesh)
    {
        internalPoints.Clear();
        octree = null;

        Bounds bounds = new Bounds(massPoints[0].position, Vector3.zero);
        foreach (var mp in massPoints) bounds.Encapsulate(mp.position);
        Matrix4x4 localToWorld = transform.localToWorldMatrix;

        octree = new OctreeNode(bounds, 12, 5);
        surfaceOctree = new OctreeNode(bounds, 12, 5);
        foreach (var mp in massPoints) surfaceOctree.Insert(mp);

        if (cachedWorldTriangles == null)
        {
            cachedWorldTriangles = new List<Triangle>();
            Vector3[] vertices = mesh.vertices;
            int[] tris = mesh.triangles;
            for (int i = 0; i < tris.Length; i += 3)
            {
                Vector3 v0 = localToWorld.MultiplyPoint3x4(vertices[tris[i]]);
                Vector3 v1 = localToWorld.MultiplyPoint3x4(vertices[tris[i + 1]]);
                Vector3 v2 = localToWorld.MultiplyPoint3x4(vertices[tris[i + 2]]);
                cachedWorldTriangles.Add(new Triangle(v0, v1, v2));
            }
        }

        HashSet<Vector3Int> visitedVoxels = new();
        int internalCount = 0;

        for (float x = bounds.min.x; x <= bounds.max.x;)
        {
            for (float y = bounds.min.y; y <= bounds.max.y;)
            {
                for (float z = bounds.min.z; z <= bounds.max.z;)
                {
                    Vector3 point = new Vector3(x, y, z);
                    float distToSurface = DistanceToNearestSurface(point);
                    float adaptiveSize = GetAdaptiveVoxelSize(distToSurface);

                    if (distToSurface > springMaxDistance * 1.5f)
                    {
                        z += adaptiveSize;
                        continue;
                    }

                    if (IsPointInsideMesh(point, cachedWorldTriangles))
                    {
                        Vector3Int key = Vector3Int.RoundToInt(point / (voxelSize * 0.5f));
                        if (!visitedVoxels.Contains(key))
                        {
                            visitedVoxels.Add(key);
                            MassPoint mp = new MassPoint(point);
                            internalPoints.Add(mp);
                            octree.Insert(mp);
                            internalCount++;
                        }
                    }

                    z += adaptiveSize;
                }
                y += voxelSize;
            }
            x += voxelSize;
        }

        Debug.Log($"✅ Internal points generated: {internalCount}");

        int countBefore = springs.Count;
        foreach (var mp in internalPoints)
        {
            Bounds b = new Bounds(mp.position, Vector3.one * internalSpringMaxDistance * 2f);
            var neighbors = octree.Query(b);
            foreach (var other in neighbors)
            {
                if (other == mp) continue;
                float dist = Vector3.Distance(mp.position, other.position);
                if (dist < internalSpringMaxDistance)
                    AddUniqueSpring(mp, other, SpringType.Shear);
            }
        }
        int internalSpringCount = springs.Count - countBefore;

        countBefore = springs.Count;
        foreach (var surf in massPoints)
        {
            Bounds b = new Bounds(surf.position, Vector3.one * internalSpringMaxDistance);
            var nearbyInternals = octree.Query(b);
            foreach (var mp in nearbyInternals)
            {
                float dist = Vector3.Distance(mp.position, surf.position);
                if (dist < internalSpringMaxDistance)
                    AddUniqueSpring(mp, surf, SpringType.Bending);
                else
                    Debug.LogWarning($"⚠️ Surface point too far: {dist} > {internalSpringMaxDistance}");
            }
        }
        int surfaceSpringCount = springs.Count - countBefore;

        Debug.Log($"🔗 Internal-Internal Springs: {internalSpringCount}");
        Debug.Log($"🔗 Internal-Surface Springs: {surfaceSpringCount}");
    }



    bool IsPointInsideMesh(Vector3 point, List<Triangle> triangles)
    {
        int hitCount = 0;
        Vector3 rayDirection = Vector3.right;

        foreach (var tri in triangles)
        {
            if (RayIntersectsTriangle(point, rayDirection, tri, out float distance))
            {
                if (distance >= 0)
                {
                    hitCount++;
                }
            }
        }

        return (hitCount % 2) == 1;
    }
    void Update()
    {
        float deltaTime = 0.02f;

        Parallel.For(0, springs.Count, i =>
        {
            var s = springs[i];
            Vector3 delta = s.b.position - s.a.position;
            float dist = delta.magnitude;
            if (dist == 0) return;

            Vector3 direction = delta / dist;
            float forceMag = (dist - s.restLength) * s.stiffness;
            Vector3 force = direction * forceMag;

            forcesA[i] = force;
            forcesB[i] = -force;
        });

        for (int i = 0; i < springs.Count; i++)
        {
            springs[i].a.AddForce(forcesA[i]);
            springs[i].b.AddForce(forcesB[i]);
        }

        Vector3 gravityForce = Vector3.down * gravity;

        Parallel.ForEach(massPoints, mp =>
        {
            mp.AddForce(gravityForce * mp.mass);
        });

        Parallel.ForEach(internalPoints, mp =>
        {
            mp.AddForce(gravityForce * mp.mass);
        });

        Parallel.ForEach(massPoints, mp =>
        {
            mp.UpdatePhysics(deltaTime, groundY, restitution);
        });

        Parallel.ForEach(internalPoints, mp =>
        {
            mp.UpdatePhysics(deltaTime, groundY, restitution);
        });
    }


    void OnDrawGizmos()
    {
        if (massPoints == null || springs == null) return;

        Gizmos.color = Color.green;
        foreach (var mp in massPoints)
        {
            Gizmos.DrawSphere(mp.position, pointSize);
        }

        if (visualizeVoxels && internalPoints != null)
        {
            Gizmos.color = Color.red;
            foreach (var mp in internalPoints)
            {
                Gizmos.DrawSphere(mp.position, pointSize);
            }
        }

        if (springs != null)
        {
            foreach (var s in springs)
            {
                bool aIsInternal = internalPoints != null && internalPoints.Contains(s.a);
                bool bIsInternal = internalPoints != null && internalPoints.Contains(s.b);

                if (aIsInternal && bIsInternal)
                {
                    Gizmos.color = Color.magenta;
                }
                else if (aIsInternal || bIsInternal)
                {
                    Gizmos.color = Color.yellow;
                }
                else
                {
                    Gizmos.color = Color.cyan;
                }

                Gizmos.DrawLine(s.a.position, s.b.position);
            }
        }
    }


    void OnRenderObject()
    {
        if (massPoints == null || pointMesh == null || defaultMat == null) return;

        defaultMat.SetPass(0);
        foreach (var mp in massPoints)
        {
            Graphics.DrawMeshNow(pointMesh, Matrix4x4.TRS(mp.position, Quaternion.identity, Vector3.one * pointSize));
        }
    }

    void AddUniqueSpring(MassPoint a, MassPoint b, SpringType type)
    {
        string key = GetSpringKey(a, b);
        if (springSet.Contains(key)) return;

        float stiffness = GetSpringStiffness(type);
        float damping = GetSpringDamping(type);

        springs.Add(new Spring(a, b, stiffness, damping, type));
        springSet.Add(key);
    }

    float GetSpringStiffness(SpringType type)
    {
        return type switch
        {
            SpringType.Structural => structuralSpringStiffness,
            SpringType.Shear => shearSpringStiffness,
            SpringType.Bending => bendingSpringStiffness,
            _ => 50f
        };
    }

    float GetSpringDamping(SpringType type)
    {
        return type switch
        {
            SpringType.Structural => 0.6f,
            SpringType.Shear => 0.5f,
            SpringType.Bending => 0.3f,
            _ => 0.4f
        };
    }



    string GetSpringKey(MassPoint a, MassPoint b)
    {
        int idA = a.id;
        int idB = b.id;

        if (idA < idB)
            return $"{idA}_{idB}";
        else
            return $"{idB}_{idA}";
    }


    struct Triangle
    {
        public Vector3 v0, v1, v2;
        public Triangle(Vector3 a, Vector3 b, Vector3 c)
        {
            v0 = a; v1 = b; v2 = c;
        }
    }

    bool RayIntersectsTriangle(Vector3 origin, Vector3 direction, Triangle tri, out float t)
    {
        t = 0;
        Vector3 edge1 = tri.v1 - tri.v0;
        Vector3 edge2 = tri.v2 - tri.v0;
        Vector3 h = Vector3.Cross(direction, edge2);
        float a = Vector3.Dot(edge1, h);
        if (a > -1e-5f && a < 1e-5f)
            return false;

        float f = 1.0f / a;
        Vector3 s = origin - tri.v0;
        float u = f * Vector3.Dot(s, h);
        if (u < 0.0f || u > 1.0f)
            return false;

        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(direction, q);
        if (v < 0.0f || u + v > 1.0f)
            return false;

        t = f * Vector3.Dot(edge2, q);
        if (t > 1e-5f)
            return true;

        return false;
    }

    float DistanceToNearestSurface(Vector3 point)
    {
        Vector3Int key = Vector3Int.RoundToInt(point / voxelSize);

        if (surfaceDistanceCache.TryGetValue(key, out float cachedDist))
            return cachedDist;

        float minDist = float.MaxValue;

        Bounds searchBounds = new Bounds(point, Vector3.one * springMaxDistance * 2f);
        var nearbyPoints = surfaceOctree.Query(searchBounds);

        foreach (var surfacePoint in nearbyPoints)
        {
            float dist = Vector3.Distance(point, surfacePoint.position);
            if (dist < minDist)
            {
                minDist = dist;

                if (minDist < voxelSize * 0.25f)
                    break;
            }
        }

        surfaceDistanceCache[key] = minDist;
        return minDist;
    }




    float GetAdaptiveVoxelSize(float distance)
    {
        float minSize = voxelSize * 0.25f;
        float maxSize = voxelSize * 1.2f;

        float threshold = springMaxDistance * 1.5f;

        float t = Mathf.Clamp01(distance / threshold);
        return Mathf.Lerp(minSize, maxSize, t);
    }




}
