using System.Collections.Generic;
using UnityEngine;

public static class MeshUtils
{
    public static List<int> WeldVertices(Vector3[] vertices, float tolerance, out List<Vector3> weldedVertices)
    {
        weldedVertices = new List<Vector3>();
        List<int> mapping = new List<int>(new int[vertices.Length]);

        Dictionary<Vector3Int, int> grid = new Dictionary<Vector3Int, int>();

        float invTolerance = 1f / tolerance;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 v = vertices[i];

            Vector3Int key = new Vector3Int(
                Mathf.RoundToInt(v.x * invTolerance),
                Mathf.RoundToInt(v.y * invTolerance),
                Mathf.RoundToInt(v.z * invTolerance)
            );

            if (grid.TryGetValue(key, out int weldedIndex))
            {
                mapping[i] = weldedIndex;
            }
            else
            {
                weldedIndex = weldedVertices.Count;
                grid.Add(key, weldedIndex);
                weldedVertices.Add(v);
                mapping[i] = weldedIndex;
            }
        }

        return mapping;
    }
}
