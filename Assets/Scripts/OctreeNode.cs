using System.Collections.Generic;
using UnityEngine;

public class OctreeNode
{
    public Bounds bounds;
    public HashSet<MassPoint> points;
    public OctreeNode[] children;
    public bool isDivided = false;
    private int capacity;
    private int maxDepth;
    private int depth;

    public OctreeNode(Bounds bounds, int capacity = 8, int maxDepth = 6, int depth = 0)
    {
        this.bounds = bounds;
        this.capacity = capacity;
        this.maxDepth = maxDepth;
        this.depth = depth;
        points = new HashSet<MassPoint>();
    }

    public void Insert(MassPoint point)
    {
        if (!bounds.Contains(point.position)) return;

        if (points.Count < capacity || depth >= maxDepth)
        {
            points.Add(point);
        }
        else
        {
            if (!isDivided) Subdivide();

            foreach (var child in children)
                child.Insert(point);
        }
    }
    public List<MassPoint> Query(Bounds range, List<MassPoint> found = null)
    {
        if (found == null) found = new List<MassPoint>();

        if (!bounds.Intersects(range)) return found;

        foreach (var p in points)
        {
            if (range.Contains(p.position))
                found.Add(p);
        }

        if (isDivided)
        {
            foreach (var child in children)
                child.Query(range, found);
        }

        return found;
    }

    private void Subdivide()
    {
        children = new OctreeNode[8];
        Vector3 childSize = bounds.size / 2f;
        Vector3 center = bounds.center;

        for (int i = 0; i < 8; i++)
        {
            Vector3 offset = new Vector3(
                ((i & 1) == 0 ? -1 : 1) * childSize.x / 2f,
                ((i & 2) == 0 ? -1 : 1) * childSize.y / 2f,
                ((i & 4) == 0 ? -1 : 1) * childSize.z / 2f
            );

            Vector3 childCenter = center + offset;
            Bounds childBounds = new Bounds(childCenter, childSize);
            children[i] = new OctreeNode(childBounds, capacity, maxDepth, depth + 1);
        }

        foreach (var p in points)
        {
            foreach (var child in children)
            {
                if (child.bounds.Contains(p.position))
                {
                    child.Insert(p);
                    break;
                }
            }
        }

        points.Clear();
        isDivided = true;
    }

}
