using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node
{
    public bool walkable;
    public Vector3 worldPosition;
    public int gCost;
    public int hCost;

    public int gridX;
    public int gridY;
    public Node parent;

    public Node(bool nwalkable, Vector3 nworldPosition, int ngridX, int ngridY)
    {
        walkable = nwalkable;
        worldPosition = nworldPosition;
        gridX = ngridX;
        gridY = ngridY;
    }
    public int fCost
    {
        get { return gCost + hCost; }
        set { }

    }
}