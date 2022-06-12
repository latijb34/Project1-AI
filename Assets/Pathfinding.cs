using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;


public class Pathfinding : MonoBehaviour
{

    public Transform seeker, target;

    public long EuclidianTime, ManhattanTime, BFSTime, DFSTime, UCSTime;
    public long EuclidianFringeLength = 1, ManhattanFringeLength = 1, BFSFringeLength = 1, DFSFringeLength = 1, UCSFringeLength = 1;
    public long EuclidianExp = 0, ManhattanExp = 0, BFSExp = 0, DFSExp = 0, UCSExp = 0;

    int countManhattan = 0;
    int countEuclidian = 0;
    int countUCS = 0;
    int countBFS = 0;
    int countDFS = 0;

    Grid grid;

    void Awake()
    {
        grid = GetComponent<Grid>();
    }

    void Update()
    {
        BFS(seeker.position, target.position);
        DFS(seeker.position, target.position);
        UCS(seeker.position, target.position);
        AStarEuclidianpath(seeker.position, target.position);
        AStarManhattanpath(seeker.position, target.position);

       
        print("Expanded Nodes : ");
        print("A* Euclidian -> " + EuclidianExp);
        print("A* Manhattan -> " + ManhattanExp);
        print("BFS -> " + BFSExp);
        print("DFS -> " + DFSExp);
        print("UCS -> " + UCSExp);
        print("Fringe Size : ");
        print("A* Euclidian -> " + EuclidianFringeLength);
        print("A* Manhattan -> " + ManhattanFringeLength);
        print("BFS -> " + BFSFringeLength);
        print("DFS -> " + DFSFringeLength);
        print("UCS -> " + UCSFringeLength);
        print("Time : ");
        print("A* Euclidian -> " + EuclidianTime + " ms");
        print("A* Manhattan -> " + ManhattanTime + " ms");
        print("BFS ->  " + BFSTime + " ms");
        print("DFS -> " + DFSTime + " ms");
        print("UCS ->  " + UCSTime + " ms");
    }

    void AStarManhattanpath(Vector3 startPos, Vector3 targetPos)
    {
        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();


        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            ManhattanExp++;
            Node node = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
                {
                    if (openSet[i].hCost < node.hCost)
                        node = openSet[i];
                }
            }

            openSet.Remove(node);
            closedSet.Add(node);

            if (node == targetNode)
            {
                stopwatch.Stop();

                ManhattanTime = stopwatch.ElapsedMilliseconds;
                RetracePathAstarManhattan(startNode, targetNode);
                return;


            }

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newCostToNeighbour = node.gCost + GetDistanceManhattan(node, neighbour);
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = GetDistanceManhattan(neighbour, targetNode);
                    neighbour.parent = node;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }


            }

            ManhattanFringeLength = openSet.Count;


        }
    }

    void AStarEuclidianpath(Vector3 startPos, Vector3 targetPos)
    {
        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();


        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            EuclidianExp++;
            Node node = openSet[0];
            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
                {
                    if (openSet[i].hCost < node.hCost)
                        node = openSet[i];
                }
            }

            openSet.Remove(node);
            closedSet.Add(node);

            if (node == targetNode)
            {
                stopwatch.Stop();

                EuclidianTime = stopwatch.ElapsedMilliseconds;
                RetracePathAstarEuclidian(startNode, targetNode);
                return;


            }

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }

                int newCostToNeighbour = node.gCost + GetDistanceEuclidian(node, neighbour);
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = GetDistanceEuclidian(neighbour, targetNode);
                    neighbour.parent = node;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }


            }

            EuclidianFringeLength = openSet.Count;


        }
    }

    void DFS(Vector3 startPos, Vector3 targetPos)
    {
        // This is an iterative DFS. A recursive one would require too much memory
        // on a large grid. It's only fair to compare the algorithms based on their 
        // best performance.
        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Stack<Node> stack = new Stack<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        stack.Push(startNode);

        while (stack.Count > 0)
        {
            Node node = stack.Pop();
            DFSExp++;
            if (node == targetNode)
            {
                stopwatch.Stop();
                DFSTime = stopwatch.ElapsedMilliseconds;
                RetracePathDFS(startNode, targetNode);
                return;
            }

            closedSet.Add(node);

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }
                if (neighbour.walkable || !stack.Contains(neighbour))
                {
                    closedSet.Add(neighbour);
                    neighbour.parent = node;
                    stack.Push(neighbour);
                }
            }
            if (stack.Count > DFSFringeLength)
            {
                DFSFringeLength = stack.Count;
            }
        }
    }

    void BFS(Vector3 startPos, Vector3 targetPos)
    {

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Queue<Node> queueBFS = new Queue<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        queueBFS.Enqueue(startNode);

        while (queueBFS.Count > 0)
        {
            Node currentNode = queueBFS.Dequeue();
            BFSExp++;
            if (currentNode == targetNode)
            {
                stopwatch.Stop();
                BFSTime = stopwatch.ElapsedMilliseconds;
                RetracePathBFS(startNode, targetNode);
                return;
            }


            closedSet.Add(currentNode);

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }
                if (neighbour.walkable || !queueBFS.Contains(neighbour))
                {
                    closedSet.Add(neighbour);
                    neighbour.parent = currentNode;
                    queueBFS.Enqueue(neighbour);
                }
            }
            if (queueBFS.Count > BFSFringeLength)
            {
                BFSFringeLength = queueBFS.Count;
            }
        }
    }

    void UCS(Vector3 startPos, Vector3 targetPos)
    {

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);

        List<Node> openSet = new List<Node>();
        HashSet<Node> closedSet = new HashSet<Node>();
        openSet.Add(startNode);


        while (openSet.Count > 0)
        {
            Node node = openSet[0];

            UCSExp++;

            for (int i = 1; i < openSet.Count; i++)
            {
                if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost)
                {
                    if (openSet[i].hCost < node.hCost)
                        node = openSet[i];
                }
            }
            openSet.Remove(node);


            if (node == targetNode)
            {
                stopwatch.Stop();
                UCSTime = stopwatch.ElapsedMilliseconds;
                RetracePathUCS(startNode, targetNode);
                return;
            }



            closedSet.Add(node);

            foreach (Node neighbour in grid.GetNeighbours(node))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }
                int newCostToNeighbour = node.gCost;
                if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newCostToNeighbour;
                    neighbour.hCost = 0;
                    neighbour.parent = node;

                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);

                }
                if (openSet.Count > UCSFringeLength)
                {
                    UCSFringeLength = openSet.Count;
                }
            }
        }
    }

    void RetracePathAstarManhattan(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            countManhattan++;
        }
        path.Reverse();
        grid.AStarManhattanPath = path;
    }

    void RetracePathAstarEuclidian(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            countEuclidian++;
        }
        path.Reverse();
        grid.AStarEuclidianPath = path;
    }


    void RetracePathUCS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            countUCS++;
        }
        path.Reverse();
        grid.UCSpath = path;
    }


    void RetracePathBFS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            countBFS++;
        }
        path.Reverse();
        grid.BFSpath = path;
    }


    void RetracePathDFS(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
            countDFS++;
        }
        path.Reverse();
        grid.DFSpath = path;
    }

    int GetDistanceManhattan(Node nodeA, Node nodeB)
    {
        int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);
        return dstX + dstY;
    }

    int GetDistanceEuclidian(Node nodeA, Node nodeB)
    {
        return (int)Mathf.Sqrt((Mathf.Pow(nodeA.gridX - nodeB.gridX, 2) + Mathf.Pow(nodeA.gridY - nodeB.gridY, 2)));
    }
}

