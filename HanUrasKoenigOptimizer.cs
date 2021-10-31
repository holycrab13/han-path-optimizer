using Pathfinding;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// Path Optimizer based on https://ojs.aaai.org/index.php/SOCS/article/view/18541 by Jihee Han et al.
/// </summary>
public class HanUrasKoenigOptimizer : MonoModifier
{
    public override int Order => 0;

    [SerializeField]
    private int _graphIndex = 0;

    private GridGraph _grid;

    public override void Apply(Path path)
    {
        if (_grid == null)   
        {
            _grid = AstarPath.active.graphs[_graphIndex] as GridGraph;
        }

        if(_grid == null)
        {
            return;
        }

        path.vectorPath = DoStringPulling(path.vectorPath);
    }

    private List<Vector3> DoStringPulling(List<Vector3> path)
    {
        List<Vector2Int> blockedCells = new List<Vector2Int>();
        List<Vector3> shortestPath = new List<Vector3>();
        int turn = 0;
        float[] cornerAngles = new float[4];

        shortestPath.Add(path[0]);

        for (int i = 2; i < path.Count; i++)
        {
            Vector3 last = shortestPath.Last();

            Vector3 diff = path[i] - last;
            blockedCells.Clear();

            if (Mathf.Abs(diff.x) > 0.01f && Mathf.Abs(diff.z) > 0.01f)
            {
                blockedCells.AddRange(FindCells(last, path[i]).Where(c => c == null || IsCellBlocked(c)));
            }

            if (blockedCells.Count() > 0)
            {
                Vector3 nodeToAdd = Vector3.zero;
                float minAngle = float.MaxValue;

                foreach(Vector2Int blockedCell in blockedCells) 
                {
                    FindCornerAngles(cornerAngles, blockedCell, last, path[i - 1]);

                    for (int k = 0; k < cornerAngles.Length; k++)
                    {
                        if (Mathf.Abs(cornerAngles[k]) <= Mathf.Abs(minAngle))
                        {
                            Vector3 corner = GetCellCorner(blockedCell, k);

                            if (minAngle == cornerAngles[k])
                            {
                                if ((corner - last).sqrMagnitude > (nodeToAdd - last).sqrMagnitude || nodeToAdd.sqrMagnitude < 0.01f)
                                {
                                    nodeToAdd = corner;
                                }
                            }
                            else
                            {
                                nodeToAdd = corner;
                            }

                            minAngle = cornerAngles[k];
                        }
                    }
                }

                shortestPath.Add(nodeToAdd);

                int endIndex = shortestPath.Count - 1;
                turn = CalculateTurn(shortestPath[endIndex - 1], shortestPath[endIndex], path[i]);

                i--;
            }
            else
            {
                if(shortestPath.Count > 1)
                {
                    int endIndex = shortestPath.Count - 1;
                    int currentTurn = CalculateTurn(shortestPath[endIndex - 1], shortestPath[endIndex], path[i]);

                    if (currentTurn != turn)
                    {
                        shortestPath.RemoveAt(endIndex);

                        if(shortestPath.Count > 1)
                        {
                            turn = CalculateTurn(shortestPath[endIndex - 2], shortestPath[endIndex - 1], path[i]); 
                        }

                        i--;
                    }
                }
            }
        }

        shortestPath.Add(path.Last());
        return shortestPath;
    }

    private int CalculateTurn(Vector3 start, Vector3 middle, Vector3 end)
    {
        Vector2 v1 = new Vector2(middle.x - start.x, middle.z - start.z);
        Vector2 v2 = new Vector2(end.x - start.x, end.z - start.z);

        float angle = Vector2.SignedAngle(v1, v2);

        if(Mathf.Abs(angle) < 0.01f)
        {
            return 0;
        }

        return Math.Sign(Vector2.SignedAngle(v1, v2));
    }

    private Vector3 GetCellCorner(Vector2Int cell, int k)
    {
        if(k == 0)
        {
            return new Vector3(cell.x + 0.5f, 0, cell.y + 0.5f);
        }

        if (k == 1)
        {
            return new Vector3(cell.x + 0.5f, 0, cell.y - 0.5f);
        }

        if (k == 2)
        {
            return new Vector3(cell.x - 0.5f, 0, cell.y - 0.5f);
        }

        if (k == 3)
        {
            return new Vector3(cell.x - 0.5f, 0, cell.y + 0.5f);
        }

        return Vector3.zero;
    }

    private void FindCornerAngles(float[] cornerAngles, Vector2Int cell, Vector3 start, Vector3 end)
    {
        Vector3 dir = end - start;

        for (int k = 0; k < 4; k++)
        {
            Vector3 cornerDir = GetCellCorner(cell, k) - start;
            cornerAngles[k] = Vector2.SignedAngle(new Vector2(dir.x, dir.z), new Vector2(cornerDir.x, cornerDir.z));
        }
    }

    private bool IsCellBlocked(Vector2Int cell)
    {
        if (!_grid.GetNode(cell.x, cell.y)?.Walkable ?? true)
        {
            return true;
        }

        if (!_grid.GetNode(cell.x - 1, cell.y)?.Walkable ?? true)
        {
            return true;
        }

        if (!_grid.GetNode(cell.x, cell.y - 1)?.Walkable ?? true)
        {
            return true;
        }

        if (!_grid.GetNode(cell.x - 1, cell.y - 1)?.Walkable ?? true)
        {
            return true;
        }

        return false;
    }

    private List<Vector2Int> FindCells(Vector3 start, Vector3 end)
    {
        List<Vector2Int> cells = new List<Vector2Int>();

        var diffX = end.x - start.x;
        var diffY = end.z - start.z;
        var stepX = Math.Sign(diffX);
        var stepY = Math.Sign(diffY);

        start.x += _grid.nodeSize * 0.5f + stepX * 0.01f;
        start.z += _grid.nodeSize * 0.5f + stepY * 0.01f;
        end.x += _grid.nodeSize * 0.5f - stepX * 0.01f;
        end.z += _grid.nodeSize * 0.5f - stepY * 0.01f;

        // Grid cells are 1.0 X 1.0.
        int x = (int)Math.Floor(start.x);
        int z = (int)Math.Floor(start.z);

        // Ray/Slope related maths.
        // Straight distance to the first vertical grid boundary.
        var xOffset = end.x > start.x ?
            (Math.Ceiling(start.x) - start.x) :
          (start.x - Math.Floor(start.x));
        // Straight distance to the first horizontal grid boundary.
        var yOffset = end.z > start.z ?
            (Math.Ceiling(start.z) - start.z) :
          (start.z - Math.Floor(start.z));
        // Angle of ray/slope.
        var angle = Math.Atan2(-diffY, diffX);
        // NOTE: These can be divide by 0's, but JS just yields Infinity! :)
        // How far to move along the ray to cross the first vertical grid cell boundary.
        var tMaxX = xOffset / Math.Cos(angle);
        // How far to move along the ray to cross the first horizontal grid cell boundary.
        var tMaxY = yOffset / Math.Sin(angle);
        // How far to move along the ray to move horizontally 1 grid cell.
        var tDeltaX = 1.0 / Math.Cos(angle);
        // How far to move along the ray to move vertically 1 grid cell.
        var tDeltaY = 1.0 / Math.Sin(angle);

        // Travel one grid cell at a time.
        var manhattanDistance = Math.Abs(Math.Floor(end.x) - Math.Floor(start.x)) +
            Math.Abs(Math.Floor(end.z) - Math.Floor(start.z));

        for (int t = 0; t <= manhattanDistance; ++t)
        {
            cells.Add(new Vector2Int(x, z));

            // Only move in either X or Y coordinates, not both.
            if (Math.Abs(tMaxX) < Math.Abs(tMaxY))
            {
                tMaxX += tDeltaX;
                x += stepX;
            }
            else
            {
                tMaxY += tDeltaY;
                z += stepY;
            }
        }

        return cells;
    }
}
