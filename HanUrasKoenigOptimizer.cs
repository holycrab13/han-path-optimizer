using Pathfinding;
using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public struct GridNodeSelectionPattern
{
    public bool[,] selectionMatrix;
}

/// <summary>
/// Path Optimizer based on https://ojs.aaai.org/index.php/SOCS/article/view/18541 by Jihee Han et al.
/// </summary>
public class HanUrasKoenigOptimizer : MonoModifier
{
    public override int Order => 0;

    [SerializeField]
    private int _graphIndex = 0;

    private GridGraph _grid;

    [SerializeField]
    private int _cellRetrievalPrecision = 8;

    private GridNodeSelectionPattern[] _patternLookup;

    public override void Apply(Path path)
    {
        if (_grid == null)
        {
            _grid = AstarPath.active.graphs[_graphIndex] as GridGraph;
        }

        if (_grid == null)
        {
            return;
        }

        try
        {
            path.vectorPath = DoStringPulling(path.vectorPath, _grid);
        }
        catch(Exception e)
        {
            Debug.LogError(e.Message);
        }

    }

    private List<Vector3> DoStringPulling(List<Vector3> path, GridGraph grid)
    {
        List<Vector2Int> sampledCells = new List<Vector2Int>();
        List<Vector2Int> blockedCells = new List<Vector2Int>();
        List<Vector3> shortestPath = new List<Vector3>();
        int turn = 0;
        float[] cornerAngles = new float[4];

        shortestPath.Add(path[0]);

        int stuckBro = 0;

        for (int i = 2; i < path.Count; i++)
        {
            Vector3 last = shortestPath.Last();

            Vector3 diff = path[i] - last;
            blockedCells.Clear();

            if (Mathf.Abs(diff.x) > 0.01f && Mathf.Abs(diff.z) > 0.01f)
            {
                sampledCells.Clear();
                FindCellsWithLookup(sampledCells, last, path[i], grid.nodeSize, grid.width);

                foreach (Vector2Int cell in sampledCells)
                {
                    if (IsCellBlocked(cell, grid))
                    {
                        blockedCells.Add(cell);
                    }
                }
            }

            if (blockedCells.Count > 0)
            {
                Vector3 nodeToAdd = Vector3.zero;
                float minAngle = float.MaxValue;

                foreach (Vector2Int blockedCell in blockedCells)
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

                stuckBro++;

                if (stuckBro > 1000)
                {
                    Debug.LogError("HELP ME, I'M STUCK!");
                    return shortestPath;
                }

                i--;
            }
            else
            {
                if (shortestPath.Count > 1)
                {
                    int endIndex = shortestPath.Count - 1;
                    int currentTurn = CalculateTurn(shortestPath[endIndex - 1], shortestPath[endIndex], path[i]);

                    if (currentTurn != turn)
                    {
                        shortestPath.RemoveAt(endIndex);

                        if (shortestPath.Count > 1)
                        {
                            turn = CalculateTurn(shortestPath[endIndex - 2], shortestPath[endIndex - 1], path[i]);
                        }

                        stuckBro++;

                        if (stuckBro > 1000)
                        {
                            Debug.LogError("HELP ME, I'M STUCK, TOO!");
                            return shortestPath;
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

        if (Mathf.Abs(angle) < 0.01f)
        {
            return 0;
        }

        return Math.Sign(Vector2.SignedAngle(v1, v2));
    }

    private Vector3 GetCellCorner(Vector2Int cell, int k)
    {
        float size = _grid.nodeSize;

        if (k == 0)
        {
            return new Vector3((cell.x + 1.5f) * size, 0, (cell.y + 1.5f) * size);
        }

        if (k == 1)
        {
            return new Vector3((cell.x + 1.5f) * size, 0, (cell.y - 0.5f) * size);
        }

        if (k == 2)
        {
            return new Vector3((cell.x - 0.5f) * size, 0, (cell.y - 0.5f) * size);
        }

        if (k == 3)
        {
            return new Vector3((cell.x - 0.5f) * size, 0, (cell.y + 1.5f) * size);
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

    private bool IsCellBlocked(Vector2Int cell, GridGraph grid)
    {
        return !grid.GetNode(cell.x, cell.y)?.Walkable ?? true;
    }

    private float DistToCell(Vector3 sample, int v1, int v2)
    {
        Vector3 cellPos = new Vector3(0.5f + v1, 0.0f, 0.5f + v2);
        return Mathf.Max(Mathf.Abs(sample.x - cellPos.x), Mathf.Abs(sample.z - cellPos.z));
    }

    public void InitializePatternLookup(int gridWidth, float precision)
    {
        int patternCount = gridWidth * (gridWidth + 1) / 2;
        _patternLookup = new GridNodeSelectionPattern[patternCount];

        int x = 0;
        int y = 0;

        for (int i = 0; i < patternCount; i++)
        {
            _patternLookup[i] = new GridNodeSelectionPattern()
            {
                selectionMatrix = CreateSelectionMatrix(x, y, precision)
            };

            x++;

            if (x % gridWidth == 0)
            {
                y++;
                x = y;
            }
        }
    }

    private bool[,] CreateSelectionMatrix(int x, int y, float precision)
    {
        Vector3 bottomLeft = new Vector3(0.5f, 0.0f, 0.5f);
        Vector3 topRight = new Vector3(x + 0.5f, 0.0f, y + 0.5f);

        List<Vector2Int> cells = FindCellsSampled(bottomLeft, topRight, precision);

        bool[,] result = new bool[x + 1, y + 1];

        foreach (Vector2Int cell in cells)
        {
            result[cell.x, cell.y] = true;
        }

        return result;
    }

    public void FindCellsWithLookup(List<Vector2Int> list, Vector3 start, Vector3 end, float nodeSize, int gridWidth)
    {
        if (_patternLookup == null)
        {
            InitializePatternLookup(gridWidth, _cellRetrievalPrecision);
        }

        Vector3 diff = (end - start) / nodeSize;
        int diffX = Mathf.Abs(Mathf.RoundToInt(diff.x));
        int diffY = Mathf.Abs(Mathf.RoundToInt(diff.z));

        int diffMin = Mathf.Min(diffX, diffY);
        int diffMax = Mathf.Max(diffX, diffY);
        bool mirror = diffY > diffX;

        int signX = (int)Mathf.Sign(diff.x);
        int signY = (int)Mathf.Sign(diff.z);

        int patternIndex = diffMax - diffMin + (2 * gridWidth * diffMin + diffMin - diffMin * diffMin) / 2;
        GridNodeSelectionPattern pattern = _patternLookup[patternIndex];

        int x = Mathf.FloorToInt(start.x / nodeSize);
        int y = Mathf.FloorToInt(start.z / nodeSize);

        for (int i = 0; i <= diffMax; i++)
        {
            for (int j = 0; j <= diffMin; j++)
            {
                if (pattern.selectionMatrix[i, j])
                {
                    int stepX = mirror ? signX * j : signX * i;
                    int stepY = mirror ? signY * i : signY * j;

                    list.Add(new Vector2Int(x + stepX, y + stepY));
                }
            }
        }
    }

    public List<Vector2Int> FindCellsSampled(Vector3 start, Vector3 end, float precision)
    {
        HashSet<Vector2Int> cells = new HashSet<Vector2Int>();
        Vector3 diff = end - start;

        start += diff * 0.01f;
        end -= diff * 0.01f;

        diff = end - start;

        var stepCount = precision * Mathf.CeilToInt(Mathf.Abs(diff.x) + Mathf.Abs(diff.z));

        if (stepCount == 0)
        {
            stepCount = 1;
        }

        Vector3 sample = start;
        Vector3 step = diff / stepCount;

        for (int i = 0; i <= stepCount; i++)
        {
            Debug.DrawLine(sample, sample + Vector3.up);
            int x = Mathf.FloorToInt(sample.x);
            int y = Mathf.FloorToInt(sample.z);

            for (int m = -1; m <= 1; m++)
            {
                for (int n = -1; n <= 1; n++)
                {
                    if (DistToCell(sample, x + m, y + n) < 1 - 0.01f)
                    {
                        cells.Add(new Vector2Int(x + m, y + n));
                    }
                }
            }

            sample += step;
        }

        return cells.ToList();
    }
}
