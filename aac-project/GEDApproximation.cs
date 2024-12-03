using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace aac_project
{
    public class GEDApproximation
    {
        const int NODE_SUBSTITUTION_COST = 0;
        const int NODE_DELETION_COST = 1;
        const int NODE_INSERTION_COST = 1;
        const int EDGE_DELETION_COST = 1;
        const int EDGE_INSERTION_COST = 1;
        const int INF_VALUE = int.MaxValue;
        public static int AStar(int[,] G1, int[,] G2, bool isDirected)
        {
            int d = int.MaxValue,
                g1_size = G1.GetLength(0),
                g2_size = G2.GetLength(0);
            // initial cost matrix
            var costMatrix = CreateCostMatrix(G1, G2, isDirected);

            // 1. initialize OPEN to empty set
            PriorityQueue<EditPath, int> open = new PriorityQueue<EditPath, int>();

            // 2. for each node w in V_2 insert the substitution into OPEN
            // i.e. create |V_2| new edit paths which include single substitution
            for (int w = 0; w < g2_size; w++)
            {
                var newEditPath = new EditPath(costMatrix, new List<(int, int)>() { (0, w) }, g1_size, g2_size);
                open.Enqueue(newEditPath, newEditPath.EstimateEditCost());
            }

            // 3. insert the deletion {u_0 -> eps} into OPEN
            {
                var newEditPath = new EditPath(costMatrix, new List<(int, int)>() { (0, -1) }, g1_size, g2_size);
                open.Enqueue(newEditPath, newEditPath.EstimateEditCost());
            }

            while (open.Count > 0) // 4.
            {
                // 5. remove the arg_min of g(n) + h(n) from OPEN
                int editCost = 0;
                EditPath currentEditPath;
                open.TryDequeue(out currentEditPath, out editCost);

                // 6. if edit path is complete
                if (currentEditPath.IsComplete())
                {
                    // 7. return it as a solution
                    d = currentEditPath.EstimateEditCost();
                    break;
                }

                if (currentEditPath.MappedNodesG1.Count - 1 < g1_size - 1) // 10.
                {
                    // 11. for each unmapped w from G2, insert current path with added substitution to w into OPEN
                    foreach (var w in currentEditPath.UnmappedNodesG2)
                    {
                        var newEditPath = (EditPath)currentEditPath.Clone();
                        newEditPath.AddOperation((currentEditPath.MappedNodesG1.Count, w));
                        open.Enqueue(newEditPath, newEditPath.EstimateEditCost());
                    }
                    {
                        var newEditPath = (EditPath)currentEditPath.Clone();
                        newEditPath.AddOperation((currentEditPath.MappedNodesG1.Count, -1));
                        open.Enqueue(newEditPath, newEditPath.EstimateEditCost());
                    }
                }
                else
                {
                    // 14. insert insertions
                    var newEditPath = (EditPath)currentEditPath.Clone();
                    foreach (var w in currentEditPath.UnmappedNodesG2)
                    {
                        newEditPath.AddOperation((w, -2));
                    }
                    open.Enqueue(newEditPath, newEditPath.EstimateEditCost());
                }
            }
            return d;
        }

        public static int[,] CreateCostMatrix(int[,] G1, int[,] G2, bool isDirected)
        {
            int n1 = G1.GetLength(0);
            int n2 = G2.GetLength(0);

            int[,] costMatrix = new int[n1 + n2, n1 + n2];

            // substitutions
            for (int u = 0; u < n1; u++)
            {
                for (int v = 0; v < n2; v++)
                {
                    if (isDirected)
                    {
                        int id_u = SumColumn(G1, u), // in degrees of u
                            od_u = SumRow(G1, u);   // out degrees of u
                        int id_v = SumColumn(G2, v), // in degrees of v
                            od_v = SumRow(G2, v);   // out degrees of v
                        int substitution_cost = Math.Abs(id_u - id_v) + Math.Abs(od_u - od_v);
                        costMatrix[u, v] = substitution_cost;
                    }
                    else
                    {
                        int deg_u = SumColumn(G1, u);
                        int deg_v = SumColumn(G2, v);
                        int substitution_cost = Math.Abs(deg_u - deg_v);
                        costMatrix[u, v] = substitution_cost;
                    }
                }
            }

            // insertions
            for (int i = n1; i < n1 + n2; i++)
            {
                for (int j = 0; j < n2; j++)
                {
                    if (i - n1 == j) // diagonal, insertion cost of jth node of G2
                    {
                        int deg_j = isDirected ? SumColumn(G2, j) + SumRow(G2, j) : SumColumn(G2, j);
                        costMatrix[i, j] = deg_j * EDGE_INSERTION_COST + NODE_INSERTION_COST;
                    }
                    else
                    {
                        costMatrix[i, j] = INF_VALUE;
                    }
                }
            }

            // deletions
            for (int i = 0; i < n1; i++)
            {
                for (int j = n2; j < n1 + n2; j++)
                {
                    if (j - n2 == i) // diagonal, deletion cost of ith node of G1
                    {
                        int deg_i = isDirected ? SumColumn(G1, i) + SumRow(G1, i) : SumColumn(G1, i);
                        costMatrix[i, j] = deg_i * EDGE_DELETION_COST + NODE_DELETION_COST;
                    }
                    else
                    {
                        costMatrix[i, j] = INF_VALUE;
                    }
                }
            }

            // zeros
            for (int i = n1; i < n1 + n2; i++)
            {
                for (int j = n2; j < n1 + n2; j++)
                {
                    costMatrix[i, j] = 0;
                }
            }

            return costMatrix;
        }

        public static int SumRow(int[,] matrix, int rowIndex)
        {
            int sum = 0;
            int cols = matrix.GetLength(1); // Number of columns in the matrix

            for (int col = 0; col < cols; col++)
            {
                sum += matrix[rowIndex, col];
            }

            return sum;
        }

        public static int SumColumn(int[,] matrix, int colIndex)
        {
            int sum = 0;
            int rows = matrix.GetLength(0); // Number of rows in the matrix

            for (int row = 0; row < rows; row++)
            {
                sum += matrix[row, colIndex];
            }

            return sum;
        }

    }
}
