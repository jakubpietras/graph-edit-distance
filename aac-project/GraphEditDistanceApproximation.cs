using Accord;
using Accord.Math;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace Task_2
{

    public class GraphEditDistanceApproximation
    {
        const int NODE_SUBSTITUTION_COST = 0;
        const int NODE_DELETION_COST = 1;
        const int NODE_INSERTION_COST = 1;
        const int EDGE_DELETION_COST = 1;
        const int EDGE_INSERTION_COST = 1;
        const int INF_VALUE = int.MaxValue;

        //private static void AddToOpen(PriorityQueue<List<int>, int> open, List<int> newMapping, int[,] G1, int[,] G2, bool isDirected, int[,] costMatrix)
        //{
        //    var partialCostMatrix = CreatePartialCostMatrix(costMatrix, newMapping, G1.GetLength(0), G2.GetLength(0));
        //    var editCost = CalculateCost(costMatrix, LinearAssignment.Solver.Solve(partialCostMatrix).ColumnAssignment);
        //    open.Enqueue(newMapping, editCost);
        //}

        //public static int AStar(int[,] G1, int[,] G2, bool isDirected)
        //{
        //    // Exact Graph Edit Distance Algorithm (Structural Pattern Recognition with Graph Edit Distance, Kaspar Riesen)
        //    // DISCLAIMER: Nodes of the search tree are of the format: (mapping, cost)
        //    // where mapping is a list L such that L[0] == v_j indicates a mapping u0 -> v_j
        //    // and cost is the output of the heuristic


        //    var costMatrix = CreateCostMatrix(G1, G2, isDirected);
        //    int n1 = G1.GetLength(0),
        //        n2 = G2.GetLength(0);
        //    int min_cost = int.MaxValue;
            
        //    // 1. initialize OPEN to the empty set
        //    PriorityQueue<List<int>, int> open = new PriorityQueue<List<int>, int>();

        //    int heuristic = 0; // g(n) + h(n)

        //    // 2. for each node w in V_2, insert the substitution {u_1 -> w} into OPEN
        //    for (int w = 0; w < n2; w++)
        //    {
        //        // first mappings are the substitution of 0th node of G1 to w in G2
        //        AddToOpen(open, new List<int> { w }, G1, G2, isDirected, costMatrix);
        //    }

        //    // 3. insert the deletion {u_1 -> eps} into OPEN
        //    AddToOpen(open, new List<int> { -1 }, G1, G2, isDirected, costMatrix);

        //    // ======== MAIN LOOP ========
        //    List<int> currentMapping = new List<int>();
        //    int currentCost = 0;
        //    List<int> nodesToInsert = new List<int>();
        //    List<int> newMapping;
        //    int[,] partialCostMatrix;

        //    // while current edit path is not complete, i.e. does not cover all G2 nodes
        //    while (currentMapping.Count + nodesToInsert.Count != n2)
        //    {
        //        // 5. remove the element with the min value of g(n) + h(n) from OPEN
        //        if (!open.TryDequeue(out currentMapping, out currentCost))
        //        {
        //            break; // shouldn't happen
        //        }

        //        // 6. if the mapping with minimal g(n) + h(n) is complete, it is the output
        //        if (currentMapping.Count + nodesToInsert.Count == n2)
        //        {
        //            min_cost = currentCost;
        //            break;
        //        }
        //        else
        //        {
        //            int k = currentMapping.Count; // how many nodes already mapped
        //            if (k < n1) // still some nodes to be substituted or deleted
        //            {
        //                for (int w = 0; w < n2; w++)
        //                {
        //                    if (!currentMapping.Contains(w))
        //                    {
        //                        newMapping = new List<int>(currentMapping);
        //                        newMapping.Add(w);
        //                        partialCostMatrix = CreatePartialCostMatrix(costMatrix, newMapping, n1, n2);
        //                        open.Enqueue(newMapping, CalculateCost(costMatrix, LinearAssignment.Solver.Solve(partialCostMatrix).ColumnAssignment));
        //                    }
        //                }
        //                newMapping = new List<int>(currentMapping);
        //                newMapping.Add(-1);
        //                partialCostMatrix = CreatePartialCostMatrix(costMatrix, newMapping, n1, n2);
        //                open.Enqueue(newMapping, CalculateCost(costMatrix, LinearAssignment.Solver.Solve(partialCostMatrix).ColumnAssignment));
        //            }
        //            else // complete the edit path with insertions
        //            {
        //                // insert all unmapped nodes from G2
        //                newMapping = new List<int>(currentMapping);
        //                nodesToInsert.Clear();
        //                // somehow add insertion to the mapping?
        //                for (int w = 0; w < n2; w++)
        //                {
        //                    if (!currentMapping.Contains(w))
        //                    {
        //                        nodesToInsert.Add(w);
        //                    }
        //                }
        //                heuristic = CalculateCompleteMappingCost(costMatrix, newMapping, nodesToInsert, n1, n2);
        //                open.Enqueue(newMapping, heuristic);
        //            }
        //        }
        //    }

        //    // after while loop finishes, we should have a complete mapping with minimal cost
        //    return min_cost;
        //}

        public static int Beam(int[,] G1, int[,] G2, bool isDirected, int b)
        {
            int d_best = int.MaxValue,
                n1 = G1.GetLength(0),
                n2 = G2.GetLength(0);

            // 1. Build cost matrix C
            int[,] costMatrix = CreateCostMatrix(G1, G2, isDirected);

            // 2. compute optimal node assignment
            var munkres = new Accord.Math.Optimization.Munkres(ConvertIntArrayToDoubleJagged(costMatrix));
            bool success = munkres.Minimize();
            if (!success)
            {
                return -1;
            }
            var initAssignment = munkres.Solution;

            // 3. d_best is the edit path cost of the current best assignment
            d_best = (int)munkres.Value;

            // 4. initialize open = {node assignment, tree level, edit path cost}
            var open = new PriorityQueue<(double[], int), int>(Comparer<int>.Create((x, y) => y - x));
            open.Enqueue((initAssignment, 0), d_best);

            // 5. while open is not empty
            while (open.Count > 0)
            {
                (double[], int) node;
                int priority;
                // 6. remove first tree node in open
                if (open.TryDequeue(out node, out priority))
                {
                    int treeLevel = node.Item2;
                    var assignment = node.Item1;
                    for (int j = treeLevel; j < n1 + n2; j++)   // 7.
                    {
                        // given (u_q, u_q') and (u_j, u_j'), replace with (u_q, u_j') and (u_j, u_q')
                        var newAssignment = (double[])assignment.Clone();
                        SwapOperations(newAssignment, n1, n2, treeLevel, j);

                        // 9. derive approximate edit distance of the new assignment
                        // TODO: calculate cost based on assignment, change 0 below that number!
                        int newEditDistance = CalculateCost(costMatrix, assignment);

                        // 10. add new tuple to open set
                        open.Enqueue((assignment, treeLevel + 1), newEditDistance);

                        // 11. update d_best
                        if (newEditDistance < d_best)
                        {
                            d_best = newEditDistance;
                        }
                    }
                    while (open.Count > b) // keep only b of the best (with minimal edit cost) currently processed nodes
                    {
                        // 16. remove tree node with highest approximation value (priority) from open
                        open.Dequeue();
                    }
                }
                else
                {
                    break;
                }
            }
            return d_best;
        }

        public static int CalculateCost(int[,] costMatrix, double[] assignment)
        {
            int result = 0;
            for (int i = 0; i < assignment.Length; i++) 
            {
                result += costMatrix[i, (int)assignment[i]];
            }
            return result;
        }

        public static int CalculateCost(int[,] costMatrix, int[] assignment)
        {
            int result = 0;
            for (int i = 0; i < assignment.Length; i++)
            {
                result += costMatrix[i, (int)assignment[i]];
            }
            return result;
        }

        public static void SwapOperations(double[] assignment, int n1, int n2, int q, int j)
        {
            if (IsSubstitution(n1, n2, q, (int)assignment[q]))
            {
                // swap substitution with...
                if (IsSubstitution(n1, n2, j, (int)assignment[j]))
                {
                    // substitution
                    (assignment[q], assignment[j]) = (assignment[j], assignment[q]);
                }
                if (IsDeletion(n1, n2, j, (int)assignment[j]))
                {
                    // deletion
                    int tmp = (int)assignment[j];
                    assignment[j] = assignment[q];
                    assignment[q] = n2 + q;
                    assignment[(int)assignment[q]] = tmp;

                }
                if (IsInsertion(n1, n2, j, (int)assignment[j]))
                {
                    // insertion

                }
            }
            else if (IsInsertion(n1, n2, q, (int)assignment[q]))
            {
                // swap insertion with...
                if (IsSubstitution(n1, n2, j, (int)assignment[j]))
                {
                    // substitution
                }
                if (IsDeletion(n1, n2, j, (int)assignment[j]))
                {
                    // deletion
                }
                // swapping insertion with insertion makes no difference
            }
            else if (IsDeletion(n1, n2, q, (int)assignment[q]))
            {
                // swap deletion with...
                if (IsSubstitution(n1, n2, j, (int)assignment[j]))
                {
                    // substitution
                    assignment[q] = assignment[j];
                    assignment[j] = n2 + j;
                    assignment[n1 + j] = assignment[n2 + q];
                }
                if (IsInsertion(n1, n2, j, (int)assignment[j]))
                {
                    // insertion - produces one substitution
                    assignment[n1 + j] = assignment[q];
                    assignment[q] = j;
                }
                // swapping deletion with deletion makes no difference
            }
            else
            {
                // do nothing - not a valid operation
                return;
            }

        }

        public static bool IsSubstitution(int n1, int n2, int operation_from, int operation_to)
        {
            return operation_from < n1 && operation_to < n2;
        }

        public static bool IsInsertion(int n1, int n2, int operation_from, int operation_to)
        {
            return operation_from < n1 && operation_to >= n2;
        }

        public static bool IsDeletion(int n1, int n2, int operation_from, int operation_to)
        {
            return operation_from >= n1 && operation_to < n2;
        }

        public static int CalculateMunkresValue(int[,] costMatrix)
        {
            var testMunkres = new Accord.Math.Optimization.Munkres(ConvertIntArrayToDoubleJagged(costMatrix));
            testMunkres.Minimize();
            return (int)testMunkres.Value;
        }

        public static double[] CalculateMunkresSolution(int[,] costMatrix)
        {
            var testMunkres = new Accord.Math.Optimization.Munkres(ConvertIntArrayToDoubleJagged(costMatrix));
            testMunkres.Minimize();
            return testMunkres.Solution;
        }

        public static int[,] CreatePartialCostMatrix(int[,] originalCostMatrix, List<int> mapping, int n1, int n2)
        {
            // Partial cost matrix has some assignments already enforced
            int[,] partialCostMatrix = DeepCopy(originalCostMatrix);

            for (int i = 0; i < mapping.Count; i++) // enforce operations that are already in the mapping
            {
                if (mapping[i] >= 0) // substitution
                {
                    int tmpCost = partialCostMatrix[i, mapping[i]];
                    FillRow(partialCostMatrix, i, INF_VALUE);
                    FillColumn(partialCostMatrix, mapping[i], INF_VALUE);
                    partialCostMatrix[i, mapping[i]] = tmpCost;
                }
                else // deletion
                {
                    int tmpCost = partialCostMatrix[i, n2 + i];
                    FillRow(partialCostMatrix, i, INF_VALUE);
                    FillColumn(partialCostMatrix, n2 + i, INF_VALUE);
                    partialCostMatrix[i, n2 + i] = tmpCost;
                }
            }
            // insertions don't appear in the mappings
            return partialCostMatrix;
        }

        //public static int[,] CreatePartialCostMatrix(
        //    int[,] G1,
        //    int[,] G2,
        //    bool isDirected,
        //    List<int> mapping)
        //{
        //    var unmatched = GetUnmatchedNodes(mapping, G1.GetLength(0), G2.GetLength(0));
        //    var unmatchedG1 = unmatched.unmatchedG1;
        //    var unmatchedG2 = unmatched.unmatchedG2;


        //    int n1 = unmatchedG1.Count;
        //    int n2 = unmatchedG2.Count;

        //    int[,] costMatrix = new int[n1 + n2, n1 + n2];

        //    // Substitutions: Unmatched nodes in G1 to unmatched nodes in G2
        //    for (int i = 0; i < n1; i++)
        //    {
        //        int u = unmatchedG1[i];
        //        for (int j = 0; j < n2; j++)
        //        {
        //            int v = unmatchedG2[j];

        //            if (isDirected)
        //            {
        //                int id_u = SumColumn(G1, u); // in-degrees of u
        //                int od_u = SumRow(G1, u);    // out-degrees of u
        //                int id_v = SumColumn(G2, v); // in-degrees of v
        //                int od_v = SumRow(G2, v);    // out-degrees of v
        //                int substitution_cost = Math.Abs(id_u - id_v) + Math.Abs(od_u - od_v);
        //                costMatrix[i, j] = substitution_cost;
        //            }
        //            else
        //            {
        //                int deg_u = SumColumn(G1, u); // Degree of u
        //                int deg_v = SumColumn(G2, v); // Degree of v
        //                int substitution_cost = Math.Abs(deg_u - deg_v);
        //                costMatrix[i, j] = substitution_cost;
        //            }
        //        }
        //    }

        //    // Insertions: Dummy rows for unmatchedG2
        //    for (int i = n1; i < n1 + n2; i++)
        //    {
        //        int v = unmatchedG2[i - n1];
        //        for (int j = 0; j < n2; j++)
        //        {
        //            if (i - n1 == j) // Diagonal, insertion cost of jth node of G2
        //            {
        //                int deg_v = isDirected ? SumColumn(G2, v) + SumRow(G2, v) : SumColumn(G2, v);
        //                costMatrix[i, j] = deg_v * EDGE_INSERTION_COST + NODE_INSERTION_COST;
        //            }
        //            else
        //            {
        //                costMatrix[i, j] = INF_VALUE;
        //            }
        //        }
        //    }

        //    // Deletions: Dummy columns for unmatchedG1
        //    for (int i = 0; i < n1; i++)
        //    {
        //        int u = unmatchedG1[i];
        //        for (int j = n2; j < n1 + n2; j++)
        //        {
        //            if (j - n2 == i) // Diagonal, deletion cost of ith node of G1
        //            {
        //                int deg_u = isDirected ? SumColumn(G1, u) + SumRow(G1, u) : SumColumn(G1, u);
        //                costMatrix[i, j] = deg_u * EDGE_DELETION_COST + NODE_DELETION_COST;
        //            }
        //            else
        //            {
        //                costMatrix[i, j] = INF_VALUE;
        //            }
        //        }
        //    }

        //    // Zeros: Dummy rows to dummy columns (no cost)
        //    for (int i = n1; i < n1 + n2; i++)
        //    {
        //        for (int j = n2; j < n1 + n2; j++)
        //        {
        //            costMatrix[i, j] = 0;
        //        }
        //    }

        //    return costMatrix;
        //}


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

        public static (List<int> unmatchedG1, List<int> unmatchedG2) GetUnmatchedNodes(
    List<int> mapping, int g1Size, int g2Size)
        {
            // List for unmatched nodes in G1 and G2
            List<int> unmatchedG1 = new List<int>();
            List<int> unmatchedG2 = new List<int>();

            // Set of matched nodes in G2
            HashSet<int> matchedG2 = new HashSet<int>();

            // Handle the mapped nodes
            for (int i = 0; i < mapping.Count; i++)
            {
                int g2Node = mapping[i];
                if (g2Node == -1) // Node is not mapped
                {
                    unmatchedG1.Add(i);
                }
                else
                {
                    matchedG2.Add(g2Node);
                }
            }

            // Add remaining unmatched nodes in G1
            for (int i = mapping.Count; i < g1Size; i++)
            {
                unmatchedG1.Add(i);
            }

            // Find unmatched nodes in G2
            for (int j = 0; j < g2Size; j++)
            {
                if (!matchedG2.Contains(j)) // Node in G2 is not mapped
                {
                    unmatchedG2.Add(j);
                }
            }

            return (unmatchedG1, unmatchedG2);
        }


        private static int CalculateCompleteMappingCost(int[,] costMatrix, List<int> mapping, List<int> nodesToInsert, int n1, int n2)
        {
            int cost = 0;
            for (int i = 0; i < mapping.Count; i++)
            {
                if (mapping[i] != -1) // substitution
                {
                    cost += costMatrix[i, mapping[i]];
                }
                else // deletion
                {
                    cost += costMatrix[i, n2 + i];
                }
            }
            foreach (int node in nodesToInsert) // insertion
            {
                cost += costMatrix[n1 + node, node];
            }

            return cost;
        }

        public static void FillRow(int[,] matrix, int row, int value)
        {
            int columns = matrix.GetLength(1);
            for (int col = 0; col < columns; col++)
            {
                matrix[row, col] = value;
            }
        }

        public static void FillColumn(int[,] matrix, int column, int value)
        {
            int rows = matrix.GetLength(0);
            for (int row = 0; row < rows; row++)
            {
                matrix[row, column] = value;
            }
        }

        public static int[,] DeepCopy(int[,] original)
        {
            int rows = original.GetLength(0);
            int cols = original.GetLength(1);
            int[,] copy = new int[rows, cols];

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    copy[i, j] = original[i, j];
                }
            }

            return copy;
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

        public static void PrintMatrix(int[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int cols = matrix.GetLength(1);

            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {
                    Console.Write(matrix[i, j] + "\t");
                }
                Console.WriteLine();
            }
        }

        public static double[][] ConvertIntArrayToDoubleJagged(int[,] input)
        {
            int rows = input.GetLength(0);
            int cols = input.GetLength(1);

            // Create a jagged array
            double[][] result = new double[rows][];

            for (int i = 0; i < rows; i++)
            {
                result[i] = new double[cols];
                for (int j = 0; j < cols; j++)
                {
                    // Convert each element from int to double
                    result[i][j] = (double)input[i, j];
                }
            }

            return result;
        }
    }
}

