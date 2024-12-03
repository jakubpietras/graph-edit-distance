using LinearAssignment;
using System;
using System.Collections.Generic;
using System.Data.Common;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace aac_project
{
    public class EditPath : ICloneable
    {
        private List<(int, int)> nodeMap;
        private int g1_size, g2_size;
        private int[,] initialCostMatrix;
        public int Count { get; set; }
        public List<int> UnmappedNodesG2
        {
            get
            {
                var mappedNodes = new HashSet<int>(
                    nodeMap.Where(node => node.Item2 >= 0 && node.Item2 < g2_size)
                           .Select(node => node.Item2)
                );
                return Enumerable.Range(0, g2_size).Where(i => !mappedNodes.Contains(i)).ToList();
            }
        }
        public List<int> UnmappedNodesG1
        {
            get
            {
                var mappedNodes = new HashSet<int>(
                    nodeMap.Where(node => node.Item1 < g1_size && node.Item2 >= -1)
                           .Select(node => node.Item1)
                );
                return Enumerable.Range(0, g1_size).Where(i => !mappedNodes.Contains(i)).ToList();
            }
        }
        public List<int> MappedNodesG1
        {
            get
            {
                // Collect all unique Item1 values (nodes from g1 that are mapped)
                return nodeMap
                    .Where(node => node.Item1 >= 0 && node.Item1 < g1_size)
                    .Select(node => node.Item1)
                    .Distinct()
                    .ToList();
            }
        }
        public List<int> MappedNodesG2
        {
            get
            {
                // Collect all unique Item2 values (nodes from g2 that are mapped)
                return nodeMap
                    .Where(node => node.Item2 >= 0 && node.Item2 < g2_size)
                    .Select(node => node.Item2)
                    .Distinct()
                    .ToList();
            }
        }
        public EditPath(int[,] costMatrix, List<(int,int)> currentNodeMap, int graph1_size, int graph2_size)
        {
            g1_size = graph1_size;
            g2_size = graph2_size;
            nodeMap = new List<(int, int)> (currentNodeMap);
            Count = nodeMap.Count;
            initialCostMatrix = (int[,])costMatrix.Clone();
        }
        public bool IsComplete()
        {
            return UnmappedNodesG1.Count == 0 && UnmappedNodesG2.Count == 0;
        }
        public int[,] PartialCostMatrix()
        {
            // columns and rows to omit
            HashSet<int> columnsToOmit = new HashSet<int>();
            HashSet<int> rowsToOmit = new HashSet<int>();
            foreach (var node in nodeMap)
            {
                if (node.Item2 >= 0)    // substitution
                {
                    rowsToOmit.Add(node.Item1);
                    columnsToOmit.Add(node.Item2);
                }
                else if (node.Item2 == -1)   // deletion
                {
                    rowsToOmit.Add(node.Item1);
                    columnsToOmit.Add(g2_size + node.Item1);
                }
                else    // insertion
                {
                    rowsToOmit.Add(g1_size + node.Item1);
                    columnsToOmit.Add(node.Item1);
                }    
            }

            // for each of the operation, one row and one column is removed
            int size = initialCostMatrix.GetLength(0) - columnsToOmit.Count;
            int[,] matrix = new int[size, size];

            // copy the remaining elements from inital cost matrix
            int newRow = 0;
            for (int i = 0; i < initialCostMatrix.GetLength(0); i++)
            {
                if (rowsToOmit.Contains(i))
                    continue;

                int newCol = 0;
                for (int j = 0; j < initialCostMatrix.GetLength(0); j++)
                {
                    if (columnsToOmit.Contains(j))
                        continue;

                    matrix[newRow, newCol] = initialCostMatrix[i, j];
                    newCol++;
                }
                newRow++;
            }
            return matrix;
        }
        public int EstimateEditCost()
        {
            int cost = 0;
            var partialCostMatrix = PartialCostMatrix();
            // g(n) + h(n)
            // g(n) is computed based on the currentNodeMap and initialCostMatrix
            foreach (var node in nodeMap)
            {
                if (node.Item2 >= 0) // substitution
                {
                    cost += initialCostMatrix[node.Item1, node.Item2];
                }
                else if (node.Item2 == -1) // deletion
                {
                    cost += initialCostMatrix[node.Item1, g2_size + node.Item1];
                }
                else // insertion
                {
                    cost += initialCostMatrix[g1_size + node.Item1, node.Item1];
                }
            }
            // h(n) is computed based on the RowAssignment from solver and initialCostMatrix
            var rowAssignment = Solver.Solve(PartialCostMatrix()).RowAssignment; // index is the row, value is the column
            foreach(var row in rowAssignment)
            {
                cost += partialCostMatrix[row, rowAssignment[row]];
            }

            return cost;
        }
        public void AddOperation((int, int) operation)
        {
            nodeMap.Add(operation);
            Count++;
        }
        public object Clone()
        {
            return new EditPath(
                (int[,])initialCostMatrix.Clone(), // Clone 2D array
                new List<(int, int)>(nodeMap), // Clone the node map
                g1_size,
                g2_size
            );
        }
    }
}
