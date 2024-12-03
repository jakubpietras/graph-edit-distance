using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Text.RegularExpressions;

namespace Task_2
{
    public class GraphEditDistanceExact
    {
        // edit operations cost
        const int NODE_SUBSTITUTION_COST = 0;
        const int NODE_DELETION_COST = 1;
        const int NODE_INSERTION_COST = 1;
        const int EDGE_DELETION_COST = 1;
        const int EDGE_INSERTION_COST = 1;

        public static int BruteForceGED(int[,] G1, int[,] G2, bool isDirected)
        {
            // graph sizes
            int n1 = G1.GetLength(0);
            int n2 = G2.GetLength(0);

            // each element in mappings array is a list that holds all possible mappings
            // for a given node in G1 (all possible ways to map a single G1 node to G2 nodes)
            // e.g. mapping[0] = [1 0 2 -1] means u0->v1 OR u0->v0 OR u0->v2 OR u0->eps (deletion)
            List<int>[] mappings = new List<int>[n1];

            // mappingCounts holds a number of possible mappings for each node in G1
            int[] mappingCounts = new int[n1];
            for (int i = 0; i < n1; i++)
            {
                mappings[i] = new List<int>();
                for (int j = 0; j < n2; j++)
                {
                    mappings[i].Add(j); // mapping i-th node of G1 to jth node of G2
                }
                mappings[i].Add(-1); // deleting the i-th node
                mappingCounts[i] = mappings[i].Count;
            }

            // total number of mappings: since for one full edit of G1 we're choosing one
            // mapping of 0-th node, one mapping of 1-st node etc... the total number
            // is calculated by the product rule
            long totalMappings = 1;
            for (int i = 0; i < n1; i++)
            {
                totalMappings *= mappingCounts[i];
            }

            int minCost = int.MaxValue;

            // iterate over all possible mappings to find the one with the minimal cost
            for (int mappingIndex = 0; mappingIndex < totalMappings; mappingIndex++)
            {
                int[] currentMapping = new int[n1]; // a single mapping of all nodes from G1->G2
                long tempIndex = mappingIndex;

                // decode the mapping index into individual node mappings
                for (int i = n1 - 1; i >= 0; i--)
                {
                    int optionIndex = (int)(tempIndex % mappingCounts[i]);
                    tempIndex /= mappingCounts[i];
                    currentMapping[i] = mappings[i][optionIndex];
                }

                // validate mapping: Ensure no two nodes in G1 map to the same node in G2
                bool isValidMapping = true;
                HashSet<int> mappedG2Nodes = new HashSet<int>();
                List<int> unmappedNodes = new List<int>();
                for (int i = 0; i < n1; i++)
                {
                    int mappedNode = currentMapping[i];
                    if (mappedNode >= 0)
                    {
                        if (mappedG2Nodes.Contains(mappedNode))
                        {
                            isValidMapping = false;
                            break;
                        }
                        mappedG2Nodes.Add(mappedNode);
                    }
                }
                if (!isValidMapping)
                    continue; // skip invalid mappings

                // total edit cost for the current mapping
                int totalCost = 0;

                // ================ node operation costs ================
                for (int i = 0; i < n1; i++) // iterate over all nodes in G1
                {
                    int mappedNode = currentMapping[i];
                    if (mappedNode >= 0)
                    {
                        // node substitution cost
                        totalCost += NODE_SUBSTITUTION_COST;
                    }
                    else
                    {
                        // node deletion cost (== -1)
                        totalCost += NODE_DELETION_COST;
                    }
                }

                // node insertion cost
                for (int j = 0; j < n2; j++) // iterate over all nodes in G2
                {
                    // if after mapping all G1 nodes, j-th node in G2 has not been mapped, we insert it to G1
                    if (!mappedG2Nodes.Contains(j))
                    {
                        unmappedNodes.Add(j);
                        totalCost += NODE_INSERTION_COST;
                    }
                }

                // ================ edge operation costs (mapped nodes) ================
                // The following considers edges between nodes in G1 that have been mapped to nodes in G2
                if (isDirected)
                {
                    // directed graph - iterate over all pairs of nodes in G1
                    for (int u = 0; u < n1; u++)
                    {
                        for (int v = 0; v < n1; v++)
                        {
                            int mappedU = currentMapping[u];
                            int mappedV = currentMapping[v];
                            int edgeInG1 = G1[u, v];

                            if (mappedU >= 0 && mappedV >= 0)
                            {
                                int edgeInG2 = G2[mappedU, mappedV];
                                if (edgeInG1 > edgeInG2)
                                {
                                    totalCost += (edgeInG1 - edgeInG2) * EDGE_DELETION_COST;
                                }
                                else if (edgeInG2 > edgeInG1)
                                {
                                    totalCost += (edgeInG2 - edgeInG1) * EDGE_INSERTION_COST;
                                }
                                // edgeInG1 == edgeInG2 means substitution, no cost is added
                            }
                            else
                            {
                                // one of the endpoints is deleted, whole edge must be deleted
                                totalCost += edgeInG1 * EDGE_DELETION_COST;
                            }
                        }
                    }
                }
                else
                {
                    // undirected graph - iterate over pairs once (v > u so no pairs are counted twice)
                    for (int u = 0; u < n1; u++)
                    {
                        for (int v = u + 1; v < n1; v++)
                        {
                            int mappedU = currentMapping[u];
                            int mappedV = currentMapping[v];
                            int edgeInG1 = G1[u, v];

                            if (mappedU >= 0 && mappedV >= 0)
                            {
                                int edgeInG2 = G2[mappedU, mappedV];
                                if (edgeInG1 > edgeInG2)
                                {
                                    totalCost += (edgeInG1 - edgeInG2) * EDGE_DELETION_COST;
                                }
                                else if (edgeInG2 > edgeInG1)
                                {
                                    totalCost += (edgeInG2 - edgeInG1) * EDGE_INSERTION_COST;
                                }
                            }
                            else
                            {
                                totalCost += edgeInG1 * EDGE_DELETION_COST;
                            }
                        }
                    }
                }

                // ================ edge operation costs (unmapped nodes) ================
                // the following consider edges between unmapped nodes and both mapped and unmapped nodes. Unmapped nodes
                // are those nodes in G2 that have not been substituted with any nodes from G1. 
                // If size of G1 is smaller than G2, then there must be some unmapped nodes

                foreach (var unmappedNode in unmappedNodes)
                {
                    foreach (var mappedNode in mappedG2Nodes)
                    {
                        if (isDirected)
                        {
                            // edge from unmappedNode to mappedNode
                            int edgeOut = G2[unmappedNode, mappedNode];
                            if (edgeOut > 0)
                            {
                                totalCost += edgeOut * EDGE_INSERTION_COST;
                            }

                            // edge from mappedNode to unmappedNode
                            int edgeIn = G2[mappedNode, unmappedNode];
                            if (edgeIn > 0)
                            {
                                totalCost += edgeIn * EDGE_INSERTION_COST;
                            }
                        }
                        else
                        {
                            // a single edge between an unmappedNode and a mappedNode
                            int edge = G2[unmappedNode, mappedNode];
                            if (edge > 0)
                            {
                                totalCost += edge * EDGE_INSERTION_COST;
                            }
                        }

                    }
                }
                foreach (var u in unmappedNodes)
                {
                    foreach (var v in unmappedNodes)
                    {
                        int edgeOut = G2[u, v];
                        if (edgeOut > 0)
                        {
                            totalCost += edgeOut * EDGE_INSERTION_COST;
                        }
                        // no need to process G2[v, u], as it will be covered when u and v swap
                    }
                }

                // Update minimum cost if the current mapping has a lower total cost
                if (totalCost < minCost)
                {
                    minCost = totalCost;
                }
            }
            return minCost;
        }

        // process the input
        public static int[,] ReadAdjacencyMatrix(string filePath)
        {
            string[] lines = File.ReadAllLines(filePath);
            // remove empty lines and trim whitespace
            lines = Array.FindAll(lines, line => !string.IsNullOrWhiteSpace(line));

            int n = lines.Length;
            int[,] adjacencyMatrix = new int[n, n];

            for (int i = 0; i < n; i++)
            {
                // remove leading/trailing whitespace
                string line = lines[i].Trim();
                // remove the trailing comma if present
                if (line.EndsWith(","))
                {
                    line = line.Substring(0, line.Length - 1);
                }
                // Split the line into tokens (numbers)
                // Use regex to split by comma(s) and optional whitespace
                string[] tokens = Regex.Split(line, @",\s*");

                if (tokens.Length != n)
                {
                    throw new FormatException($"Row {i + 1} does not contain {n} columns.");
                }

                for (int j = 0; j < n; j++)
                {
                    string token = tokens[j].Trim();
                    if (!int.TryParse(token, out int value))
                    {
                        throw new FormatException($"Invalid integer value at row {i + 1}, column {j + 1}: '{token}'");
                    }

                    adjacencyMatrix[i, j] = value;
                }
            }

            return adjacencyMatrix;
        }

        
    }
}
