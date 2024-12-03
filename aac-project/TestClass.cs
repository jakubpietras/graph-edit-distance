// #define EXACT
 #define ASTAR
// #define BEAM
 #define JV
using aac_project;
using LinearAssignment;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;


namespace Task_2
{
    public class TestClass
    {
        static void Main(string[] args)
        {
            string baseDirectory = AppDomain.CurrentDomain.BaseDirectory;
            string projectRoot = Path.Combine(baseDirectory, "..", "..", "..");
            // read input matrices (TODO: change reading function to the one from MA)
            string G1_filename = "graph1.txt";
            string G2_filename = "graph2.txt";
            int[,] G1 = GraphEditDistanceExact.ReadAdjacencyMatrix(Path.Combine(Path.Combine(projectRoot, G1_filename)));
            int[,] G2 = GraphEditDistanceExact.ReadAdjacencyMatrix(Path.Combine(Path.Combine(projectRoot, G2_filename)));
            bool isDirected = true;
            int[,] costMatrix = GraphEditDistanceApproximation.CreateCostMatrix(G1, G2, isDirected);

            //var editPath = new EditPath(costMatrix, new List<(int, int)>() { (1, 1), (3, -1), (2, 2) }, G1.GetLength(0), G2.GetLength(0));
            //GraphEditDistanceApproximation.PrintMatrix(editPath.PartialCostMatrix());

            //var res = Solver.Solve(editPath.PartialCostMatrix()).RowAssignment;

            //Console.WriteLine(editPath.EstimateEditCost());



#if EXACT
            // ====== GED - Brute Force ======
            Console.WriteLine("====== GED - Brute Force ======");
            int ged = GraphEditDistanceExact.BruteForceGED(G1, G2, isDirected);
            Console.WriteLine($"Exact Graph Edit Distance is: {ged}\n");
#endif

#if ASTAR
            // ====== GED - LSAP solved by Munkres Algorithm with Beam Optimization ======
            Console.WriteLine("====== GED - A* Approximation ======");
            int result = GEDApproximation.AStar(G1, G2, isDirected);
            Console.WriteLine($"A* Approximated Graph Edit Distance: {result}");

#endif
#if JV
            // Jonker-Volgenant 
            Console.WriteLine("====== GED - Jonker-Volgenant LSAP Approximation ======");
            var res = Solver.Solve(costMatrix).ColumnAssignment;
            //foreach (var el in res)
            //{
            //    Console.WriteLine(el);
            //}
            Console.WriteLine($"J-V: {GraphEditDistanceApproximation.CalculateCost(costMatrix, res)}");
#endif
#if BEAM
            // ====== GED - LSAP solved by Munkres Algorithm with Beam Optimization ======
            Console.WriteLine("====== GED - LSAP solved by Munkres Algorithm with Beam Optimization ======");
            int beamParameter = 10;
            var lsapResult = GraphEditDistanceApproximation.Beam(G1, G2, isDirected, beamParameter);
            Console.WriteLine($"LSAP with Beam Approximated Graph Edit Distance is: {lsapResult}\n");
#endif
        }
    }
}
