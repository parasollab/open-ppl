
This benchmark creates an R-MAT graph to simulate small-world, scale-free
networks like the internet, social networks, etc. It then runs breadth-first
searches from various vertices on the graphs to simulate common operations on
said networks.

For more information, see www.graph500.org


Building: (assuming you are in the benchmark/graph/g500/ folder)
1. Compile the generator: cd $STAPL/benchmark/graph/g500/generator/; gmake;
2. Compile the benchmark: cd $STAPL/benchmark/graph/g500/ ; gmake;
   The benchmark will link with the graph500 generator that was built in step#1.


Running:
./g500_bench SCALE Edge-Factor [--miniterations k]

SCALE: This is log2(N), where N is the number of vertices you want the input
       graph to have. Or, put another way, the input graph will have 2^SCALE
       vertices.

Edge-Factor: This specifies the average number of edges per vertex. Use a value
             of 16 for most cases, unless you want to create different
             topologies of graphs. Note: This does not guarantee that all
             vertices will have 16 edges. In fact, the vertices will follow a
             power-law out-degree (PLOD) distribution in their edges.
             The benchmark specification uses a value of 16.

--miniterations k: This will run k different BFS traversals from k randomly
                   chosen vertices. The graph is built once.


A few tips to choose SCALE:
1. For good performance, choose SCALE to be near the max value that would fill
   the local memory while still leaving enough space for communication buffers.
   Typically, this means a SCALE of 19-20 per 1.5-2GB of RAM.
2. Weak-scaling -- increment SCALE by +1 for each doubling of cores.




Besides the official implementation of the benchmark in g500_bench.cc, drivers
are also provided for the following algorithms:

bc_bench.cc  -- runs the Betweenness Centrality algorithm on the G500 graph.
cc_bench.cc  -- runs the Connected Components algorithm on the G500 graph.
cd_bench.cc  -- runs the Community Detection algorithm on the G500 graph.
kc_bench.cc  -- runs the k-core Decomposition algorithm on the G500 graph.
pd_bench.cc  -- runs the Pseudo Diameter algorithm on the G500 graph.
pr_bench.cc  -- runs the PageRank algorithm on the G500 graph.
