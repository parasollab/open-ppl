GAP
======

This is an implementation of the [GAP Benchmark
Suite](http://gap.cs.berkeley.edu/benchmark.html). It includes six graph processing kernels:

- Breadth-first search
- Single-source shortest path
- PageRank
- Weakly connected components
- Betweenness centrality
- Triangle count

In addition to the kernels, it provides mechanisms to generate or read graph inputs in the following formats:

- er: Uniform (Erdos-Renyi)
- kron: Kronecker (Graph 500)
- mtx: Matrix market format
- el: Edge list format
- sadj: Sharded adjacency list format
- nsel: Nested-sharded edge list format

References
====

The GAP Benchmark Suite, Scott Beamer, Krste Asanović, and David Patterson,
arXiv:1508.03619 [cs.DC], 2015. arXiv
