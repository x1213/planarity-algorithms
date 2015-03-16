A planar graph is a graph that can be drawn on the plane without any crossing. A planarity test is a decision algorithm that determine whether a graph is planar. A certificate to a graph being planar is its planar embedding. On the other hand, a graph is non-planar if and only if it contains a Kuratowski subgraph.

There are many algorithm which can do the planarity testing in linear time, some of them are based on a data structure named PQ-tree. Shih and Hsu developed a linear time algorithm that based on PC-tree, which is a generalization of PQ-tree. Their algorithm can be modified to be a MPS algorithm that can output a maximal planar subgraph of the input graph.

The project is an implementation of planarity testing algorithm via PC-tree. It can output a combinatorial embedding if the input graph is planar, otherwise, a Kuratowski subgraph is isolated, Furthermore, it can find a maximal planar subgraph of the input graph. All of them can be done in linear time.

Note: You may need the Microsoft Visual C++ 2010 Redistributable Package in order to execute this program.