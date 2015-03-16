# An example of MPS algorithm via PC-tree #

Here we present an example of our MPS algorithm via PC-tree. The double cycle denotes the C-node; the arrow indicates parent/children relation; the solid line is boundary link; the dotted line is back edge. We use i’ denoted the replica node of i. If i contains multiple replica nodes, then the new replica nodes will be i’’, i’’’, …

Input graph = K\_5

Nodes: {0, 1, 2, 3, 4}

DFS-tree: 0→1→2→3→4

Back edges: {(2, 0), (3, 0), (3, 1), (4, 0), (4, 1), (4, 2)}



### Add (2, 0) ###

![https://lh5.googleusercontent.com/-BGP-oV8uj_0/UCsv7utrv1I/AAAAAAAAAGA/TwrOtmR-y_I/h120/1.png](https://lh5.googleusercontent.com/-BGP-oV8uj_0/UCsv7utrv1I/AAAAAAAAAGA/TwrOtmR-y_I/h120/1.png)

### Add (3, 0) ###
Construct(0) yields a C-node of a 2-component, which consists of {0’, 2’} on the BC and an artificial link τ(0’, 2’) composed of only 1.

![https://lh3.googleusercontent.com/-zh1pY4D1U58/UCsv7prSPkI/AAAAAAAAAGE/2tc6PmSw4TA/s487/2.png](https://lh3.googleusercontent.com/-zh1pY4D1U58/UCsv7prSPkI/AAAAAAAAAGE/2tc6PmSw4TA/s487/2.png)

### Add (3, 1) ###
Find(1) procedure call the Trim(1) procedure, since 1 is in an artificial link.

After that, we make 1 a child of the C-node, and 1’ on the BC.

![https://lh4.googleusercontent.com/-DpYgGaVxrOo/UCsv7iqyGjI/AAAAAAAAAGI/Kapf7Qq2aYY/w432-h250-n-k/3.png](https://lh4.googleusercontent.com/-DpYgGaVxrOo/UCsv7iqyGjI/AAAAAAAAAGI/Kapf7Qq2aYY/w432-h250-n-k/3.png)

### Add (4, 0) ###
Construct(0) yields a C-node of a 3-component, which consists of {0’’, 3’} on the BC and an artificial link τ(0’’, 3’) composed of previous C-node with two children {1, 2}.

![https://lh6.googleusercontent.com/-DY2rW5WUrJk/UCsv7-VbfsI/AAAAAAAAAGQ/SCqMohaZ1CE/s509/4.png](https://lh6.googleusercontent.com/-DY2rW5WUrJk/UCsv7-VbfsI/AAAAAAAAAGQ/SCqMohaZ1CE/s509/4.png)

### Add (4, 1) ###
Find(1) visit the C-node hierarchy.

Trim(1) is called, and then 2 is eliminated, so the back edge (4, 2) is deleted.

The C-node in the lower hierarchy is destroyed; the remaining nodes are merged to the BC of the top-tier C-node.

![https://lh5.googleusercontent.com/-SmZiMEP23ec/UCsv7wCWeJI/AAAAAAAAAGU/IBo48F5Qa9c/w510-h307-n-k/5.png](https://lh5.googleusercontent.com/-SmZiMEP23ec/UCsv7wCWeJI/AAAAAAAAAGU/IBo48F5Qa9c/w510-h307-n-k/5.png)

We are done.

The MPS of K\_5 is K\_5 minus an edge, namely (4, 2) in our example.