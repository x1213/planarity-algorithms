//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of mps.cpp, mps_io.cpp.
//-----------------------------------------------------------------------------------

#ifndef _MPS_H
#define _MPS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <climits>
#include "mps_node.h"
#include "miscellaneous.h"
#include "simple_graph.h"

using namespace std;

class simple_graph;
class mps_node;
class maximal_planar_subgraph_finder;

//finding mps
class maximal_planar_subgraph_finder
{
public:
	maximal_planar_subgraph_finder();
	~maximal_planar_subgraph_finder();
	bool planarity_test(simple_graph* in);
	void find_mps(simple_graph* in, simple_graph* out);
	void read_from_file(ifstream* in);
	void read_from_graph(simple_graph* g);
	void output_to_graph(simple_graph* g);


	mps_node* get_new_node(node_type t);
	void postOrderTraversal();
	void sort_adj_list();
	void determine_edges();
	void back_edge_traversal();
	bool back_edge_traversal_without_deletion();
	bool back_edge_traversal(mps_node* traverse_node, int index);
	void make_essential(mps_node* p_node, mps_node* c_node);
	mps_node* find(mps_node* n);
	void merge(pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > boundary, mps_node* list_node);
	void eliminate(mps_node* u);
	void eliminate_AE(mps_node* u, mps_node* v);
	pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > trim(mps_node* u);
	void c_node_extension(mps_node* c_node);
	void recursively_shaving(mps_node* u);
	pair<mps_node*, mps_node*> shave(mps_node* x);
	pair<mps_node*, mps_node*> parallel_search_sentinel(mps_node* x, mps_node* &c);
	pair<mps_node*, mps_node*> parallel_search_sentinel(mps_node* n0, mps_node* n0_prev, mps_node* n1, mps_node* n1_prev, mps_node* & c);
	pair<mps_node*, mps_node*> count_sentinel_elimination(pair<mps_node*, mps_node*> sentinel_1, int num_sentinel);
	mps_node* construct(mps_node* u);
	mps_node* construct(mps_node* c, mps_node* p);
	void parenting_labeling_shaving(mps_node* u, mps_node* node_i) ;

private:
	vector<mps_node*> _node_list; //List of nodes input.
	vector<pair<mps_node*, mps_node*> > _edge_list; // Edges in DFS-tree. These edges must be contained in the maximal planar subgraph that we found.
	vector<mps_node*> _post_order_list; //The sorted version (increasing with post-order-index) of _node_list.
	vector<pair<mps_node*, mps_node*> > _back_edge_list; // Edges other than that in DFS-tree. (The first node's index is higher than the second's.)
	vector<bool> _is_back_edge_eliminate; //Record that if the back-edge has been eliminated or not.
	vector<mps_node*> _new_node_list; //Newly added nodes.
};

#endif
