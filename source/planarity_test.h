//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#ifndef PLANARITY_TEST
#define PLANARITY_TEST


#include <utility>
#include <vector>
#include <fstream>
#include <iostream>

#include "miscellaneous.h"
#include "list_node.h"
#include "embedding.h"
#include "node.h"
#include "boundary_cycle.h"
#include "simple_graph.h"

using namespace::std;

class node;
class list_node;
class simple_graph;

class planarity_test
{
public:
	planarity_test();
	~planarity_test();

	bool planarity_testing(simple_graph* in, simple_graph* emb, simple_graph* obs);

	//io
    void read_from_graph(simple_graph* g);
	void output_embedding(simple_graph* g);
	void output_obstruction(simple_graph* g);

	//pre-processing
	node* new_c_node(int i);
	void determine_biconnected_component();
	void postOrderTraversal(vector<node*> &node_list, vector<node*> &post_order_list);
	void initLabeling(vector<node*> &post_order_list);
	void sortAdjList(vector<node*> &post_order_list);

	//post-processing
	void c_node_flip();
	void recover_embedding();
	void trivial_embed(node* node_i, node* node_t);

	//BET
	bool back_edge_traversal();
    bool back_edge_traversal(node* node_i, node* node_t, vector<node*> &traverse_list);
	void simple_upwardTraversal(node* s_node, int index, vector<node*> &c_node_list);
    bool check_upwardTraversal(node* s_node, int index);
    boundary_cycle* simple_c_node_traverse(node* node_s);
	boundary_cycle* parallel_search(node* node_s, int index);
	bool build_c_node(node* node_t, node* node_i);
	node* check_RBC_validity(node* node_t, int index);

	//for bebug
	void brute_force_check_emb();
	int emb_get_next(int curr, int prev);
	void output_partial_embedding();

	//extend terminal path
	void downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int come_from);
	void downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction);
	void embed_p_node(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int add_node_direction);
    void downward_embed_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent);
	void downward_embed_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent, int embed_direction, int come_from);
    void embed_p_node_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent, int embed_direction, int add_node_direction);
	void embed_node_m(node* m, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent);

	//obstruction
	void seg_2_extract_obstruction(node* current, node* node_m, int index);
	void three_terminal_extract_obstruction(node* m, int index);

	void p_K_3_3_search_1(node* m, node* u_0, node* u_1, int index);
	void p_K_3_3_search_2(node* m, node* u_0, node* u_1, node* u_2, int index);
    void c_K_3_3_search_1(node* v, node* u_0, node* u_1, int index);
	void c_K_3_3_search_2(node* m, node* v, node* u_0, node* u_1, int index);
	void c_K_5_search(node* m, node* u_0, node* u_1, int index);

	boundary_cycle* c_node_search(node* u);
	node* find_LCA(node* u, node* v);
	node* add_back_edge_from_descendant(node* u, vector<node*> &list, int post_order_index);
	node* add_higher_back_edge_from_descendant(node* u, vector<node*> &list, int post_order_index, node* forbid= 0);
	node* c_node_square_linking(node* v, node* u_0, node* u_1);
	node* c_node_triangle_linking(node* u_0, node* u_1);
	void add_path_to_obstruction(node* p, node* c);
	void add_to_obstruction(int i, int j);
	void radix_sort_obstruction_list();

private:
	vector<int> _node_list;
	vector<pair<int, int> > _obstruction;
	vector<vector<int> > _adj_list;
	vector<vector<int> > _embed_list;
	vector<node*> _dfs_list;
	vector<node*> _c_node_list;
};


#endif
