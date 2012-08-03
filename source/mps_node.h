//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of mps_node.cpp.
//-----------------------------------------------------------------------------------

#ifndef _MPS_NODE_H
#define _MPS_NODE_H

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <climits>

#include "miscellaneous.h"

using namespace::std;

// node in mps algorithm
class mps_node
{
public:
	//CONSTRUCTOR
	mps_node(node_type t);

	//DESTRUCTOR
	~mps_node() {}

	//TYPE, ID, INDEX
    node_type type();
	int post_order_index();
	void set_id(int i);
	void set_post_order_index(int i);
	void recursively_labeling();
	int node_id();

	//DFS-TREE
	void add_adj(mps_node* n);
	int degree();
	mps_node* adj(int i);
	void set_adj_list(vector<mps_node*> vec);
	void DFS_visit(vector<mps_node*> &dfsList, int &index);

	//PARENT-CHILDREN
	void set_parent(mps_node* n) ;
	mps_node* parent();
	int child_num();
	mps_node* child(int i);
	void add_child(mps_node* n);
	void clear_children();
	void remove_child(int i);
	void remove_child(mps_node* n);
	vector<mps_node*>* get_children_list();

	//BOUNDARY_PATH
	void set_to_boundary_path(mps_node* n0, mps_node* n1);
	void set_neighbor(int i, mps_node* n);
	void set_neighbor(mps_node* u, mps_node* v);
	mps_node* neighbor(int i);
	mps_node* get_next(mps_node* prev);

	//ARTIFICIAL EDGE
	mps_node* AE(int i);
	void set_AE(int i, mps_node* j);
	void add_AE(mps_node* j);
	void inherit_AE(mps_node* u);
	void init_AE(mps_node* u);

	//REPLICA
	mps_node* original_node();
	mps_node* get_c_node();
	void set_c_node(mps_node* c);
	bool is_sentinel();
	static bool is_same(mps_node* n1, mps_node* n2);
	void init_replica(mps_node* u, mps_node* c);

	//LABELING
	void set_1st_label(int i);
	void set_2nd_label(label i);
	int get_1st_label();
	label get_2nd_label();

	//C-NODE
	mps_node* get_a_list_node();
	int c_node_size();
	mps_node* essential(int i);
	void clear_essential();
	void add_essential(mps_node* u);

	//MARK
	void mark();
	static void init_mark();
	void un_mark();
	bool is_marked();

private:
	//Basic information.
	node_type _type;
	pair<int, label> _label;

	//Information about neighborhood.
	mps_node* _neighbor[2];
	mps_node* _AE_root[2];

	//Information about higher hierarchy.
	mps_node* _original_node; 
    mps_node* _c_node;

	//Information about parent-children relation.
	mps_node* _parent;
	vector<mps_node*> _children;

	//Information about about p-nodes in DFS-tree
	vector<mps_node*> _adj_list;
	int _post_order_index;
	int _node_id;

	//List of essential nodes in c-node
	vector<mps_node*> _essential_list;

	//Mark
	int _mark;
	static int _ref_mark;
};

#endif
