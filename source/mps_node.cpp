//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <climits>

#include "mps_node.h"

//-----------------------------------------------------------------------------------
// CONSTRUCTOR
//-----------------------------------------------------------------------------------
mps_node::mps_node(node_type t) {
	_type = t;
	_label = pair<int, label>(INT_MAX, NOT_VISITED);
	_neighbor[0] = _neighbor[1] = 0;
	_AE_root[0] = _AE_root[1]  = 0;
	_original_node = 0; 
    _c_node = 0;
	_parent = 0;
	_post_order_index = INT_MAX;
	_node_id = INT_MAX;
	_mark = 0;
}

//-----------------------------------------------------------------------------------
// TYPE, ID, INDEX
//-----------------------------------------------------------------------------------
node_type mps_node::type() {return _type;}

int mps_node::post_order_index() {return _post_order_index;}

void mps_node::set_id(int i) {_node_id = i;}

void mps_node::set_post_order_index(int i) {_post_order_index = i;}

//Only used when consturcting c-node
//The first node calling this function would not be labeled.
void mps_node::recursively_labeling() {		
	for (int i = 0; i < _children.size(); ++i) {
		_children[i]->_label.second = ARTIFICIAL_EDGE;
		_children[i]->recursively_labeling();
	} 
}

int mps_node::node_id() {return _node_id;}

//-----------------------------------------------------------------------------------
// DFS-TREE
//-----------------------------------------------------------------------------------
void mps_node::add_adj(mps_node* n) {_adj_list.push_back(n);}

int mps_node::degree() {return _adj_list.size();}

mps_node* mps_node::adj(int i) {return _adj_list[i];}

void mps_node::set_adj_list(vector<mps_node*> vec) {_adj_list = vec;}

void mps_node::DFS_visit(vector<mps_node*> &dfsList, int &index) {
	mark();
	for (int i = 0; i < _adj_list.size(); ++i) {
		if (!_adj_list[i]->is_marked()) {
			_adj_list[i]->_parent = this;
			_adj_list[i]->DFS_visit(dfsList, index);
		}
	}
	set_post_order_index(index);
	dfsList.push_back(this);
	++index;
}

//-----------------------------------------------------------------------------------
// PARENT-CHILDREN
//-----------------------------------------------------------------------------------
int mps_node::child_num() {return _children.size();}

mps_node* mps_node::child(int i) {return _children[i];}

mps_node* mps_node::parent() {return _parent;}

void mps_node::clear_children() {
	_children.clear();
}

void mps_node::remove_child(int i) {
	_children[i] = _children[_children.size()-1];
	_children.resize(_children.size()-1);
}

void mps_node::remove_child(mps_node* n) {
	for (int i = 0; i < _children.size(); ++i) {
	    if (_children[i] == n) {
			_children[i] = _children[_children.size()-1];
	        _children.resize(_children.size()-1);
		}
	}
}

void mps_node::add_child(mps_node* n) {
	_children.push_back(n);
}

vector<mps_node*>* mps_node::get_children_list() {
	vector<mps_node*>* ptr = new vector<mps_node*>(_children);
	return ptr;
}

void mps_node::set_parent(mps_node* n) {
	_parent= n;
}

//-----------------------------------------------------------------------------------
// BOUNDARY_PATH
//-----------------------------------------------------------------------------------
void mps_node::set_to_boundary_path(mps_node* n0, mps_node* n1) {
	_parent = 0;
	_children.clear();
	_neighbor[0] = n0;
	_neighbor[1] = n1;
	set_2nd_label(BOUNDARY_PATH);
}

mps_node* mps_node::get_next(mps_node* prev) {
	if (_neighbor[0] != prev) return _neighbor[0];
	else return _neighbor[1];
}

mps_node* mps_node::neighbor(int i) {return _neighbor[i];}

void mps_node::set_neighbor(int i, mps_node* n) {_neighbor[i] = n;}

void mps_node::set_neighbor(mps_node* u, mps_node* v) {
	_neighbor[0] = u;
	_neighbor[1] = v;
}

//-----------------------------------------------------------------------------------
// ARTIFICIAL EDGE
//-----------------------------------------------------------------------------------
mps_node* mps_node::AE(int i) {return _AE_root[i];}

void mps_node::set_AE(int i, mps_node* j) {
	_AE_root[i] = j;
	if (j != 0) j->set_parent(this);
} 

void mps_node::add_AE(mps_node* j) {
	if (j == 0) return;
	if (_AE_root[0] == 0) set_AE(0, j);
	else if (_AE_root[1] == 0) set_AE(1, j);
}

//Inherit u's artificial edge.
void mps_node::inherit_AE(mps_node* u) {
	if (u->_AE_root[0] != 0) add_AE(u->_AE_root[0]);
	if (u->_AE_root[1] != 0) add_AE(u->_AE_root[1]);
	u->_AE_root[0] = u->_AE_root[1] = 0;
}

//Set itself to be an AE-root-node in u.
//Inherite u's chilren-list.
//Do nothing if u does not have any children.
void mps_node::init_AE(mps_node* u) {
	if (u->child_num() == 0) return;
	_children = u->_children;
	u->clear_children();
	for (int i = 0; i < _children.size(); ++i) {
		_children[i]->set_parent(this);
	}
	set_parent(u);
	set_1st_label(_children[0]->get_1st_label());
	set_2nd_label(ARTIFICIAL_EDGE);
	u->add_AE(this);
}

//-----------------------------------------------------------------------------------
// REPLICA
//-----------------------------------------------------------------------------------
mps_node* mps_node::original_node() {return _original_node;}

mps_node* mps_node::get_c_node() {return _c_node;}

void mps_node::set_c_node(mps_node* c) {_c_node = c;}

bool mps_node::is_sentinel() {return type() == REPLICA_NODE;} 

//Check if n1 and n2 correspond to the same node
bool mps_node::is_same(mps_node* n1, mps_node* n2) {
	mps_node* s1 = (n1->type() == REPLICA_NODE)? n1->original_node() : n1;
	mps_node* s2 = (n2->type() == REPLICA_NODE)? n2->original_node() : n2;
	return s1 == s2;
}

//Set itself to be a replica-node of u in c.
//Only inherit some basic setting, not including info about neighborhood.
void mps_node::init_replica(mps_node* u, mps_node* c) {
	set_post_order_index(u->post_order_index());
	set_2nd_label(BOUNDARY_PATH);
	_original_node = (u->type() == REPLICA_NODE)? u->original_node() : u;
	_c_node = c;
}

//-----------------------------------------------------------------------------------
// LABELING
//-----------------------------------------------------------------------------------
void mps_node::set_1st_label(int i) {_label.first = i;}

void mps_node::set_2nd_label(label i) {_label.second = i;}

int mps_node::get_1st_label() {return _label.first;}

label mps_node::get_2nd_label() {return _label.second;}

//-----------------------------------------------------------------------------------
// C-NODE
//-----------------------------------------------------------------------------------
mps_node* mps_node::get_a_list_node() {
	return _essential_list[0];
}

int mps_node::c_node_size() {
	return _essential_list.size();
}

mps_node* mps_node::essential(int i) {
	return _essential_list[i];
}

void mps_node::clear_essential() {_essential_list.clear();}

void mps_node::add_essential(mps_node* u) {_essential_list.push_back(u);}

//-----------------------------------------------------------------------------------
// MARK
//-----------------------------------------------------------------------------------
void mps_node::mark() {_mark = _ref_mark;}

void mps_node::init_mark() {++_ref_mark;}

void mps_node::un_mark() {_mark = 0;}

bool mps_node::is_marked() {return _mark == _ref_mark;}

int mps_node::_ref_mark = 1;
