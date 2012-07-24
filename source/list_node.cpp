//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "list_node.h"

list_node::list_node(node* n) 
{
	_n = n;
	_RBC = 0;
	_neighbor[0] = _neighbor[1] = 0;
}

list_node::~list_node() {}

list_node* list_node::n(int i) 
{
	return _neighbor[i];
}

list_node* list_node::get_next(list_node* n) 
{
	if (_neighbor[1] == n) return _neighbor[0];
	else if (_neighbor[0] == n) return _neighbor[1];
	else return 0;
}

//收縮list-node
void list_node::contract() 
{
	if (_neighbor[0] != 0 || !_neighbor[0]->is_end()) {
	    if (_neighbor[0]->n(0) == this) _neighbor[0]->_neighbor[0] = _neighbor[1];
		else _neighbor[0]->_neighbor[1] = _neighbor[1];
	}
	if (_neighbor[1] != 0 || !_neighbor[1]->is_end()) {
	    if (_neighbor[1]->n(0) == this) _neighbor[1]->_neighbor[0] = _neighbor[0];
		else _neighbor[1]->_neighbor[1] = _neighbor[0];
	}
}


node* list_node::get_node() 
{
	return _n;
}

boundary_cycle* list_node::get_RBC() 
{
	return _RBC;
}

void list_node::set_RBC(boundary_cycle* c) 
{
	_RBC = c;
}

void list_node::set_neighborhood(list_node* n0, list_node* n1) 
{
	_neighbor[0] = n0;
	_neighbor[1] = n1;
}

void list_node::set_neighbor(int i, list_node* u) 
{
	_neighbor[i] = u;
}

bool list_node::is_end() 
{
	return (this == list_node::NULL_0) || (this == list_node::NULL_1);
}

//link n1-n2, 兩個node都要有空位.
bool list_node::link(list_node* n1, list_node* n2) 
{
	//link n1->n2
	if (n1->_neighbor[0] == 0 || n1->_neighbor[0]->is_end()) n1->_neighbor[0] = n2;
	else if (n1->_neighbor[1] == 0 || n1->_neighbor[1]->is_end()) n1->_neighbor[1] = n2;
	else return false;
	//link n2->n1
	if (n2->_neighbor[0] == 0 || n2->_neighbor[0]->is_end()) n2->_neighbor[0] = n1;
	else if (n2->_neighbor[1] == 0 || n2->_neighbor[1]->is_end()) n2->_neighbor[1] = n1;
	else return false;
	return true;
}

list_node* list_node::NULL_0 = new list_node(0);

list_node* list_node::NULL_1 = new list_node(0);