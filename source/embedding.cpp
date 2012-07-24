//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "embedding.h"
#include "list_node.h"

#include <vector>

using namespace::std;

class list_node;

int switch_int(int);

embedding::embedding(node* u) 
{
	_RBC = 0;
	_n = u;
	_size = 0;
	_flip = false; //default == not flip
	_end_point[0] = _end_point[1] = 0;
}

embedding::~embedding() 
{
	vector<list_node*> delete_list;
	list_node* curr = _end_point[0];
	list_node* prev = 0;
	list_node* temp = 0;
	while (curr != 0) {
		delete_list.push_back(curr);
		temp = curr;
		curr = curr->get_next(prev);
		prev = curr;
	}
	for (int i = 0; i < delete_list.size(); ++i) delete delete_list[i];
	_end_point[0] = _end_point[1] = 0;
}

//clear embedding
void embedding::clear()
{
	_end_point[0] = _end_point[1] = 0;
}

void embedding::add(node* u, int dir, boundary_cycle* b) 
{
	_RBC = b;
	list_node* uu = new list_node(u);
	if ( _end_point[dir] != 0) {
		list_node::link(uu, _end_point[dir]);
	    _end_point[dir] = uu;
	}
	else {
		_end_point[0] = _end_point[1] = uu;
	}
	++_size;
}

//if not flip, link u's end[0] to the end[dir].
//Otherwise, add to end[switch_int(dir)].
//clear u.
void embedding::add(embedding* u, int dir, bool flip, boundary_cycle* b) 
{
	_RBC = b;
	if (_size == 0) {
		if (!flip) {
			_end_point[dir] = u->_end_point[1];
			_end_point[switch_int(dir)] = u->_end_point[0];
		}
		else {
			_end_point[dir] = u->_end_point[0];
			_end_point[switch_int(dir)] = u->_end_point[1];
		}
		return;
	}
	if (!flip) {
		list_node::link(u->_end_point[0], _end_point[dir]);
		_end_point[dir] = u->_end_point[1];
	}
	else {
		list_node::link(u->_end_point[1], _end_point[dir]);
		_end_point[dir] = u->_end_point[0];
	}
	_size += u->size();
	u->clear();
}

int embedding::size() 
{
	return _size;
}

void embedding::set_flip(bool f) 
{
	_flip = f;
}

bool embedding::is_flip() 
{
	return _flip;
}

list_node* embedding::end(int i) 
{
	return _end_point[i];
}

void embedding::flip_end_point() 
{
	list_node* temp = _end_point[0];
	_end_point[0] = _end_point[1];
	_end_point[1] = temp;
}

boundary_cycle* embedding::get_RBC() 
{
	return _RBC;
}

void embedding::set_RBC(boundary_cycle* b) 
{
	_RBC = b;
}
