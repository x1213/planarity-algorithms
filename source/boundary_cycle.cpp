//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "boundary_cycle.h"
#include <vector>
#include <utility>

extern int __markRef;
int switch_int(int);


boundary_cycle::boundary_cycle(node* c, node* i, node* t) 
{
	_c = c;
	_head = i;
	_node_t = t;
	_end_point[0] = _end_point[1] = 0;
    _emb = 0;
    _is_flip = false;
    _mark = 0;
}

boundary_cycle::~boundary_cycle() 
{
	if (_emb != 0) delete _emb;
}

//for debug
//check the integrity of the RBC.
//end[0] & end[1] should be able to travel to each other.
void boundary_cycle::debug_rbc() 
{
	if (_end_point[0] == _end_point[1] && _end_point[0] == 0) return;
	list_node* curr = _end_point[0];
	list_node* prev = list_node::NULL_0;
	list_node* temp = 0;
	while (curr != list_node::NULL_1) {
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	if (prev != _end_point[1]) {
		//There's a bug!
	}
	curr = _end_point[1];
	prev = list_node::NULL_1;
	temp = 0;
	while (curr != list_node::NULL_0) {
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	if (prev != _end_point[0]) {
		//There's a bug!
	}
}

//Set the partial embedding of head.
//The default of the embedding has the directoin from end[1] to end[0]
//No flip is required.
void boundary_cycle::set_embedding(embedding* emb) 
{
	_emb = emb;
}

//contract all the list-node that should be contract, namely, the nodes that has finished its embedding.
void boundary_cycle::contract_list_node() 
{
	//The nodes in this list may be traversed later.
	for (int i = 0; i < _possible_contract_list.size(); ++i) {
		_possible_contract_list[i]->set_RBC(0); //To prevent error in traverse to the wrong RBC in future BET.
		if (!_possible_contract_list[i]->get_node()->is_finished()) continue;
		//Move the end-point.
		if (_possible_contract_list[i] == _end_point[0]) _end_point[0] = _end_point[0]->get_next(list_node::NULL_0);
		if (_possible_contract_list[i] == _end_point[1]) _end_point[1] = _end_point[1]->get_next(list_node::NULL_1);
		_possible_contract_list[i]->contract();
	}
	//Set the end-point's RBC pointer
	if (_end_point[0] != 0) _end_point[0]->set_RBC(this);
	if (_end_point[1] != 0) _end_point[1]->set_RBC(this);
	//may need to call debug_rbc() if one wants to debug.
}

//recursively flip the sub-c-node
//the function is called only after the whole BET is done.
void boundary_cycle::recursively_flip(bool is_flip) 
{
	c_node()->mark();
	_is_flip = is_flip;
	for (int i = 0; i < _containment_list.size(); ++i) {
		_containment_list[i].first->recursively_flip((is_flip)? (!_containment_list[i].second) : _containment_list[i].second);
	}
}

//The case that node-m is p-node
//dir1 is the direction to end[1]
void boundary_cycle::init_RBC(node* u, int dir1) 
{
	//u is not in any c-node
	if (u->get_list_node() == 0) {
		u->set_list_node(new list_node(u));
	}
	_end_point[0] = _end_point[1] = u->get_list_node();
	if (dir1 == 1) _end_point[0]->set_neighborhood(list_node::NULL_0, list_node::NULL_1);
	else _end_point[0]->set_neighborhood(list_node::NULL_1, list_node::NULL_0);
	_possible_contract_list.push_back(u->get_list_node());// add to possible-contact-list
	//may need to call debug_rbc() if one wants to debug.
}
	
//The case that node-m is c-node
//end_dir0 is the direction of n0 toward end[0].
//Same definition for 1.
void boundary_cycle::init_RBC(list_node* n0, list_node* n1, int end_dir0, int end_dir1) 
{
	_end_point[0] = n0;
	_end_point[1] = n1;
	n0->set_neighbor(end_dir0, list_node::NULL_0);
	n1->set_neighbor(end_dir1, list_node::NULL_1);
	_possible_contract_list.push_back(n0);// add to possible-contact-list
	if (n1 != n0) _possible_contract_list.push_back(n1);
	//may need to call debug_rbc() if one wants to debug.
}

//This RBC must not be empty before adding p-node
//Be careful with the direction.
//This function add a p-node that has not been in any RBC before.
void boundary_cycle::add_p_node(int dir, node* add) 
{
	if (add->get_list_node() != 0) {
		//debug
		//The program should never reach here.
	}
	add->set_list_node(new list_node(add));
	if (dir == 0) {
		if (_end_point[0]->n(0) == list_node::NULL_0) {
			_end_point[0]->set_neighbor(0, add->get_list_node());
		}
		else if (_end_point[0]->n(1) == list_node::NULL_0) {
			_end_point[0]->set_neighbor(1, add->get_list_node());
		} 
		else {
		    //debug
		    //The program should never reach here.
		}
		add->get_list_node()->set_neighborhood(list_node::NULL_0, _end_point[0]);
		_end_point[0] = add->get_list_node();
	}
	else if (dir == 1) {
		if (_end_point[1]->n(0) == list_node::NULL_1) {
			_end_point[1]->set_neighbor(0, add->get_list_node());
		}
		else if (_end_point[1]->n(1) == list_node::NULL_1) {
			_end_point[1]->set_neighbor(1, add->get_list_node());
		} 
		else {
		    //debug
		    //The program should never reach here.
		}
		add->get_list_node()->set_neighborhood(_end_point[1], list_node::NULL_1);
		_end_point[1] = add->get_list_node();
	}
	_possible_contract_list.push_back(add->get_list_node());// add to possible-contact-list
	//may need to call debug_rbc() if one wants to debug.
}
	
//add b (a c-node in segment-2)
//end-node is the farest node that need to be added in b.
//dir is the embed direction, namely the direction that b be added.
//start_dir is the direction of the start-point of b (end[0] or end[1])
//end_dir is the direction that end_node toward the NULL, or future nodes to be added.
void boundary_cycle::add_list(boundary_cycle* b, list_node* end_node, int dir, int start_dir, int end_dir) 
{
	//add to possible-contact-list
	_possible_contract_list.push_back(b->_end_point[start_dir]);
	if (b->_end_point[start_dir] != end_node) _possible_contract_list.push_back(end_node);
	
    //set endnode's neighbor
	end_node->set_neighbor(end_dir, (dir==0)? list_node::NULL_0 : list_node::NULL_1);
	//set b's endpoint's neighbor
	if (b->_end_point[start_dir]->n(0) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(0, _end_point[dir]);
	else if (b->_end_point[start_dir]->n(1) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(1, _end_point[dir]);
	else {
		//debug
		//The program should never reach here.
	}
	//set _end_point[dir]'s neighbor
	if (_end_point[dir]->n(0) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(0, b->_end_point[start_dir]);
	else if (_end_point[dir]->n(1) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(1, b->_end_point[start_dir]);
	else {
		//debug
		//The program should never reach here.
	}
		_end_point[dir] = end_node;//reassign endpoint
	//may need to call debug_rbc() if one wants to debug.;
}
	
//Check if the c-node can be in segment-1
//return 0 indicate false
//if true, return the node that link downward in seg.1.
node* boundary_cycle::segment_one_check(int index) 
{
	list_node* curr = _end_point[0];
	list_node* prev = list_node::NULL_0;
	list_node* temp = 0;
	node* return_node = 0;
	while (curr != list_node::NULL_1) {
		//Not i-tree -> terminal node
		//We can have only one.
		if (!curr->get_node()->is_i_tree(index)) {
			if (return_node == 0) return_node = curr->get_node();
			else return 0;
		}
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	//terminal node must be visited
	if (return_node->is_visited(index)) return return_node;
	else return 0;
}

//Check if the c-node can be in segment-2
//we can have only one consecutive region of i-trees
//return the marginal node in the region.
//possibly return a node whose label <= i, it means we have reached the end of the terminal path.
node* boundary_cycle::segment_two_check(int index, bool &is_terminated) 
{
	is_terminated = false;
	list_node* curr = 0;
	list_node* prev = 0;
	list_node* temp = 0;
	node* return_node = 0;
	//0-方向被visited
	if (_end_point[0]->get_node()->is_visited(index)) {
		curr = _end_point[0];
		prev = list_node::NULL_0;
	}
	if (_end_point[1]->get_node()->is_visited(index)) {
		//雙邊都被visited, 唯一可能就是恰好其一端點為terminal path上的node, 其他都接i-tree
		//跑一次segment_one_check, 如果為true, 且回傳的node為end_point, 就為合法.
		if (curr != 0) {
			return_node = segment_one_check(index);
			if (return_node == _end_point[0]->get_node() || return_node == _end_point[1]->get_node()) return return_node;
			return 0;
		}
		//正常情形, 只有1-方向被visited
		curr = _end_point[1];
		prev = list_node::NULL_1;
	} 
	while (!curr->is_end()) {
		//走到curr沒被visited為止, 此時prev就是所求.
		if (!curr->get_node()->is_visited(index)) {
			//如果prev是i-tree, 就表示所有visited的subtree都是i-tree
			//回傳prev, 記下is_terminated
			if (prev->get_node()->is_i_tree(index)) is_terminated = true;
			return prev->get_node();
		}
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	return 0;
}

//It is the node_m. (Impossible to be false)
//return two nodes toward to direction of the terminal path. 
//(may be 0, or i-tree if we don't have.)
pair<node*, node*> boundary_cycle::node_m_check(int index) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp = 0;
	while (curr0->get_node()->is_visited(index)) {
		//visited non-i-tree, end point rached.
		if (!curr0->get_node()->is_i_tree(index)) {
			prev0 = curr0;
			break;
		}
		temp = curr0;
		curr0 = curr0->get_next(prev0);
		prev0 = temp;
	}
	while (curr1->get_node()->is_visited(index)) {
		if (!curr1->get_node()->is_i_tree(index)) {
			prev1 = curr1;
			break;
		}
		temp = curr1;
		curr1 = curr1->get_next(prev1);
		prev1 = temp;
	}
	return pair<node*, node*>((prev0->is_end())? (node*)0 : prev0->get_node(), (prev1->is_end())? (node*)0 : prev1->get_node());
}

//assume it is node-m
//same as the node_m_check() but we finish init-RBC with node-m
pair<node*, node*> boundary_cycle::node_m_init_RBC(int index, boundary_cycle* p) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp0 = 0;
	list_node* temp1 = 0;
	while (curr0->get_node()->is_visited(index)) {
		//visited non-i-tree, end point rached.
		if (!curr0->get_node()->is_i_tree(index)) {
			temp0 = prev0;
			prev0 = curr0;
			break;
		}
		temp0 = curr0;
		curr0 = curr0->get_next(prev0);
		prev0 = temp0;
	}
	while (curr1->get_node()->is_visited(index)) {
		if (!curr1->get_node()->is_i_tree(index)) {
			temp1 = prev1;
			prev1 = curr1;
			break;
		}
		temp1 = curr1;
		curr1 = curr1->get_next(prev1);
		prev1 = temp1;
	}
	//determine the region of new RBC
	//note: can't have only one child
	int e0, e1;
	list_node* n0, * n1;
	//0方向沒有被visited
	if (prev0 == list_node::NULL_0) {
		n0 = _end_point[0];
		e0 = (_end_point[0]->n(0) == list_node::NULL_0)? 0 : 1;
	}
	//prev0是i-tree, n0==curr0
	else if (prev0->get_node()->is_i_tree(index)) {
		n0 = curr0;
		e0 = (prev0 == curr0->n(0))? 0 : 1;
	}
	//prev0不是i-tree, 則prev0也會在被add進去
	else {
		n0 = prev0;
		//temp是在prev0之前的
		e0 = (temp0 == prev0->n(0))? 0 : 1;
	}
	//1方向沒有被visited
	if (prev1 == list_node::NULL_1) {
		n1 = _end_point[1];
		e1 = (_end_point[1]->n(0) == list_node::NULL_1)? 0 : 1;
	}
	//prev1是i-tree, n1==curr1
	else if (prev1->get_node()->is_i_tree(index)) {
		n1 = curr1;
		e1 = (prev1 == curr1->n(0))? 0 : 1;
	}
	//prev1不是i-tree, 則prev1也會在被add進去
	else {
		n1 = prev1;
		e1 = (temp1 == prev1->n(0))? 0 : 1;
	}
	/*CHECK VALIDITY
	list_node* curr2 = n0;
	list_node* prev2 = n0->n(e0);
	list_node* temp2 = 0;
	while (curr2 != n1) {
		if (curr2 == 0) {
			//.............
		}
		temp2 = curr2;
		curr2 = curr2->get_next(prev2);
		prev2 = temp2;
	}
	curr2 = n1;
	prev2 = n1->n(e1);
	temp2 = 0;
	while (curr2 != n0) {
		if (curr2 == 0) {
			c//....................
		}
		temp2 = curr2;
		curr2 = curr2->get_next(prev2);
		prev2 = temp2;
	}
    //end CHECK VALIDITY
	*/
	p->init_RBC(n0, n1, e0, e1);
	return pair<node*, node*>((prev0==list_node::NULL_0)? (node*)0 : prev0->get_node(), (prev1==list_node::NULL_1)? (node*)0 : prev1->get_node());
}

node* boundary_cycle::head() 
{
	return _head;
}

node* boundary_cycle::c_node() 
{
	return _c;
}

bool boundary_cycle::is_flip() 
{
	return _is_flip;
}

embedding* boundary_cycle::get_embedding() 
{
	return _emb;
}

list_node* boundary_cycle::end_point(int i) 
{
	return _end_point[i];
}

//add b to the containment-list
//f indicate whether is flip or not.
void boundary_cycle::add_to_containment_list(boundary_cycle* b, bool f) 
{
	_containment_list.push_back(pair<boundary_cycle*, bool>(b, f));
}

//return the direction to the i-tree-region: 0 or 1
//If both, return INT_MAX.
int boundary_cycle::i_tree_location(int index) 
{
	//0或1是否接i-tree
	bool is_0_i_tree = end_point(0)->get_node()->is_i_tree(index);
	bool is_1_i_tree = end_point(1)->get_node()->is_i_tree(index);
	if (is_0_i_tree && is_1_i_tree) {
		return INT_MAX;
	}
	else if (!is_0_i_tree && !is_1_i_tree) {
		//沒有i-tree部分, 因此current iteration中只該有一個child (除非是node_m)
		//現來看它在哪個部份
		if (end_point(0)->get_node()->is_visited(index)) is_0_i_tree = true;
		if (end_point(1)->get_node()->is_visited(index)) is_1_i_tree = true;
	}
    //兩邊都是i-tree, 代表此c-node在embed inside的部分.
	//或者是...只有一個node且被visited
	if (is_0_i_tree && is_1_i_tree) return INT_MAX; 
	if (is_0_i_tree) return 0;
	if (is_1_i_tree) return 1;
	//impossible to not return value, since one way must be visited.
}

node* boundary_cycle::find_unvisited_child(int index) 
{
	list_node* curr = end_point(0);
	list_node* prev = list_node::NULL_0;
	list_node* temp = 0;
	while (curr != list_node::NULL_1) {
		if (!curr->get_node()->is_visited(index)) return curr->get_node();
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	return 0;
}
void boundary_cycle::mark() 
{
	_mark = __markRef;
}

bool boundary_cycle::is_mark() 
{
	return _mark == __markRef;
}

node* boundary_cycle::get_node_t() 
{
	return _node_t;
}