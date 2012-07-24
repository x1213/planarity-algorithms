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
		curr = 0;
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
		curr = 0;
	}
}
	//設置head在此c-node中的embedding, 方向為1->0, 沒有flip.
void boundary_cycle::set_embedding(embedding* emb) 
{
	_emb = emb;
}

void boundary_cycle::contract_list_node() 
{
	//這個list是之後還有可能被traverse的list-nodes
	for (int i = 0; i < _possible_contract_list.size(); ++i) {
		_possible_contract_list[i]->set_RBC(0);//以防以後traverse時走到錯的RBC
		if (!_possible_contract_list[i]->get_node()->is_finished()) continue;
		//移動end-point
		if (_possible_contract_list[i] == _end_point[0]) _end_point[0] = _end_point[0]->get_next(list_node::NULL_0);
		if (_possible_contract_list[i] == _end_point[1]) _end_point[1] = _end_point[1]->get_next(list_node::NULL_1);
		_possible_contract_list[i]->contract();
	}
	//最後設好end-point的RBC
	if (_end_point[0] != 0) _end_point[0]->set_RBC(this);
	if (_end_point[1] != 0) _end_point[1]->set_RBC(this);
	//debug_rbc();
}

void boundary_cycle::recursively_flip(bool is_flip) 
{
	c_node()->mark();
	_is_flip = is_flip;
	for (int i = 0; i < _containment_list.size(); ++i) {
		_containment_list[i].first->recursively_flip((is_flip)? (!_containment_list[i].second) : _containment_list[i].second);
	}
}

//node-m是p-node ::待修改
//dir1是往1方向
void boundary_cycle::init_RBC(node* u, int dir1) 
{
	//已經不在任何c-node中
	if (u->get_list_node() == 0) {
		u->set_list_node(new list_node(u));
	}
	_end_point[0] = _end_point[1] = u->get_list_node();
	if (dir1 == 1) _end_point[0]->set_neighborhood(list_node::NULL_0, list_node::NULL_1);
	else _end_point[0]->set_neighborhood(list_node::NULL_1, list_node::NULL_0);
	_possible_contract_list.push_back(u->get_list_node());//加到可能收縮的list
	//debug_rbc();
}
	
//node-m是c-node
//end_dir0是n0接0的方向, 1同理.
void boundary_cycle::init_RBC(list_node* n0, list_node* n1, int end_dir0, int end_dir1) 
{
	_end_point[0] = n0;
	_end_point[1] = n1;
	n0->set_neighbor(end_dir0, list_node::NULL_0);
	n1->set_neighbor(end_dir1, list_node::NULL_1);
	_possible_contract_list.push_back(n0);//加到可能收縮的list
	if (n1 != n0) _possible_contract_list.push_back(n1);
	//debug_rbc();
}

//之前必不為empty
//方向要正確
void boundary_cycle::add_p_node(int dir, node* add) 
{
	if (add->get_list_node() != 0) {//debug
		int ded = 0;
	}
	add->set_list_node(new list_node(add));
	if (dir == 0) {
		if (_end_point[0]->n(0) == list_node::NULL_0) {
			_end_point[0]->set_neighbor(0, add->get_list_node());
		}
		else if (_end_point[0]->n(1) == list_node::NULL_0) {
			_end_point[0]->set_neighbor(1, add->get_list_node());
		} 
		else {//debug
			dir = 100;
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
		else {//debug
			dir = 100;
		}
		add->get_list_node()->set_neighborhood(_end_point[1], list_node::NULL_1);
		_end_point[1] = add->get_list_node();
	}
	_possible_contract_list.push_back(add->get_list_node());//加到可能收縮的list
	//debug_rbc();
}
	
//把b add進來...(segment 2的c-node)
//end node是b中要add的最終端的node
//dir是embed direction, 就是要add在哪個方向.
//start_dir是b中要從哪端開始
//end_dir是end_node哪端要接0
void boundary_cycle::add_list(boundary_cycle* b, list_node* end_node, int dir, int start_dir, int end_dir) 
{
	//加到可能收縮的list
	_possible_contract_list.push_back(b->_end_point[start_dir]);
	if (b->_end_point[start_dir] != end_node) _possible_contract_list.push_back(end_node);
	
       //set endnode的neighbor
	end_node->set_neighbor(end_dir, (dir==0)? list_node::NULL_0 : list_node::NULL_1);
		//set b的endpoint的neighbor
	if (b->_end_point[start_dir]->n(0) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(0, _end_point[dir]);
	else if (b->_end_point[start_dir]->n(1) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(1, _end_point[dir]);
	else {
		dir = 100;
	}
		//set _end_point[dir]的neighbor
	if (_end_point[dir]->n(0) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(0, b->_end_point[start_dir]);
	else if (_end_point[dir]->n(1) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(1, b->_end_point[start_dir]);
	else {
		dir = 100;
	}
		_end_point[dir] = end_node;//重新assign endpoint
	//debug_rbc();
}
	
//檢查此c-node是否可在segment-1 (在node_m以上的部分)
//即是只能恰有一個node的label > index
//而它必須要被visited
//最後傳回該node
//return 0 表示 false
node* boundary_cycle::segment_one_check(int index) 
{
	list_node* curr = _end_point[0];
	list_node* prev = list_node::NULL_0;
	list_node* temp = 0;
	node* return_node = 0;
	while (curr != list_node::NULL_1) {
		//不是i-tree的話就是terminal node
		//只能有一個
		if (!curr->get_node()->is_i_tree(index)) {
			if (return_node == 0) return_node = curr->get_node();
			else return 0;
		}
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	//terminal node一定要被visited
	if (return_node->is_visited(index)) return return_node;
	else return 0;
}
	//只能有一個i-tree區塊
//最後傳回該區塊最邊緣的node
//有可能會傳回一個label <= i的node, 這表示此node已經是terminal path的終點了.
//dir是i*-tree部分的方向(指哪個endpoint開始)
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

//檢查是否為node_m (其實根本不可能不是)
//總之傳回兩端search下去的終點
//有可能傳回的node是下接i-tree
pair<node*, node*> boundary_cycle::node_m_check(int index) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp = 0;
	while (curr0->get_node()->is_visited(index)) {
		//若遇到visited的非i-tree, 即是終點.
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
	//assume 這是node-m
//與之前一樣, 但是會完成init-RBC
pair<node*, node*> boundary_cycle::node_m_init_RBC(int index, boundary_cycle* p) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp0 = 0;
	list_node* temp1 = 0;
	while (curr0->get_node()->is_visited(index)) {
		//若遇到visited的非i-tree, 即是終點.
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
	//開始決定new RBC的範圍
	//note: 不可能只有single child
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
	/*
    //如果只有single node, 會有一種可能是原本方向是反的.
    //由於最後他一定會轉正, 所以要手動將embedding轉正
	if (n0 == n1) {
		if (e1 == 0) {//逆向
			 n0->get_node()->get_embedding()->set_flip(false);
			 n0->get_node()->get_embedding()->flip_end_point();
			 n0->set_RBC(this);
		}
	}
	*/
	//CHECK VALIDITY
	list_node* curr2 = n0;
	list_node* prev2 = n0->n(e0);
	list_node* temp2 = 0;
	while (curr2 != n1) {
		if (curr2 == 0) {
			curr2 = 0;
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
			curr2 = 0;
		}
		temp2 = curr2;
		curr2 = curr2->get_next(prev2);
		prev2 = temp2;
	}
	//end CHECK VALIDITY
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

void boundary_cycle::add_to_containment_list(boundary_cycle* b, bool f) 
{
	_containment_list.push_back(pair<boundary_cycle*, bool>(b, f));
}
	//回傳接i-tree的部分: 0或1
//若兩邊都可, 則傳回INT_MAX
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
	//不可能不會回傳值, 因為一定有一邊被visited.
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