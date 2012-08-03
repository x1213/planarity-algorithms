//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "node.h"
#include <vector>
#include <utility>
#include <iostream>
#include <climits>

extern int __markRef;


//-----------------------------------------------------------------------------------
// Constructor, descructor.
//-----------------------------------------------------------------------------------

node::node(int i, node_type t) 
{
	_type = t;
	_Mark = 0;
	_index = i;
	_parent = 0;
    _post_order_index = INT_MAX;
    _maxBackAnscenstorID = INT_MAX;
	_visited = INT_MAX;
	_list_node = 0;
	_RBC = 0;
	_emb = 0;
	_is_done = false;
	_i_tree_test = pair<int, bool> (INT_MAX, false);
}

node::~node()
{
	if (_RBC != 0) delete _RBC;
	if (_list_node != 0) delete _list_node;
	if (_emb != 0) delete _emb;
}


//-----------------------------------------------------------------------------------
// Misc...
//-----------------------------------------------------------------------------------

void node::init_c_node(node* head, node* node_t, boundary_cycle* b) 
{
	_parent = head;
	_maxBackAnscenstorID = node_t->_maxBackAnscenstorID;
	_RBC = b;
}

node_type node::type() {return _type;}
int node::index() {return _index;}

bool node::is_finished() 
{//是否已經被embed完了. 是的話就可以被contract
	if (type() == P_NODE) return _is_done = (_emb != 0 && _emb->size() == degree());
	else return _is_done;
}

void node::finish() {_is_done = true;}

//自己連到post-order-index最高的node的post-order-index
int node::maxBackEdgeID() {return _adjList[_adjList.size() - 1]->_post_order_index;}
//類似上面, 但是範圍包括了自己往下的整個sub-tree所連到的node
int node::maxBackAnscenstorID() {return _maxBackAnscenstorID;}



void node::add_adj(node* u) {_adjList.push_back(u);}
void node::set_parent(node* u) {_parent = u;}
void node::setAdjList(vector<node*> &adjList) 
{
	_adjList = adjList;
}
int node::postOrderIndex() {
	return _post_order_index;
}
int node::degree() {
	return _adjList.size();
}
node* node::adj(int i) {
	return _adjList[i];
}
bool node::is_root() {return _parent == 0;}
void node::mark() {
	_Mark = __markRef;
}
bool node::isMarked() {
	return _Mark == __markRef;
}

node* node::parent() {return _parent;}
bool node::is_visited(int i) {return _visited == i;}
void node::visit(int i) {_visited = i;}



list_node* node::get_list_node() {return _list_node;}
void node::set_list_node(list_node* n) {_list_node = n;}
boundary_cycle* node::get_RBC() {return _RBC;}


embedding* node::get_embedding() 
{ 
	if (_emb == 0) _emb = new embedding(this);
	return _emb;
}


//_list_node != 0 <-> 它在某c-node的RBC中
bool node::is_in_c_node() {return _list_node != 0;}

//-----------------------------------------------------------------------------------
// 2-connected comp, dfs, pre-processing...
//-----------------------------------------------------------------------------------

void node::DFS_visit(vector<node*> &dfsList, int &index) 
{
	mark();
	for (int i = 0; i < _adjList.size(); ++i) {
		if (!_adjList[i]->isMarked()) {
			_adjList[i]->_parent = this;
			_descendantList.push_back(_adjList[i]);
			_adjList[i]->DFS_visit(dfsList, index);
		}
	}
	_post_order_index = index;
	dfsList.push_back(this);
	++index;
}

void node::setMaxBackAnscenstorID() 
{
	if (_adjList.empty()) return;
	_maxBackAnscenstorID = _adjList[_adjList.size() - 1]->_post_order_index;
	for (int i = 0; i < _descendantList.size(); ++i) {
		if (_descendantList[i]->_maxBackAnscenstorID > _maxBackAnscenstorID) 
			_maxBackAnscenstorID = _descendantList[i]->_maxBackAnscenstorID;
	}
}


//找到節點, 並將之分離.
//設好各個分身在其連通部份對應的adj-list
void node::split_arculation_point(vector<node*> &node_list, node* modified) 
{
	mark();
	bool is_slab = true;
	int iterate_index = 0;
	node* new_node = 0;
	//此區塊的root已被修改.
	if (modified != 0 && _adjList[_adjList.size()-1]->_index == modified->_index) _adjList[_adjList.size()-1] = modified;
	//如果不是節點
	if (!is_articulation_point()) {
		for (int i = 0; i < _descendantList.size(); ++i) {
			_descendantList[i]->split_arculation_point(node_list, modified);
		}
		return;
	}
	//是節點
	for (int i = 0; i < _descendantList.size(); ++i) {
		is_slab = false; //這個boolean表示"this node + 此subtree"是否為一區塊
		if (_descendantList[i]->_maxBackAnscenstorID == _post_order_index) {
			//測試是否為slab
			for (int j = 0; j < _descendantList[i]->_descendantList.size(); ++j) {
				if (_descendantList[i]->_descendantList[j]->_maxBackAnscenstorID <= _post_order_index) {
					is_slab = true;
					node_list.push_back(new_node = new node(_index));
					_descendantList[i]->split_arculation_point(node_list, node_list[node_list.size()-1]);
					break;
				}
			}
			//這個subtree只是leaf, 亦為slab
			if (_descendantList[i]->_descendantList.size() == 0) {
				is_slab = true;
				node_list.push_back(new_node = new node(_index));
				_descendantList[i]->split_arculation_point(node_list, node_list[node_list.size()-1]);
			}
		}
		//將屬於slab內的鄰居設成0, 並交給new_node
		while (iterate_index < _adjList.size() && _adjList[iterate_index]->_post_order_index <= _descendantList[i]->_post_order_index) {
			if (is_slab && _adjList[iterate_index]->_post_order_index < _post_order_index && _adjList[iterate_index]->isMarked()) {
				new_node->add_adj(_adjList[iterate_index]);
			    _adjList[iterate_index] = 0;
			}
			++iterate_index;
		}
	}
	//刪掉adjList中不該存在的nodes
	for (int i = 0; i < _adjList.size(); ++i) {
		while (i < _adjList.size() && _adjList[i] == 0) {
			_adjList[i] = _adjList[_adjList.size()-1];
		    _adjList.resize(_adjList.size()-1);
		}
	}
	//往剩下的部分search
	for (int i = 0; i < _descendantList.size(); ++i) {
		if (!_descendantList[i]->isMarked()) _descendantList[i]->split_arculation_point(node_list, modified);
	}
}

bool node::is_articulation_point() 
{
	for (int i = 0; i < _descendantList.size(); ++i) {
		if (_descendantList[i]->_maxBackAnscenstorID == _post_order_index) return true;
	}
	return false;
}
void node::reset_dfs_info() 
{
	_parent = 0;
	_post_order_index = INT_MAX;
	_maxBackAnscenstorID = INT_MAX;
	_descendantList.clear();
}
	
//-----------------------------------------------------------------------------------
// Descendant-list
//-----------------------------------------------------------------------------------

int node::num_curr_descendant() {return _curr_descendantList.size();}
node* node::curr_descendant(int i) {return _curr_descendantList[i];}
int node::num_descendant() {return _descendantList.size();}
node* node::descendant(int i) {return _descendantList[i];}
int node::num_remain_descendant() {return _remain_descendantList.size();}
node* node::remain_descendant(int i) {return _remain_descendantList[i];}

void node::get_remain_list(vector<node*> &list) 
{
	for (int i = 0; i < _remain_descendantList.size(); ++i) {
		if (_remain_descendantList[i]->is_finished() || (_remain_descendantList[i]->get_list_node() != 0)) continue;
		list.push_back(_remain_descendantList[i]);
	}
}
void node::get_current_list(vector<node*> &list) 
{
	for (int i = 0; i < _curr_descendantList.size(); ++i) {
		if (_curr_descendantList[i]->is_finished() || (_curr_descendantList[i]->get_list_node() != 0)) continue;
		list.push_back(_curr_descendantList[i]);
	}
}
void node::get_total_list(vector<node*> &list) 
{
	for (int i = 0; i < _descendantList.size(); ++i) {
		list.push_back(_descendantList[i]);
	}		
}

//將u放到current-descendant-list中
//若還沒被visit(i), 則先clear.
void node::add_node(node* u, int index) 
{
	if (!is_visited(index)) _curr_descendantList.clear();
	_curr_descendantList.push_back(u);
}

void node::clear_current_descendant() 
{
	_curr_descendantList.clear();
}


//terminal path上的p-node專用
//next為"唯一"label > index的, 即是往下的另一個terminal node
//i_tree為其他剩下label == index的node-list
void node::get_curr_descendantList(vector<node*> &i_tree, int index, node* &next) 
{
	for (int i = 0; i < _curr_descendantList.size(); ++i) {
		if (_curr_descendantList[i]->maxBackAnscenstorID() > index) next = _curr_descendantList[i];
		else i_tree.push_back(_curr_descendantList[i]);
	}
}
	
//上面的變種, 只傳回list
void node::get_curr_descendantList(vector<node*> &i_tree, int index) 
{
	for (int i = 0; i < _curr_descendantList.size(); ++i) {
		if (_curr_descendantList[i]->maxBackAnscenstorID() == index) i_tree.push_back(_curr_descendantList[i]);
	}
}

//將u加到remain-list中. (u必為c-node, 加到它的head的remain-list.)
void node::add_to_remain_list(node* u) {_remain_descendantList.push_back(u);}
//初始化, 將_curr_descendantList設成_descendantList
void node::init_remain_list() 
{
	_remain_descendantList = _descendantList;
}

//-----------------------------------------------------------------------------------
// Checks during constructing terminal path
//-----------------------------------------------------------------------------------

//僅適用於p-node
//檢查是否只有最大label的child可以超過index, 且有被visited
//回傳他在terminal path中向下的child
node* node::segment_one_check(int index) 
{
	//不可以有future back edge
	if (maxBackEdgeID() > index) return 0;
	node* return_node = 0;
	for (int i = 0; i < _remain_descendantList.size(); ++i) {
		//存在已經結束的node, 從list上刪除
		if (_remain_descendantList[i]->is_finished() || _remain_descendantList[i]->is_in_c_node()) {
			_remain_descendantList[i] = _remain_descendantList[_remain_descendantList.size()-1];
			_remain_descendantList.resize(_remain_descendantList.size()-1);
			--i;
			continue;
		}
		//存在sub-non-i-tree
		//它必須要被visited
		//如果存在沒被visited的, return false.
		//只能有一個這種child滿足
		if (_remain_descendantList[i]->maxBackAnscenstorID() > index) {
			if (_remain_descendantList[i]->is_visited(index) && return_node == 0) return_node = _remain_descendantList[i];
			else return 0;
		}
    }
	return return_node;
}

//僅適用於p-node
//檢查current-descendant-list中是否恰有一個label > index的node, 回傳之
//若都沒有, 那就表示這是terminal path的終點了
//不是seg. 2的話會return 0
node* node::segment_two_check(int index, bool &is_terminated) 
{
	is_terminated = false;
	node* return_node = 0;
	for (int i = 0; i < _curr_descendantList.size(); ++i) {
		if (_curr_descendantList[i]->maxBackAnscenstorID() > index) {
			if (return_node != 0) return 0;
			return_node = _curr_descendantList[i];
		}
	}
	if (return_node == 0) is_terminated = true;
	return return_node;
}

//基本上只有在 >= 3個terminal node時會return false
//有可能會有0個
pair<node*, node*> node::node_m_check(int index, bool &is_valid) 
{
	is_valid = true;
	pair<node*, node*> return_pair((node*)0, (node*)0);
	for (int i = 0; i < _curr_descendantList.size(); ++i) {
		if (_curr_descendantList[i]->maxBackAnscenstorID() > index) {
			if (return_pair.first == 0) return_pair.first = _curr_descendantList[i];
			else if (return_pair.second == 0) return_pair.second = _curr_descendantList[i];
			else is_valid = false;
		}
	}
	return return_pair;
}


//自己必須要被visited
//測試他是否下接i-tree, 等同於remain-list中所有nodes都被visited, 且label == i.
//此時curr-list == remain-list
//所以curr-list的node的label都 == i
//如果沒有children亦ok, 即remain-list = curr-list = empty. (表示他在這個iteration是leaf, 被node-i直接連到.)
bool node::is_i_tree(int index) 
{
	if (!is_visited(index) || maxBackEdgeID() > index) return false;	
	//還沒test過
	if (_i_tree_test.first != index) {
		bool test = true;
		for (int i = 0; i < _remain_descendantList.size(); ++i) {
			//存在已經結束的node, 從list上刪除
			if (_remain_descendantList[i]->is_finished() || _remain_descendantList[i]->is_in_c_node()) {
				_remain_descendantList[i] = _remain_descendantList[_remain_descendantList.size()-1];
				_remain_descendantList.resize(_remain_descendantList.size()-1);
				--i;
				continue;
			}
			//存在sub-non-i-tree, return false.
		    if (_remain_descendantList[i]->maxBackAnscenstorID() > index || !_remain_descendantList[i]->is_visited(index)) {
				test = false;
				break;
			}
	    }
		//記錄test結果
		_i_tree_test.first = index;
		_i_tree_test.second = test;
	}
	return _i_tree_test.second;
}


//-----------------------------------------------------------------------------------
// Post-processing
//-----------------------------------------------------------------------------------

//加回至原本embedding list
void node::add_to_final_embedding(vector<int> &emb_list) 
{
	list_node* prev = 0;
	list_node* curr;
	list_node* temp;
	if (!_emb->is_flip()) {
		curr = _emb->end(0);
	}
	else {
		curr = _emb->end(1);
	}
	while (curr != 0) {
		emb_list.push_back(curr->get_node()->index());
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
}



//-----------------------------------------------------------------------------------
// Debug
//-----------------------------------------------------------------------------------

void node::output_embedding(bool is_flip) 
{
	list_node* curr;
	list_node* prev = 0;
	list_node* temp;
	if (is_flip && _emb->is_flip() || !is_flip && !_emb->is_flip()) curr = _emb->end(0);
	else curr = _emb->end(1);
	cout << index() << " : ";
	while (curr != 0) {
		cout << curr->get_node()->index() << " ";
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	cout << endl;
}