//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "planarity_test.h"
#include "miscellaneous.h"

#include <utility>
#include <climits>
#include <vector>
#include <fstream>
#include <iostream>
#include <queue>

using namespace::std;

void 
planarity_test::seg_2_extract_obstruction(node* current, node* node_m, int index) 
{
	node* u_0 = 0;
	node* u_1 = 0;
	node* v = 0;
	if (current->type() == P_NODE) {
		for (int i = 0; i < current->num_curr_descendant(); ++i) {
			if (!current->curr_descendant(i)->is_i_tree(index)) {
				if (u_0 == 0) u_0 = current->curr_descendant(i);
				else u_1 = current->curr_descendant(i);
			}
		}
		p_K_3_3_search_1(node_m, u_0, u_1, index);
	}
	else {
		//c-node:
		//如果有not-visited的child, 就用K_3_3_search_2
		//沒有的話就用K_5_search
		boundary_cycle* rbc = current->get_RBC();
		v = rbc->find_unvisited_child(index);
		if (v != 0) c_K_3_3_search_2(node_m, v, rbc->end_point(0)->get_node(), rbc->end_point(1)->get_node(), index);
		else {
			pair<node*, node*> mypair = rbc->node_m_check(index);
			c_K_5_search(node_m, mypair.first, mypair.second, index);
		}
	}
}

void 
planarity_test::three_terminal_extract_obstruction(node* m, int index) 
{
	node* u_0 = 0;
	node* u_1 = 0;
	node* u_2 = 0;
	for (int i = 0; i < m->num_curr_descendant(); ++i) {
		if (!m->curr_descendant(i)->is_i_tree(index)) {
			if (u_0 == 0) u_0 = m->curr_descendant(i);
			else if (u_1 == 0) u_1 = m->curr_descendant(i);
			else u_2 = m->curr_descendant(i);
		}
	}
	p_K_3_3_search_2(m, u_0, u_1, u_2, index);
}

void 
planarity_test::p_K_3_3_search_1(node* m, node* u_0, node* u_1, int index) 
{
	if (m->type() == C_NODE) m = m->get_RBC()->get_node_t();
	if (u_0->type() == C_NODE) u_0 = u_0->get_RBC()->get_node_t();
	if (u_1->type() == C_NODE) u_1 = u_1->get_RBC()->get_node_t();
	//init descendant-list
	vector<node*> m_list;
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	m->get_total_list(m_list);
	u_0->get_total_list(u_0_list);
	u_1->get_total_list(u_1_list);
	//find obstruction
	node* r = find_LCA(u_0, u_1);
	node* i = add_back_edge_from_descendant(u_0, u_0_list, index);
	add_back_edge_from_descendant(u_1, u_1_list, index);
	node* p_0 = add_higher_back_edge_from_descendant(u_0, u_0_list, index);
	node* p_1 = add_higher_back_edge_from_descendant(u_1, u_1_list, index);
	node* p_m = add_higher_back_edge_from_descendant(m, m_list, index, r);
	node* p = find_LCA(find_LCA(p_0, p_1), p_m);
	add_path_to_obstruction(i, r);
}

//3 term
void 
planarity_test::p_K_3_3_search_2(node* m, node* u_0, node* u_1, node* u_2, int index) 
{
	if (u_0->type() == C_NODE) u_0 = u_0->get_RBC()->get_node_t();
	if (u_1->type() == C_NODE) u_1 = u_1->get_RBC()->get_node_t();
	if (u_2->type() == C_NODE) u_2 = u_2->get_RBC()->get_node_t();
	//init descendant-list
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	vector<node*> u_2_list;
	u_0->get_total_list(u_0_list);
	u_1->get_total_list(u_1_list);
	u_2->get_total_list(u_2_list);
	//find obstruction
	add_path_to_obstruction(m, u_0);
	add_path_to_obstruction(m, u_1);
	add_path_to_obstruction(m, u_2);
	node* p_0 = add_higher_back_edge_from_descendant(u_0, u_0_list, index);
	node* p_1 = add_higher_back_edge_from_descendant(u_1, u_1_list, index);
	node* p_2 = add_higher_back_edge_from_descendant(u_2, u_2_list, index);
	add_back_edge_from_descendant(u_0, u_0_list, index); 
	add_back_edge_from_descendant(u_1, u_1_list, index); 
	add_back_edge_from_descendant(u_2, u_2_list, index); 
    find_LCA(find_LCA(p_0, p_1), p_2);
}

//during parallel search, the false i,i*-tree pattern
void 
planarity_test::c_K_3_3_search_1(node* v, node* u_0, node* u_1, int index) 
{
	//init descendant-list
	vector<node*> v_list;
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	v->get_current_list(v_list);
	u_0->get_remain_list(u_0_list);
	u_1->get_remain_list(u_1_list);
	//find obstruction
	node* r = c_node_square_linking(v, u_0, u_1);
	node* i = add_back_edge_from_descendant(v, v_list, index);
	node* p_0 = add_higher_back_edge_from_descendant(u_0, u_0_list, index);
	node* p_1 = add_higher_back_edge_from_descendant(u_1, u_1_list, index);
	node* p = find_LCA(p_0, p_1);
	find_LCA(r, p);
}

//
void 
planarity_test::c_K_3_3_search_2(node* m, node* v, node* u_0, node* u_1, int index) 
{ 
	//init descendant-list
	vector<node*> m_list;
	vector<node*> v_list;
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	m->get_total_list(m_list);
	v->get_remain_list(v_list);
	u_0->get_current_list(u_0_list);
	u_1->get_current_list(u_1_list);
	//find obstruction
	node* r = c_node_square_linking(v, u_0, u_1);
	node* v_p = add_higher_back_edge_from_descendant(v, v_list, index);
	node* m_p = add_higher_back_edge_from_descendant(m, m_list, index, r);
	node* i = add_back_edge_from_descendant(u_0, u_0_list, index);
	add_back_edge_from_descendant(u_1, u_1_list, index);
	node* p = find_LCA(v_p, m_p);
	find_LCA(i, p);
}

void 
planarity_test::c_K_5_search(node* m, node* u_0, node* u_1, int index) 
{ 
	if (m->type() == C_NODE) m = m->get_RBC()->get_node_t();
	//init descendant-list
	vector<node*> m_list;
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	m->get_total_list(m_list);
	u_0->get_remain_list(u_0_list);
	u_1->get_remain_list(u_1_list);
	//find obstruction
	node* r = c_node_triangle_linking(u_0, u_1);
	node* p_m = add_higher_back_edge_from_descendant(m, m_list, index, r);
	add_back_edge_from_descendant(u_0, u_0_list, index);
	add_back_edge_from_descendant(u_1, u_1_list, index);
	node* p_0 = add_higher_back_edge_from_descendant(u_0, u_0_list, index);
	node* p_1 = add_higher_back_edge_from_descendant(u_1, u_1_list, index);
	node* p = find_LCA(find_LCA(p_0, p_1), p_m);
	find_LCA(r, p);
}

//找u-node直屬的c-node
boundary_cycle* 
planarity_test::c_node_search(node* u) 
{
	list_node* curr = u->get_list_node();
	list_node* prev = curr->n(0);
	list_node* temp;
	while (!curr->is_end()) {
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	return prev->get_RBC();
}

//找到u, v的LCA並傳回, 把路途的edges都加到obstruction中
node* 
planarity_test::find_LCA(node* u, node* v) 
{
	//triv
	if (u == v) return u;
	//init
	initMark();
	vector<node*> u_list;
	vector<node*> v_list;
	node* lca = 0;
	u_list.push_back(u);
	v_list.push_back(v);
	u->mark();
	v->mark();
	//concurrent search
	while (true) {
		if (u_list[u_list.size()-1]->parent() != 0) {
			u_list.push_back(u_list[u_list.size()-1]->parent());
		    if (u_list[u_list.size()-1]->isMarked()) {
			    lca = u_list[u_list.size()-1];
			    break;
			}
			u_list[u_list.size()-1]->mark();
		}
		if (v_list[v_list.size()-1]->parent() != 0) {
			v_list.push_back(v_list[v_list.size()-1]->parent());
		    if (v_list[v_list.size()-1]->isMarked()) {
			    lca = v_list[v_list.size()-1];
			    break;
			}
			v_list[v_list.size()-1]->mark();
		}
	}
	//add to obstruction list
	if (u_list[0] != lca) {
		for (int i = 0; i < u_list.size()-1; ++i) {
		    add_to_obstruction(u_list[i]->index(), u_list[i+1]->index());
		    if (u_list[i+1] == lca) break;
		}
	}
	if (v_list[0] != lca) {
		for (int i = 0; i < v_list.size()-1; ++i) {
		    add_to_obstruction(v_list[i]->index(), v_list[i+1]->index());
		    if (v_list[i+1] == lca) break;
		}
	}
	return lca;
}

//從current-list下去找就對了
//找到u-subtree中任一點的adj-list中有著該post order index的鄰居(用back edge連)
//將該edge及u往下serach的道路加到obs-list
//回傳該node(back edge連上去的node)
node* 
planarity_test::add_back_edge_from_descendant(node* u, vector<node*> &list, int post_order_index) 
{
	for (int i = 0; i < u->degree(); ++i) {
	    if (u->adj(i)->postOrderIndex() == post_order_index) {
			add_to_obstruction(u->index(), u->adj(i)->index());
			return u->adj(i);
		}
	}
	node* temp;
	queue<node*> myqueue;
	for (int i = 0; i < list.size(); ++i) {
		if (list[i]->maxBackAnscenstorID() >= post_order_index) {
		    if (list[i]->type() == C_NODE) myqueue.push(list[i]->get_RBC()->get_node_t());
		    else myqueue.push(list[i]);
		}
	}
	while (!myqueue.empty()) {
		temp = myqueue.front();
			myqueue.pop();
	    for (int i = 0; i < temp->degree(); ++i) {
	    	if (temp->adj(i)->postOrderIndex() == post_order_index) {
				add_to_obstruction(temp->index(), temp->adj(i)->index());
			    add_path_to_obstruction(u, temp);
			    return temp->adj(i);
		    }
	    }
		for (int i = 0; i < temp->num_descendant(); ++i) {
			if (temp->descendant(i)->maxBackAnscenstorID() >= post_order_index) {
				myqueue.push(temp->descendant(i));
			}
		}
	}
	return 0;
}

//找到u-subtree中任一點的有連到 >post_order_index 的back edge
//將該edge及u往下serach的道路加到obs-list
//回傳該node(back edge連上去的node)
//forbid是禁止走到的
node* 
planarity_test::add_higher_back_edge_from_descendant(node* u, vector<node*> &list, int post_order_index, node* forbid) 
{
	for (int i = 0; i < u->degree(); ++i) {
	    if (u->adj(i)->postOrderIndex() > post_order_index) {
			add_to_obstruction(u->index(), u->adj(i)->index());
			return u->adj(i);
		}
	}
	node* temp;
	queue<node*> myqueue;
	for (int i = 0; i < list.size(); ++i) {
		if (list[i]->maxBackAnscenstorID() > post_order_index) {
		    if (list[i]->type() == C_NODE) myqueue.push(list[i]->get_RBC()->get_node_t());
		    else myqueue.push(list[i]);
			break;
		}
	}
	while (!myqueue.empty()) {
		temp = myqueue.front();
		myqueue.pop();
		if (temp == forbid) continue;
	    for (int i = 0; i < temp->degree(); ++i) {
	    	if (temp->adj(i)->postOrderIndex() > post_order_index) {
				add_to_obstruction(temp->index(), temp->adj(i)->index());
			    add_path_to_obstruction(u, temp);
			    return temp->adj(i);
		    }
	    }
		for (int i = 0; i < temp->num_descendant(); ++i) {
			if (temp->descendant(i)->maxBackAnscenstorID() > post_order_index) {
				myqueue.push(temp->descendant(i));
				//只需要一個label較大者就可以保證找得到.
				//所以可以break
				break;
			}
		}
	}
	return 0;
}

node* 
planarity_test::c_node_square_linking(node* v, node* u_0, node* u_1) 
{
	boundary_cycle* c_node = c_node_search(v);
	node* r = c_node->head();
	node* s_0 = find_LCA(u_0, v);
	node* s_1 = find_LCA(u_1, v);
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	if (s_0 == v) {
		u_0->get_total_list(u_0_list);
		add_back_edge_from_descendant(u_0, u_0_list, r->postOrderIndex());
	}
	else {
		add_path_to_obstruction(r, s_0);
	}
	if (s_1 == v) {
		u_1->get_total_list(u_1_list);
		add_back_edge_from_descendant(u_1, u_1_list, r->postOrderIndex());
	}
	else {
		add_path_to_obstruction(r, s_1);
	}
	return c_node->get_node_t();
}

node* 
planarity_test::c_node_triangle_linking(node* u_0, node* u_1) 
{
	boundary_cycle* c_node = c_node_search(u_0);
	node* r = c_node->head();
	node* v = find_LCA(u_0, u_1);
	vector<node*> u_0_list;
	vector<node*> u_1_list;
	if (u_0 == v) {
		//we must have u_1 != v, and u_1 is lower than v.
		add_path_to_obstruction(r, v);
		u_1->get_total_list(u_1_list);
		add_back_edge_from_descendant(u_1, u_1_list, r->postOrderIndex());
	}
	else if (u_1 == v) {
		//we must have u_0 != v, and u_0 is lower than v.
		add_path_to_obstruction(r, v);
		u_0->get_total_list(u_0_list);
		add_back_edge_from_descendant(u_0, u_0_list, r->postOrderIndex());
	}
	else {
		u_0->get_total_list(u_0_list);
		u_1->get_total_list(u_1_list);
		add_back_edge_from_descendant(u_0, u_0_list, r->postOrderIndex());
		add_back_edge_from_descendant(u_1, u_1_list, r->postOrderIndex());
	}
	return c_node->get_node_t();
}

//把path c-> ... -> p加到obstruction
void 
planarity_test::add_path_to_obstruction(node* p, node* c) 
{
	while (c != p) {
		add_to_obstruction(c->index(), c->parent()->index());
		c = c->parent();
	}
}

void
planarity_test::add_to_obstruction(int i, int j) 
{
	if (i <= j) _obstruction.push_back(pair<int, int>(i, j));
	else _obstruction.push_back(pair<int, int>(j, i));
}

void 
planarity_test::radix_sort_obstruction_list() 
{
vector<vector<pair<int, int> > > bucket;
	bucket.resize(_node_list.size());
	for (int i = 0; i < _obstruction.size(); ++i) {
		bucket[_obstruction[i].second].push_back(_obstruction[i]);
	}
	_obstruction.clear();
	for (int i = 0; i < bucket.size(); ++i) {
		for (int j = 0; j < bucket[i].size(); ++j) {
		    _obstruction.push_back(bucket[i][j]);
		}
	}
	bucket.clear();
	bucket.resize(_node_list.size());
	for (int i = 0; i < _obstruction.size(); ++i) {
		bucket[_obstruction[i].first].push_back(_obstruction[i]);
	}
	_obstruction.clear();
	for (int i = 0; i < bucket.size(); ++i) {
		for (int j = 0; j < bucket[i].size(); ++j) {
		    if (_obstruction.empty() || bucket[i][j] != _obstruction[_obstruction.size()-1]) _obstruction.push_back(bucket[i][j]);
		}
	}
}
