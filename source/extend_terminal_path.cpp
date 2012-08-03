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

//-----------------------------------------------------------------------------------
// Seg. 2
//-----------------------------------------------------------------------------------

//p-node, 且為c-node child的case
//embed_direction表示embed的順序 (0表從1往0, 1表從0往1)
//即是recursive call的呼叫順序 <-這樣才能正確assign node_i的embedding順序
//flip表需不需要flip, 目前node之0, 1方向是否與新建c-node"不一致"
//come_from != direction就是要flip的意思
//例如 0.............(1, 0).............1就是不一致
//    其實這個(1,0)在子c-node中看起來會是(0,1), 所以come_from會是0
//令emb[0,...1]為原本的embedding
//以下假設方向為0->1, 若為1->0則都相反過來
//若有flip則新embedding為 [0]<- (emb[1,...0] , i-tree部分 , i*-tree部分 ) ->[1]  flip=true
//若無flip則新embedding為 [0]<- (emb[0,...1] , i-tree部分 , i*-tree部分 ) ->[1]  flip=false
//若原本沒有embedding, 則為 [0]<- (parent , i-tree部分 , i*-tree部分 ) ->[1]
void 
planarity_test::downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int come_from) 
{
	embed_p_node(u, list, node_i, parent, embed_direction, come_from);		
	embedding* my_embed = u->get_embedding();
	my_embed->set_flip(come_from != embed_direction);
}

//一般case
//處理extend terminal path
void 
planarity_test::downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction) 
{
	//p-node case
	if (u->type() == P_NODE) {
		if (u->maxBackAnscenstorID() > node_i->postOrderIndex()) {//在terminal path上
			parent->add_p_node(embed_direction, u);//add u to RBC
		}
		embed_p_node(u, list, node_i, parent, embed_direction, embed_direction); //修改過, add-node-dir不用switch
	}
	//c-node case
	else {
		//i-tree部分的direction和embed_direction是否不同  <-> 是否flip
		list_node* curr = 0;
	    list_node* prev = 0;
		list_node* temp = 0;
		int come_from;
		int start_dir;
		//0或1是否接i-tree
		bool is_0_i_tree = (u->get_RBC()->end_point(0)->get_node()->is_i_tree(node_i->postOrderIndex()));
		bool is_1_i_tree = (u->get_RBC()->end_point(1)->get_node()->is_i_tree(node_i->postOrderIndex()));
		bool flip;
		if (is_0_i_tree && is_1_i_tree) {//兩邊都是i-tree, 代表此c-node在embed inside的部分.
			flip = false;
			curr = u->get_RBC()->end_point(embed_direction);
			prev = (embed_direction==0)? list_node::NULL_0 : list_node::NULL_1;
		}
		else if (!is_0_i_tree && !is_1_i_tree) {//沒有i-tree部分, 因此只該有一個被visited的child
			//如果可以從embed_direction走過來的話, 不用flip
			if (u->get_RBC()->end_point(embed_direction)->get_node()->is_visited(node_i->postOrderIndex())) {
				curr = u->get_RBC()->end_point(embed_direction);
				prev = (embed_direction==0)? list_node::NULL_0 : list_node::NULL_1;
				flip = false;
			}
			//否則還是要
			else {
				curr = u->get_RBC()->end_point(switch_int(embed_direction));
				prev = (embed_direction==1)? list_node::NULL_0 : list_node::NULL_1;
				flip = true;
			}
		}
		//i-tree在某一邊時
		else if (is_0_i_tree) {
			curr = u->get_RBC()->end_point(0);
			prev = list_node::NULL_0;
			flip = embed_direction != 0;
		}
		else if (is_1_i_tree) {
			curr = u->get_RBC()->end_point(1);
			prev = list_node::NULL_1;
			flip = embed_direction != 1;
		}
		//把它add到新建c-node的list
		parent->add_to_containment_list(u->get_RBC(), flip);

		//用一個while-loop去embed所有visited的nodes
		start_dir = (prev==list_node::NULL_0)? 0 : 1;
		while (curr != ((start_dir==0)? list_node::NULL_1 : list_node::NULL_0)) {
			come_from = (prev == curr->n(0))? 0 : 1;
               //未被visited, 走到i*-tree的部分了. 把i*-tree部分add到RBC
			if (!curr->get_node()->is_visited(node_i->postOrderIndex())) {
				parent->add_list(u->get_RBC(), curr, embed_direction, switch_int(start_dir), come_from); //add to RBC
				break;
			}			
			//curr不是i-tree, 故是terminal node, 連同剩下未走到部分add到RBC
			if (!curr->get_node()->is_i_tree(node_i->postOrderIndex())) {
				parent->add_list(u->get_RBC(), curr, embed_direction, switch_int(start_dir), come_from); //add to RBC
				downward_embed(curr->get_node(), list, node_i, parent, embed_direction, come_from);
				break;
			}
			downward_embed(curr->get_node(), list, node_i, parent, embed_direction, come_from);
			temp = curr;
			curr = curr->get_next(prev);
			prev = temp;
		}
	}
}

//p-node時的embed
//add_node_direction係指把新的node加在embedding的哪邊
//分成有parent和無parent兩種...
//不處理flip, 不處理RBC extension
void 
planarity_test::embed_p_node(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int add_node_direction) 
{
    embedding* my_embed = u->get_embedding();
	vector<node*> i_tree;
	node* next = 0;
	u->get_curr_descendantList(i_tree, node_i->postOrderIndex(), next);
	//還沒開始embed...即此p-node尚未被加到c-node中
	if (my_embed->size() == 0) my_embed->add(u->parent(), add_node_direction, parent);
	//若被marked...則把node i加進去
	if (u->isMarked()) {
		my_embed->add(node_i, add_node_direction, parent);
		list.push_back(u);
	}
	if (next != 0) i_tree.push_back(next);
	for (int i = 0; i < i_tree.size(); ++i) {
		//p-node
		if (i_tree[i]->type() == P_NODE) {
			my_embed->add(i_tree[i], add_node_direction, parent);
			downward_embed(i_tree[i], list, node_i, parent, embed_direction);
		}
		//c-node
		else {
		    //embed_direction==1  : 只有在i-tree只分佈在end[0]端時才需要flip
			//embed_direction==0  : 只有在i-tree只分佈在end[1]端時才"不"需要flip
		    int test = i_tree[i]->get_RBC()->i_tree_location(node_i->postOrderIndex());
			bool need_flip = (embed_direction==1)? test == 0 : test != 1;
		    my_embed->add(i_tree[i]->get_RBC()->get_embedding(), add_node_direction, need_flip, parent);
		    downward_embed(i_tree[i], list, node_i, parent, embed_direction);
	    }
    }
   }


//-----------------------------------------------------------------------------------
// Seg. 1
//-----------------------------------------------------------------------------------

//從node_t開始向下, 直到node_m被遇到為止
//都不需要flip
//遇到p-node時, 一律embed到dir = 1那邊去
void 
planarity_test::downward_embed_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent) 
{
	if (u->type() == P_NODE) {
		embed_p_node_1(u, list_0, list_1, node_i, parent, 1, 1); //修改過, add-node-dir不用switch
	}
	else {
		list_node* curr = 0;
		list_node* prev = 0;
		list_node* temp = 0;
		int embed_direction;
		int come_from;
		node* next = u->get_RBC()->segment_one_check(node_i->postOrderIndex());
		if (next == 0) return; //達到node_m
		//把它add到新建c-node的list
		parent->add_to_containment_list(u->get_RBC(), false);
		embed_direction = 1; //先從1開始
		/*
		//single node case, 藉由看他的embedding是否flip已知方向性...
		if (u->get_RBC()->end_point(0) == u->get_RBC()->end_point(1)) {
			if (next->get_embedding()->is_flip()) come_from = 0;
			else come_from = 1;
			downward_embed_1(next, list_0, list_1, node_i, parent, embed_direction, come_from);
			return;
		}
		*/
		//用一個while-loop去embed所有visited的nodes
		//第一部分: 從end1到next
		curr = u->get_RBC()->end_point(1);
		prev = list_node::NULL_1;
		while (curr != list_node::NULL_0) {
			come_from = (prev == curr->n(0))? 0 : 1;
			if (curr->get_node() != next) downward_embed(curr->get_node(), list_1, node_i, parent, embed_direction, come_from);
			else {
				downward_embed_1(curr->get_node(), list_0, list_1, node_i, parent, embed_direction, come_from);
				break;
			}
			temp = curr;
			curr = curr->get_next(prev);
			prev = temp;
		}
		//第二部分:從end0到next之前
		curr = u->get_RBC()->end_point(0);
		prev = list_node::NULL_0;
		embed_direction = 0;
		while (curr->get_node() != next) {
			come_from = (prev == curr->n(0))? 0 : 1;
			downward_embed(curr->get_node(), list_1, node_i, parent, embed_direction, come_from);
			temp = curr;
			curr = curr->get_next(prev);
			prev = temp;
		}
	}
}

void 
planarity_test::downward_embed_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent, int embed_direction, int come_from) 
{
	embed_p_node_1(u, list_0, list_1, node_i, parent, embed_direction, come_from);		
	embedding* my_embed = u->get_embedding();
	my_embed->set_flip(come_from != embed_direction);
}

void 
planarity_test::embed_p_node_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent, int embed_direction, int add_node_direction) 
{
	if (u == 0) return;
	embedding* my_embed = u->get_embedding();
	vector<node*> i_tree;
	node* next = u->segment_one_check(node_i->postOrderIndex());
	if (next == 0) return;
	u->get_curr_descendantList(i_tree, node_i->postOrderIndex(), next);
	//還沒開始embed...即此p-node尚未被加到c-node中
	if (my_embed->size() == 0) my_embed->add(u->parent(), add_node_direction, parent);
	//若被marked...則把node i加進去
	if (u->isMarked()) {
		my_embed->add(node_i, add_node_direction, parent);
		list_1.push_back(u);
	}
	i_tree.push_back(next);
	for (int i = 0; i < i_tree.size(); ++i) {
		//p-node
		if (i_tree[i]->type() == P_NODE) {
			my_embed->add(i_tree[i], add_node_direction, parent);
			if (i_tree[i] != next) downward_embed(i_tree[i], list_1, node_i, parent, embed_direction);
			else downward_embed_1(i_tree[i], list_0, list_1, node_i, parent);
		}
		//c-node
		else {
		    //在seg. 1都不用flip c-node
		    my_embed->add(i_tree[i]->get_RBC()->get_embedding(), add_node_direction, false, parent);
		    if (i_tree[i] != next) downward_embed(i_tree[i], list_1, node_i, parent, embed_direction);
			else downward_embed_1(i_tree[i], list_0, list_1, node_i, parent);
	    }
    }
}


//-----------------------------------------------------------------------------------
// Node-m
//-----------------------------------------------------------------------------------

//assume傳進的node-m為valid
//會初始化RBC
//p-node case: 全部的i-tree都丟到1那邊
void 
planarity_test::embed_node_m(node* m, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent) 
{
	bool is_flip = false;
	//p-node case
	if (m->type() == P_NODE) {
		//基本宣告
		int add_node_direction = 1;
		int embed_direction = 1;
		embedding* my_embed = m->get_embedding();
		pair<node*, node*> mypair = m->node_m_check(node_i->postOrderIndex(), is_flip);//is_flip只是拿來湊的..
		vector<node*> i_tree;
		is_flip = false; //要改回來呀!!
			//上接p-node parent.
		if (my_embed->size() == 0) my_embed->add(m->parent(), 1, parent);
		//上接c-node
		else {
			//藉由往前走到終點, 以知走哪邊會到end_1, 再決定add-node-direction
			//node_m以上的c-node被embed inside, 所以RBC未被破壞.
			//這樣做是OK的
			list_node* prev = m->get_list_node();
			list_node* curr = prev->n(1);
			list_node* temp = 0;
			while (!curr->is_end()) {
				temp = curr;
				curr = curr->get_next(prev);
				prev = temp;
			}
			is_flip = (curr == list_node::NULL_0); //往1方向走到的終點是0的話代表有flip (從當前的c-node看下去)
			if (is_flip) add_node_direction = 0;
		}
		 
	    //若被marked...則把node i加進去 (新修改)
	    if (m->isMarked()) {
	    	my_embed->add(node_i, add_node_direction, parent);
		    list_1.push_back(m);
	    }
			//初始化RBC
		parent->init_RBC(m, (is_flip)? 0 : 1);
		//取出curr-descendant-list
		m->get_curr_descendantList(i_tree, node_i->postOrderIndex());
		if (mypair.second != 0) i_tree.push_back(mypair.second);
		if (mypair.first != 0) i_tree.push_back(mypair.first);
		//往下embed
		for (int i = 0; i < i_tree.size(); ++i) {
			//往0端時記得要改...
			if (i == i_tree.size() - 1 && mypair.first != 0) {
				embed_direction = 0;
				add_node_direction = 0;
			}
		    //p-node
		    if (i_tree[i]->type() == P_NODE) {
		    	my_embed->add(i_tree[i], add_node_direction, parent);
		    	downward_embed(i_tree[i], (embed_direction==1)? list_1 : list_0, node_i, parent, embed_direction);
		    }
		    //c-node
		    else {
		        //只有在i-tree分佈在end[0]端時才需要flip
		        int test = i_tree[i]->get_RBC()->i_tree_location(node_i->postOrderIndex());
		        my_embed->add(i_tree[i]->get_RBC()->get_embedding(), add_node_direction, test == 0, parent);
		        downward_embed(i_tree[i], (embed_direction==1)? list_1 : list_0, node_i, parent, embed_direction);		
			}
		}
	}
    //c-node case
	//此時c-node不可能只有single child, 否則它還是在seg. 1
	//就0, 1兩端分別走過, 直到not-visited的node被遇到為止...
	else {
		//基本宣告
		int start_dir;
		int embed_direction;
		int come_from;
		list_node* curr;
		list_node* prev = 0;
		list_node* temp;
		pair<node*, node*> mypair = m->get_RBC()->node_m_init_RBC(node_i->postOrderIndex(), parent);//init RBC
		//0, 1兩端分別從哪來的
		int come_from_0 = INT_MAX, come_from_1 = INT_MAX;
		if (mypair.first != 0) come_from_0 = (mypair.first->get_list_node()->n(0) == list_node::NULL_0)? 0 : 1;
		if (mypair.second != 0) come_from_1 = (mypair.second->get_list_node()->n(0) == list_node::NULL_1)? 0 : 1;

		//把它add到新建c-node的list
		parent->add_to_containment_list(m->get_RBC(), false); 
		//用while-loop去embed所有visited的nodes
		//先走0方向
		if (mypair.first != 0) {
	    	curr = m->get_RBC()->end_point(0);
			prev = list_node::NULL_0;
	    	start_dir = 0;
	    	embed_direction = 0;
	    	while (curr->get_node() != mypair.first) {
	    		if (!curr->get_node()->is_visited(node_i->postOrderIndex())) break;//未被visited, 走到i*-tree的部分了.
	    		come_from = (prev == curr->n(0))? 0 : 1;
	    		downward_embed(curr->get_node(), list_0, node_i, parent, embed_direction, come_from);
	    		temp = curr;
	    		curr = curr->get_next(prev);
	    		prev = temp;
	    	}
	    	//embed 0方向的終點
			//注意: 此時c-node已經被拆掉, 所以有可能curr不再連往prev
			if (prev == curr->n(0)) come_from = 0;
			else if (prev == curr->n(1)) come_from = 1; 
	    	else come_from = come_from_0;
	    	downward_embed(curr->get_node(), list_0, node_i, parent, embed_direction, come_from);
	    }
		//1方向相同...
		if (mypair.second != 0) {
		    start_dir = 1;
		    embed_direction = 1;
		    curr = m->get_RBC()->end_point(1);
			prev = list_node::NULL_1;
		    while (curr->get_node() != mypair.second) {
		    	if (!curr->get_node()->is_visited(node_i->postOrderIndex())) break;//未被visited, 走到i*-tree的部分了.
		    	come_from = (prev == curr->n(0))? 0 : 1;
		    	downward_embed(curr->get_node(), list_1, node_i, parent, embed_direction, come_from);
		    	temp = curr;
		    	curr = curr->get_next(prev);
		    	prev = temp;
		    }
			if (prev == curr->n(0)) come_from = 0;
			else if (prev == curr->n(1)) come_from = 1; 
	    	else come_from = come_from_1;
		    downward_embed(curr->get_node(), list_1, node_i, parent, embed_direction, come_from);
		}
	}
}