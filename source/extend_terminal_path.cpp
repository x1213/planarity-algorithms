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

//p-node, �B��c-node child��case
//embed_direction���embed������ (0��q1��0, 1��q0��1)
//�Y�Orecursive call���I�s���� <-�o�ˤ~�ॿ�Tassign node_i��embedding����
//flip��ݤ��ݭnflip, �ثenode��0, 1��V�O�_�P�s��c-node"���@�P"
//come_from != direction�N�O�nflip���N��
//�Ҧp 0.............(1, 0).............1�N�O���@�P
//    ���o��(1,0)�b�lc-node���ݰ_�ӷ|�O(0,1), �ҥHcome_from�|�O0
//�Oemb[0,...1]���쥻��embedding
//�H�U���]��V��0->1, �Y��1->0�h���ۤϹL��
//�Y��flip�h�sembedding�� [0]<- (emb[1,...0] , i-tree���� , i*-tree���� ) ->[1]  flip=true
//�Y�Lflip�h�sembedding�� [0]<- (emb[0,...1] , i-tree���� , i*-tree���� ) ->[1]  flip=false
//�Y�쥻�S��embedding, �h�� [0]<- (parent , i-tree���� , i*-tree���� ) ->[1]
void 
planarity_test::downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int come_from) 
{
	embed_p_node(u, list, node_i, parent, embed_direction, come_from);		
	embedding* my_embed = u->get_embedding();
	my_embed->set_flip(come_from != embed_direction);
}

//�@��case
//�B�zextend terminal path
void 
planarity_test::downward_embed(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction) 
{
	//p-node case
	if (u->type() == P_NODE) {
		if (u->maxBackAnscenstorID() > node_i->postOrderIndex()) {//�bterminal path�W
			parent->add_p_node(embed_direction, u);//add u to RBC
		}
		embed_p_node(u, list, node_i, parent, embed_direction, embed_direction); //�ק�L, add-node-dir����switch
	}
	//c-node case
	else {
		//i-tree������direction�Membed_direction�O�_���P  <-> �O�_flip
		list_node* curr = 0;
	    list_node* prev = 0;
		list_node* temp = 0;
		int come_from;
		int start_dir;
		//0��1�O�_��i-tree
		bool is_0_i_tree = (u->get_RBC()->end_point(0)->get_node()->is_i_tree(node_i->postOrderIndex()));
		bool is_1_i_tree = (u->get_RBC()->end_point(1)->get_node()->is_i_tree(node_i->postOrderIndex()));
		bool flip;
		if (is_0_i_tree && is_1_i_tree) {//���䳣�Oi-tree, �N��c-node�bembed inside������.
			flip = false;
			curr = u->get_RBC()->end_point(embed_direction);
			prev = (embed_direction==0)? list_node::NULL_0 : list_node::NULL_1;
		}
		else if (!is_0_i_tree && !is_1_i_tree) {//�S��i-tree����, �]���u�Ӧ��@�ӳQvisited��child
			//�p�G�i�H�qembed_direction���L�Ӫ���, ����flip
			if (u->get_RBC()->end_point(embed_direction)->get_node()->is_visited(node_i->postOrderIndex())) {
				curr = u->get_RBC()->end_point(embed_direction);
				prev = (embed_direction==0)? list_node::NULL_0 : list_node::NULL_1;
				flip = false;
			}
			//�_�h�٬O�n
			else {
				curr = u->get_RBC()->end_point(switch_int(embed_direction));
				prev = (embed_direction==1)? list_node::NULL_0 : list_node::NULL_1;
				flip = true;
			}
		}
		//i-tree�b�Y�@���
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
		//�⥦add��s��c-node��list
		parent->add_to_containment_list(u->get_RBC(), flip);

		//�Τ@��while-loop�hembed�Ҧ�visited��nodes
		start_dir = (prev==list_node::NULL_0)? 0 : 1;
		while (curr != ((start_dir==0)? list_node::NULL_1 : list_node::NULL_0)) {
			come_from = (prev == curr->n(0))? 0 : 1;
               //���Qvisited, ����i*-tree�������F. ��i*-tree����add��RBC
			if (!curr->get_node()->is_visited(node_i->postOrderIndex())) {
				parent->add_list(u->get_RBC(), curr, embed_direction, switch_int(start_dir), come_from); //add to RBC
				break;
			}			
			//curr���Oi-tree, �G�Oterminal node, �s�P�ѤU�����쳡��add��RBC
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

//p-node�ɪ�embed
//add_node_direction�Y����s��node�[�bembedding������
//������parent�M�Lparent���...
//���B�zflip, ���B�zRBC extension
void 
planarity_test::embed_p_node(node* u, vector<node*> &list, node* node_i, boundary_cycle* parent, int embed_direction, int add_node_direction) 
{
    embedding* my_embed = u->get_embedding();
	vector<node*> i_tree;
	node* next = 0;
	u->get_curr_descendantList(i_tree, node_i->postOrderIndex(), next);
	//�٨S�}�lembed...�Y��p-node�|���Q�[��c-node��
	if (my_embed->size() == 0) my_embed->add(u->parent(), add_node_direction, parent);
	//�Y�Qmarked...�h��node i�[�i�h
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
		    //embed_direction==1  : �u���bi-tree�u���G�bend[0]�ݮɤ~�ݭnflip
			//embed_direction==0  : �u���bi-tree�u���G�bend[1]�ݮɤ~"��"�ݭnflip
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

//�qnode_t�}�l�V�U, ����node_m�Q�J�쬰��
//�����ݭnflip
//�J��p-node��, �@��embed��dir = 1����h
void 
planarity_test::downward_embed_1(node* u, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent) 
{
	if (u->type() == P_NODE) {
		embed_p_node_1(u, list_0, list_1, node_i, parent, 1, 1); //�ק�L, add-node-dir����switch
	}
	else {
		list_node* curr = 0;
		list_node* prev = 0;
		list_node* temp = 0;
		int embed_direction;
		int come_from;
		node* next = u->get_RBC()->segment_one_check(node_i->postOrderIndex());
		if (next == 0) return; //�F��node_m
		//�⥦add��s��c-node��list
		parent->add_to_containment_list(u->get_RBC(), false);
		embed_direction = 1; //���q1�}�l
		/*
		//single node case, �ǥѬݥL��embedding�O�_flip�w����V��...
		if (u->get_RBC()->end_point(0) == u->get_RBC()->end_point(1)) {
			if (next->get_embedding()->is_flip()) come_from = 0;
			else come_from = 1;
			downward_embed_1(next, list_0, list_1, node_i, parent, embed_direction, come_from);
			return;
		}
		*/
		//�Τ@��while-loop�hembed�Ҧ�visited��nodes
		//�Ĥ@����: �qend1��next
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
		//�ĤG����:�qend0��next���e
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
	//�٨S�}�lembed...�Y��p-node�|���Q�[��c-node��
	if (my_embed->size() == 0) my_embed->add(u->parent(), add_node_direction, parent);
	//�Y�Qmarked...�h��node i�[�i�h
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
		    //�bseg. 1������flip c-node
		    my_embed->add(i_tree[i]->get_RBC()->get_embedding(), add_node_direction, false, parent);
		    if (i_tree[i] != next) downward_embed(i_tree[i], list_1, node_i, parent, embed_direction);
			else downward_embed_1(i_tree[i], list_0, list_1, node_i, parent);
	    }
    }
}


//-----------------------------------------------------------------------------------
// Node-m
//-----------------------------------------------------------------------------------

//assume�Ƕi��node-m��valid
//�|��l��RBC
//p-node case: ������i-tree�����1����
void 
planarity_test::embed_node_m(node* m, vector<node*> &list_0, vector<node*> &list_1, node* node_i, boundary_cycle* parent) 
{
	bool is_flip = false;
	//p-node case
	if (m->type() == P_NODE) {
		//�򥻫ŧi
		int add_node_direction = 1;
		int embed_direction = 1;
		embedding* my_embed = m->get_embedding();
		pair<node*, node*> mypair = m->node_m_check(node_i->postOrderIndex(), is_flip);//is_flip�u�O���Ӵꪺ..
		vector<node*> i_tree;
		is_flip = false; //�n��^�ӧr!!
			//�W��p-node parent.
		if (my_embed->size() == 0) my_embed->add(m->parent(), 1, parent);
		//�W��c-node
		else {
			//�ǥѩ��e������I, �H��������|��end_1, �A�M�wadd-node-direction
			//node_m�H�W��c-node�Qembed inside, �ҥHRBC���Q�}�a.
			//�o�˰��OOK��
			list_node* prev = m->get_list_node();
			list_node* curr = prev->n(1);
			list_node* temp = 0;
			while (!curr->is_end()) {
				temp = curr;
				curr = curr->get_next(prev);
				prev = temp;
			}
			is_flip = (curr == list_node::NULL_0); //��1��V���쪺���I�O0���ܥN��flip (�q��e��c-node�ݤU�h)
			if (is_flip) add_node_direction = 0;
		}
		 
	    //�Y�Qmarked...�h��node i�[�i�h (�s�ק�)
	    if (m->isMarked()) {
	    	my_embed->add(node_i, add_node_direction, parent);
		    list_1.push_back(m);
	    }
			//��l��RBC
		parent->init_RBC(m, (is_flip)? 0 : 1);
		//���Xcurr-descendant-list
		m->get_curr_descendantList(i_tree, node_i->postOrderIndex());
		if (mypair.second != 0) i_tree.push_back(mypair.second);
		if (mypair.first != 0) i_tree.push_back(mypair.first);
		//���Uembed
		for (int i = 0; i < i_tree.size(); ++i) {
			//��0�ݮɰO�o�n��...
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
		        //�u���bi-tree���G�bend[0]�ݮɤ~�ݭnflip
		        int test = i_tree[i]->get_RBC()->i_tree_location(node_i->postOrderIndex());
		        my_embed->add(i_tree[i]->get_RBC()->get_embedding(), add_node_direction, test == 0, parent);
		        downward_embed(i_tree[i], (embed_direction==1)? list_1 : list_0, node_i, parent, embed_direction);		
			}
		}
	}
    //c-node case
	//����c-node���i��u��single child, �_�h���٬O�bseg. 1
	//�N0, 1��ݤ��O���L, ����not-visited��node�Q�J�쬰��...
	else {
		//�򥻫ŧi
		int start_dir;
		int embed_direction;
		int come_from;
		list_node* curr;
		list_node* prev = 0;
		list_node* temp;
		pair<node*, node*> mypair = m->get_RBC()->node_m_init_RBC(node_i->postOrderIndex(), parent);//init RBC
		//0, 1��ݤ��O�q���Ӫ�
		int come_from_0 = INT_MAX, come_from_1 = INT_MAX;
		if (mypair.first != 0) come_from_0 = (mypair.first->get_list_node()->n(0) == list_node::NULL_0)? 0 : 1;
		if (mypair.second != 0) come_from_1 = (mypair.second->get_list_node()->n(0) == list_node::NULL_1)? 0 : 1;

		//�⥦add��s��c-node��list
		parent->add_to_containment_list(m->get_RBC(), false); 
		//��while-loop�hembed�Ҧ�visited��nodes
		//����0��V
		if (mypair.first != 0) {
	    	curr = m->get_RBC()->end_point(0);
			prev = list_node::NULL_0;
	    	start_dir = 0;
	    	embed_direction = 0;
	    	while (curr->get_node() != mypair.first) {
	    		if (!curr->get_node()->is_visited(node_i->postOrderIndex())) break;//���Qvisited, ����i*-tree�������F.
	    		come_from = (prev == curr->n(0))? 0 : 1;
	    		downward_embed(curr->get_node(), list_0, node_i, parent, embed_direction, come_from);
	    		temp = curr;
	    		curr = curr->get_next(prev);
	    		prev = temp;
	    	}
	    	//embed 0��V�����I
			//�`�N: ����c-node�w�g�Q�, �ҥH���i��curr���A�s��prev
			if (prev == curr->n(0)) come_from = 0;
			else if (prev == curr->n(1)) come_from = 1; 
	    	else come_from = come_from_0;
	    	downward_embed(curr->get_node(), list_0, node_i, parent, embed_direction, come_from);
	    }
		//1��V�ۦP...
		if (mypair.second != 0) {
		    start_dir = 1;
		    embed_direction = 1;
		    curr = m->get_RBC()->end_point(1);
			prev = list_node::NULL_1;
		    while (curr->get_node() != mypair.second) {
		    	if (!curr->get_node()->is_visited(node_i->postOrderIndex())) break;//���Qvisited, ����i*-tree�������F.
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