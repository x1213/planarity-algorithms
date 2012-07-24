//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------


#include "planarity_test.h"
#include "miscellaneous.h"
#include "simple_graph.h"

#include <utility>
#include <climits>
#include <vector>
#include <fstream>
#include <iostream>
#include <queue>

using namespace::std;

planarity_test::planarity_test() {}
planarity_test::~planarity_test() 
{
	for (int i = 0; i < _c_node_list.size(); ++i) delete _c_node_list[i];
	for (int i = 0; i < _dfs_list.size(); ++i) delete _dfs_list[i];
}

node* planarity_test::new_c_node(int i) 
{
	node* u = new node(i, C_NODE);
	_c_node_list.push_back(u);
	return u;
}

void planarity_test::determine_biconnected_component() 
{
	vector<node*> post_order_list;
	//�򥻤u�@
	postOrderTraversal(_dfs_list, post_order_list);
	sortAdjList(post_order_list);
	initLabeling(post_order_list);
	//�}�l����biconnected component
	initMark();
	for (int i = post_order_list.size()-1; i >= 0; --i) {
		if (!post_order_list[i]->isMarked()) {
	        post_order_list[i]->split_arculation_point(post_order_list);
		}
	}
	/*debug... check integrity
	for (int i = 0; i < post_order_list.size(); ++i) {
		node* node_i = post_order_list[i];
		for (int j = 0; j < post_order_list[i]->degree(); ++j) {
			node* node_j = post_order_list[i]->adj(j);
			bool test = false;
			for (int k = 0; k < node_j->degree(); ++k) {
				if (node_j->adj(k) == node_i) test = true;
			}
			if (!test) {
				test = true;
			}
		}
	}
	*/
	post_order_list[_dfs_list.size()-1] = post_order_list[post_order_list.size()-1];
	post_order_list.resize(post_order_list.size()-1);
	//���s�]�w
	for (int i = 0; i < post_order_list.size(); ++i) {
		post_order_list[i]->reset_dfs_info();
	}
	_dfs_list.clear();
	//���sdfs�򥻤u�@, �s��_dfs_list�Y�������n���@��disjoint biconnected components.
	postOrderTraversal(post_order_list, _dfs_list);
	sortAdjList(_dfs_list);
	initLabeling(_dfs_list);
	//��l��remain-list
	for (int i = 0; i < _dfs_list.size(); ++i) {
		_dfs_list[i]->init_remain_list();
	}
}

void planarity_test::postOrderTraversal(vector<node*> &node_list, vector<node*> &post_order_list) 
{
	initMark();
	int postOrderID = 0;
	for (int i = 0; i < node_list.size(); ++i) {
		if (!node_list[i]->isMarked()) {
			node_list[i]->DFS_visit(post_order_list, postOrderID);
		}
	}
}
void planarity_test::initLabeling(vector<node*> &post_order_list) {
	for (int i = 0; i < post_order_list.size(); ++i) {
		post_order_list[i]->setMaxBackAnscenstorID();
	}
}

void planarity_test::sortAdjList(vector<node*> &post_order_list) 
{
	vector<vector<node*>> vecList;
	vecList.resize(post_order_list.size());
	for (int i = 0; i < post_order_list.size(); ++i) {
		for (int j = 0; j < post_order_list[i]->degree(); ++j) {
			vecList[post_order_list[i]->adj(j)->postOrderIndex()].push_back(post_order_list[i]);
		}
	}
	for (int i = 0; i < post_order_list.size(); ++i) {
		post_order_list[i]->setAdjList(vecList[i]);
	}
}
	
//�������˦n��, �}�l��c_node����flip�쥿�T����V
void planarity_test::c_node_flip() 
{
	initMark();
	for (int i = _c_node_list.size() - 1; i >= 0; --i) {
		if (!_c_node_list[i]->isMarked()) {
			//��embedding���head�W
			_c_node_list[i]->get_RBC()->head()->get_embedding()->add(_c_node_list[i]->get_RBC()->get_embedding(), 1, false, _c_node_list[i]->get_RBC());
			//flip
			_c_node_list[i]->get_RBC()->recursively_flip(false);
		}
	}
}

void planarity_test::recover_embedding() 
{
	_embed_list.resize(_node_list.size());
	for (int i = 0; i < _dfs_list.size(); ++i) {
		if (_dfs_list[i]->get_embedding()->get_RBC() != 0 && _dfs_list[i]->get_embedding()->get_RBC()->is_flip()) _dfs_list[i]->get_embedding()->flip_end_point();
		_dfs_list[i]->add_to_final_embedding(_embed_list[_dfs_list[i]->index()]);
	}
}
	
//�o��biconnected component�u�O�@��edge
void planarity_test::trivial_embed(node* node_i, node* node_t) 
{
	node_t->get_embedding()->add(node_i, 1, 0);
	node_i->get_embedding()->add(node_t, 1, 0);
}

bool planarity_test::back_edge_traversal() 
{
	int iterate_index = 0; // �ثe�bnode_i����adj-list�W���쪺��m
	vector<node*> traverse_list; // �bt-subtree�U��i-back-edge��nodes
	node* node_i = 0; // iteration i
	node* node_t = 0; // i���Y��child t
	for (int i = 0; i < _dfs_list.size(); ++i) {
		node_i = _dfs_list[i];
		iterate_index = 0;
		for (int j = 0; j < node_i->num_descendant(); ++j) {
			traverse_list.clear();
			node_t = node_i->descendant(j);
			while (iterate_index < node_i->degree() && node_i->adj(iterate_index)->postOrderIndex() <= node_t->postOrderIndex()) {
				traverse_list.push_back(node_i->adj(iterate_index));
				++iterate_index;
			}
			if (!back_edge_traversal(node_i, node_t, traverse_list)) return false;
		}
	}
	return true;
}

bool planarity_test::back_edge_traversal(node* node_i, node* node_t, vector<node*> &traverse_list) 
{
	vector<node*> c_node_list;
	//�S��back edge, return.
	//traverse_list���̫�@�Ӥ@�w�Onode_t
	if (traverse_list.size() == 1) {
		if (node_t->degree() == 1) trivial_embed(node_i, node_t);
		return true;
	}
	//clear current-descendant-list
	for (int i = 0; i < traverse_list.size(); ++i) {
		traverse_list[i]->clear_current_descendant();
	}
	//visit
	for (int i = 0; i < traverse_list.size() - 1; ++i) {
		simple_upwardTraversal(traverse_list[i], node_i->postOrderIndex(), c_node_list);
	}
	//check i-tree, i*-tree pattern
	initMark(); 
	for (int i = 0; i < traverse_list.size() - 1; ++i) {
		if (!check_upwardTraversal(traverse_list[i], node_i->postOrderIndex())) return false;
	}
	//mark ����back edge��nodes
	initMark(); 
	for (int i = 0; i < traverse_list.size() - 1; ++i) {
		traverse_list[i]->mark();
	}
	//build c-node
	if (!build_c_node(node_t, node_i)) return false;
	//��Ҧ����L��c-node�]��finished
	for (int i = 0; i < c_node_list.size(); ++i) {
		c_node_list[i]->finish();
	}
	return true;
}

//�V�Wtraverse
//�Y�ۤv�w�Qvisited, �h��.
//�Y����i-node, �簱.
//�_�h, �N�ۤvadd��parent��children-list�� <- current iteration��
//�p�G�J��c-node�h��parallel search. <- c-node����add child���ʧ@, ��c-node��parent, �Yc->head(), �|add��c-node��child.
//�Ҧ����쪺node���|�Q�]��visited(i)
//�O�U�Ҧ����Lc-node
void planarity_test::simple_upwardTraversal(node* s_node, int index, vector<node*> &c_node_list) 
{
	node* current = s_node;
	boundary_cycle* c = 0;
	while (true) {
	    if (current->is_visited(index)) break;	//�w�Q���L		
		if (current->postOrderIndex() == index) {//�F��node_i
			break;
		}
		current->visit(index);
		if (current->get_list_node() != 0) {//��ܥL�Oc-node�W��list-node
			c = simple_c_node_traverse(current);
			if (c == 0) return;
			current = c->head();
			if (c->c_node()->is_visited(index)) break;
			else {
				c_node_list.push_back(c->c_node());
				current->add_node(c->c_node(), index);
				c->c_node()->visit(index);
			}
		}
		else {
			current->parent()->add_node(current, index);
			current = current->parent();
		}
	}
	return;
}

//check i, i* -tree pattern
//�Ҧ�c-node�W�Qvisited��nodes�������n����g�Ѥ@�s��i-tree����head
bool planarity_test::check_upwardTraversal(node* s_node, int index) 
{
	node* current = s_node;
	boundary_cycle* c = 0;
	while (true) {
	    if (current->isMarked()) break;	//�w�Q���L		
		if (current->postOrderIndex() == index) {//�F��node_i
			break;
		}
		//init i-tree testing
		current->is_i_tree(index);
		current->mark();
		if (current->get_list_node() != 0) {//��ܥL�Oc-node�W��list-node
			c = parallel_search(current, index);
			if (c == 0) {
				return false;
			}
			current = c->head();
			if (c->c_node()->isMarked()) break;
			else {
				c->c_node()->mark();
			}
		}
		else {
			current = current->parent();
		}
	}
	return true;
}
//�p�G�L�Oend-point, �N�Ǧ^RBC
//�_�h�Ǧ^0
boundary_cycle* planarity_test::simple_c_node_traverse(node* node_s) 
{
	/*Debug...
	if (node_s->get_list_node()->get_RBC() != 0) {
		if (!node_s->get_list_node()->n(0)->is_end() && !node_s->get_list_node()->n(1)->is_end()){
			system("pause");
		}
	}
	*/
	return node_s->get_list_node()->get_RBC();
}

//node_s���Yc-node����list-node
//��parallel search (�ȭ����U��i-tree��nodes)
//�`�N: �o�N��i*-tree�û����|�Qsearch��, �ҥH�S�Q���쪺node�����|�b��iteration���Qembed inside.
//      �B�Ҧ����쪺node���Fterminal node�H�~���|�Qembed inside.
//�J�즳node�Oc-node�ɰ�
boundary_cycle* planarity_test::parallel_search(node* node_s, int index) 
{
	list_node* curr[2];
	list_node* prev[2];
	list_node* temp = 0;
	vector<list_node*> traversed[2];
	//visit node_s
	node_s->visit(index);
	//�p�G�w��RBC, ����return��.
	if (node_s->get_list_node()->get_RBC() != 0) return node_s->get_list_node()->get_RBC();
	curr[0] = node_s->get_list_node()->n(0);
	curr[1] = node_s->get_list_node()->n(1);
	prev[0] = prev[1] = node_s->get_list_node();
	traversed[0].push_back(prev[0]);
	traversed[1].push_back(prev[1]);
       while (true) {
		//���䳣���q, return 0;
		if ((!curr[0]->get_node()->is_i_tree(index)) && (!curr[1]->get_node()->is_i_tree(index))) {
			//find kuratowski subgraph
			c_K_3_3_search_1(node_s, curr[0]->get_node(), curr[1]->get_node(), index);
			return 0;
		}
	    for (int i = 0; i <= 1; ++i) {
	    	if (!curr[i]->get_node()->is_i_tree(index)) continue;//����label > i��, ��.
			traversed[i].push_back(curr[i]);
	    	if (curr[i]->get_RBC()!= 0) {
				//�⨫�L��nodes�]RBC
				for (int j = 0; j < traversed[i].size(); ++i) {
					traversed[i][j]->set_RBC(curr[i]->get_RBC());
					return curr[i]->get_RBC();
				}
			}
	    	temp = curr[i];
	    	curr[i] = curr[i]->get_next(prev[i]);
	    	prev[i] = temp;
		}
	}
}
bool planarity_test::build_c_node(node* node_t, node* node_i) 
{
	//init, test validity.
	int index = node_i->postOrderIndex();
	node* c_node = new_c_node(node_i->index());
	boundary_cycle* new_RBC = new boundary_cycle(c_node, node_i, node_t);
	c_node->init_c_node(node_i, node_t, new_RBC);
	node* node_m = check_RBC_validity(node_t, index);
	//�̧ǩ�0�M��1��V��back edge to i��node-list
	vector<node*> toward_0;
	vector<node*> toward_1;
	//�w�g��root�F
	if (node_i->parent() == 0) {
		downward_embed(node_t, toward_1, node_i, new_RBC, 1);
	}
	//�@�뱡��
	else {
		if (node_m == 0) {
			return false;
		}
	    //embedding, set boundary path
	    if (node_m != node_t) downward_embed_1(node_t, toward_0, toward_1, node_i, new_RBC);
	    embed_node_m(node_m, toward_0, toward_1, node_i, new_RBC);
	}
	//parent-link
	c_node->set_parent(node_i);
	//embedding node_i in c-node
	//�`�N: ��V�O�q1��0, �|�ϹL��.
	embedding* new_emb = new embedding(node_i);
	new_emb->add(node_t, 1, new_RBC);
	for (int i = 0; i < toward_0.size(); ++i) new_emb->add(toward_0[i], 1, new_RBC);
	for (int i = 0; i < toward_1.size(); ++i) new_emb->add(toward_1[i], 0, new_RBC);
	c_node->get_RBC()->set_embedding(new_emb);
	c_node->get_RBC()->contract_list_node();
	//�Nc-node�[��node_i��remain-list
	node_i->add_to_remain_list(c_node);
	//output_partial_embedding();
	return true;
}

//�Ǧ^node_m
//�Y���X�k, �Χ䤣��node-m, �h�Ǧ^0
node* planarity_test::check_RBC_validity(node* node_t, int index) 
{
	bool test = true;
	node* curr = node_t;
	node* temp = 0;
	node* node_m = 0;
	pair<node*, node*> child_of_m;
	//segment 1 : node_m�H�W
	while (true) {
		if (curr->type() == P_NODE) temp = curr->segment_one_check(index);
		else temp = curr->get_RBC()->segment_one_check(index);
		if (temp != 0) curr = temp;
		else {
			node_m = curr;
			break;
		}
	}
	if (node_m->type() == P_NODE) child_of_m = node_m->node_m_check(index, test);
	else child_of_m = node_m->get_RBC()->node_m_check(index);
	if (!test) {
		//3 terminal case : p-node
		//K_3_3
		three_terminal_extract_obstruction(node_m, index);
		return 0;
	}
	//segment 2: end[0]��V
	curr = child_of_m.first;
	if (curr != 0) while (true) {
		if (curr->type() == P_NODE) temp = curr->segment_two_check(index, test);
		else temp = curr->get_RBC()->segment_two_check(index, test);
		if (test) break;
		else if (temp == 0) {
			seg_2_extract_obstruction(curr, node_m, index);
			return 0;
		}
		curr = temp;
	}
	//segment 2: end[1]��V
	curr = child_of_m.second;
	if (curr != 0) while (true) {
		if (curr->type() == P_NODE) temp = curr->segment_two_check(index, test);
		else temp = curr->get_RBC()->segment_two_check(index, test);
		if (test) break;
		else if (temp == 0) {
			seg_2_extract_obstruction(curr, node_m, index);
			return 0;
		}
		curr = temp;
	}
	return node_m;
}

void planarity_test::brute_force_check_emb() 
{
	int current_node;
	int start_node;
	int end_node;
	int curr;
	int prev;
	int temp;
	
	for (int i = 0; i < _embed_list.size(); ++i) {
		if (_embed_list[i].size() != _adj_list[i].size()) {
			int ccc = 0;
		}
		current_node = i;
		for (int j = 0; j < _embed_list[i].size(); ++j) {
			start_node = _embed_list[i][(j+1)%_embed_list[i].size()];
			end_node = _embed_list[i][j];
			curr = start_node;
			prev = current_node;
			int count = 0;
			while (true) {
				temp = curr;
				curr = emb_get_next(curr, prev);
				prev = temp;
				++count;
				if (count > 10000) {
					int fff = 0;
				}
				if (current_node == curr && prev == end_node) break;
			}
		}
	}
}

int planarity_test::emb_get_next(int curr, int prev) 
{
	for (int i = 0; i < _embed_list[curr].size(); ++i) {
		if (_embed_list[curr][i] == prev) {
			return _embed_list[curr][(i+1)%_embed_list[curr].size()];
		}
	}
	int iii = 5;
}


void planarity_test::output_partial_embedding() 
{
	initMark();
	for (int i = _c_node_list.size() - 1; i >= 0; --i) {
		if (!_c_node_list[i]->isMarked()) {
			//flip
			_c_node_list[i]->get_RBC()->recursively_flip(false);
		}
	}
	list_node* curr = _c_node_list[_c_node_list.size()-1]->get_RBC()->end_point(0);
	list_node* prev = list_node::NULL_0;
	list_node* temp;
	cout << "Current c-node: " << _c_node_list[_c_node_list.size()-1]->get_RBC()->head()->index() << " : ";
	while (curr != list_node::NULL_1) {
		if (curr == 0) break;
		cout << curr->get_node()->index() << " ";
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	cout << endl << "embedding: " << endl;
		for (int i = 0; i < _dfs_list.size(); ++i) {
		if (_dfs_list[i]->get_embedding()->size() == 0) continue;
		_dfs_list[i]->output_embedding((_dfs_list[i]->get_embedding()->get_RBC()==0)? false : _dfs_list[i]->get_embedding()->get_RBC()->is_flip());
	}
		cout << "head of c-node:" << endl;
	curr = _c_node_list[_c_node_list.size()-1]->get_RBC()->get_embedding()->end(0);
	prev = 0;
	while (curr != 0) {
		cout << curr->get_node()->index() << " ";
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	cout << endl;
}
    