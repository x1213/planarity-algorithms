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
	//�]�mhead�b��c-node����embedding, ��V��1->0, �S��flip.
void boundary_cycle::set_embedding(embedding* emb) 
{
	_emb = emb;
}

void boundary_cycle::contract_list_node() 
{
	//�o��list�O�����٦��i��Qtraverse��list-nodes
	for (int i = 0; i < _possible_contract_list.size(); ++i) {
		_possible_contract_list[i]->set_RBC(0);//�H���H��traverse�ɨ������RBC
		if (!_possible_contract_list[i]->get_node()->is_finished()) continue;
		//����end-point
		if (_possible_contract_list[i] == _end_point[0]) _end_point[0] = _end_point[0]->get_next(list_node::NULL_0);
		if (_possible_contract_list[i] == _end_point[1]) _end_point[1] = _end_point[1]->get_next(list_node::NULL_1);
		_possible_contract_list[i]->contract();
	}
	//�̫�]�nend-point��RBC
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

//node-m�Op-node ::�ݭק�
//dir1�O��1��V
void boundary_cycle::init_RBC(node* u, int dir1) 
{
	//�w�g���b����c-node��
	if (u->get_list_node() == 0) {
		u->set_list_node(new list_node(u));
	}
	_end_point[0] = _end_point[1] = u->get_list_node();
	if (dir1 == 1) _end_point[0]->set_neighborhood(list_node::NULL_0, list_node::NULL_1);
	else _end_point[0]->set_neighborhood(list_node::NULL_1, list_node::NULL_0);
	_possible_contract_list.push_back(u->get_list_node());//�[��i�ব�Y��list
	//debug_rbc();
}
	
//node-m�Oc-node
//end_dir0�On0��0����V, 1�P�z.
void boundary_cycle::init_RBC(list_node* n0, list_node* n1, int end_dir0, int end_dir1) 
{
	_end_point[0] = n0;
	_end_point[1] = n1;
	n0->set_neighbor(end_dir0, list_node::NULL_0);
	n1->set_neighbor(end_dir1, list_node::NULL_1);
	_possible_contract_list.push_back(n0);//�[��i�ব�Y��list
	if (n1 != n0) _possible_contract_list.push_back(n1);
	//debug_rbc();
}

//���e������empty
//��V�n���T
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
	_possible_contract_list.push_back(add->get_list_node());//�[��i�ব�Y��list
	//debug_rbc();
}
	
//��b add�i��...(segment 2��c-node)
//end node�Ob���nadd���̲׺ݪ�node
//dir�Oembed direction, �N�O�nadd�b���Ӥ�V.
//start_dir�Ob���n�q���ݶ}�l
//end_dir�Oend_node���ݭn��0
void boundary_cycle::add_list(boundary_cycle* b, list_node* end_node, int dir, int start_dir, int end_dir) 
{
	//�[��i�ব�Y��list
	_possible_contract_list.push_back(b->_end_point[start_dir]);
	if (b->_end_point[start_dir] != end_node) _possible_contract_list.push_back(end_node);
	
       //set endnode��neighbor
	end_node->set_neighbor(end_dir, (dir==0)? list_node::NULL_0 : list_node::NULL_1);
		//set b��endpoint��neighbor
	if (b->_end_point[start_dir]->n(0) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(0, _end_point[dir]);
	else if (b->_end_point[start_dir]->n(1) == ((start_dir==0)? list_node::NULL_0 : list_node::NULL_1)) b->_end_point[start_dir]->set_neighbor(1, _end_point[dir]);
	else {
		dir = 100;
	}
		//set _end_point[dir]��neighbor
	if (_end_point[dir]->n(0) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(0, b->_end_point[start_dir]);
	else if (_end_point[dir]->n(1) == ((dir==0)? list_node::NULL_0 : list_node::NULL_1)) _end_point[dir]->set_neighbor(1, b->_end_point[start_dir]);
	else {
		dir = 100;
	}
		_end_point[dir] = end_node;//���sassign endpoint
	//debug_rbc();
}
	
//�ˬd��c-node�O�_�i�bsegment-1 (�bnode_m�H�W������)
//�Y�O�u��꦳�@��node��label > index
//�ӥ������n�Qvisited
//�̫�Ǧ^��node
//return 0 ��� false
node* boundary_cycle::segment_one_check(int index) 
{
	list_node* curr = _end_point[0];
	list_node* prev = list_node::NULL_0;
	list_node* temp = 0;
	node* return_node = 0;
	while (curr != list_node::NULL_1) {
		//���Oi-tree���ܴN�Oterminal node
		//�u�঳�@��
		if (!curr->get_node()->is_i_tree(index)) {
			if (return_node == 0) return_node = curr->get_node();
			else return 0;
		}
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	//terminal node�@�w�n�Qvisited
	if (return_node->is_visited(index)) return return_node;
	else return 0;
}
	//�u�঳�@��i-tree�϶�
//�̫�Ǧ^�Ӱ϶�����t��node
//���i��|�Ǧ^�@��label <= i��node, �o��ܦ�node�w�g�Oterminal path�����I�F.
//dir�Oi*-tree��������V(������endpoint�}�l)
node* boundary_cycle::segment_two_check(int index, bool &is_terminated) 
{
	is_terminated = false;
	list_node* curr = 0;
	list_node* prev = 0;
	list_node* temp = 0;
	node* return_node = 0;
	//0-��V�Qvisited
	if (_end_point[0]->get_node()->is_visited(index)) {
		curr = _end_point[0];
		prev = list_node::NULL_0;
	}
	if (_end_point[1]->get_node()->is_visited(index)) {
		//���䳣�Qvisited, �ߤ@�i��N�O��n��@���I��terminal path�W��node, ��L����i-tree
		//�]�@��segment_one_check, �p�G��true, �B�^�Ǫ�node��end_point, �N���X�k.
		if (curr != 0) {
			return_node = segment_one_check(index);
			if (return_node == _end_point[0]->get_node() || return_node == _end_point[1]->get_node()) return return_node;
			return 0;
		}
		//���`����, �u��1-��V�Qvisited
		curr = _end_point[1];
		prev = list_node::NULL_1;
	} 
	while (!curr->is_end()) {
		//����curr�S�Qvisited����, ����prev�N�O�ҨD.
		if (!curr->get_node()->is_visited(index)) {
			//�p�Gprev�Oi-tree, �N��ܩҦ�visited��subtree���Oi-tree
			//�^��prev, �O�Uis_terminated
			if (prev->get_node()->is_i_tree(index)) is_terminated = true;
			return prev->get_node();
		}
		temp = curr;
		curr = curr->get_next(prev);
		prev = temp;
	}
	return 0;
}

//�ˬd�O�_��node_m (���ڥ����i�ण�O)
//�`���Ǧ^���search�U�h�����I
//���i��Ǧ^��node�O�U��i-tree
pair<node*, node*> boundary_cycle::node_m_check(int index) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp = 0;
	while (curr0->get_node()->is_visited(index)) {
		//�Y�J��visited���Di-tree, �Y�O���I.
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
	//assume �o�Onode-m
//�P���e�@��, ���O�|����init-RBC
pair<node*, node*> boundary_cycle::node_m_init_RBC(int index, boundary_cycle* p) 
{
	list_node* curr0 = _end_point[0];
	list_node* prev0 = list_node::NULL_0;
	list_node* curr1 = _end_point[1];
	list_node* prev1 = list_node::NULL_1;
	list_node* temp0 = 0;
	list_node* temp1 = 0;
	while (curr0->get_node()->is_visited(index)) {
		//�Y�J��visited���Di-tree, �Y�O���I.
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
	//�}�l�M�wnew RBC���d��
	//note: ���i��u��single child
	int e0, e1;
	list_node* n0, * n1;
	//0��V�S���Qvisited
	if (prev0 == list_node::NULL_0) {
		n0 = _end_point[0];
		e0 = (_end_point[0]->n(0) == list_node::NULL_0)? 0 : 1;
	}
	//prev0�Oi-tree, n0==curr0
	else if (prev0->get_node()->is_i_tree(index)) {
		n0 = curr0;
		e0 = (prev0 == curr0->n(0))? 0 : 1;
	}
	//prev0���Oi-tree, �hprev0�]�|�b�Qadd�i�h
	else {
		n0 = prev0;
		//temp�O�bprev0���e��
		e0 = (temp0 == prev0->n(0))? 0 : 1;
	}
	//1��V�S���Qvisited
	if (prev1 == list_node::NULL_1) {
		n1 = _end_point[1];
		e1 = (_end_point[1]->n(0) == list_node::NULL_1)? 0 : 1;
	}
	//prev1�Oi-tree, n1==curr1
	else if (prev1->get_node()->is_i_tree(index)) {
		n1 = curr1;
		e1 = (prev1 == curr1->n(0))? 0 : 1;
	}
	//prev1���Oi-tree, �hprev1�]�|�b�Qadd�i�h
	else {
		n1 = prev1;
		e1 = (temp1 == prev1->n(0))? 0 : 1;
	}
	/*
    //�p�G�u��single node, �|���@�إi��O�쥻��V�O�Ϫ�.
    //�ѩ�̫�L�@�w�|�ॿ, �ҥH�n��ʱNembedding�ॿ
	if (n0 == n1) {
		if (e1 == 0) {//�f�V
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
	//�^�Ǳ�i-tree������: 0��1
//�Y���䳣�i, �h�Ǧ^INT_MAX
int boundary_cycle::i_tree_location(int index) 
{
	//0��1�O�_��i-tree
	bool is_0_i_tree = end_point(0)->get_node()->is_i_tree(index);
	bool is_1_i_tree = end_point(1)->get_node()->is_i_tree(index);
	if (is_0_i_tree && is_1_i_tree) {
		return INT_MAX;
	}
	else if (!is_0_i_tree && !is_1_i_tree) {
		//�S��i-tree����, �]��current iteration���u�Ӧ��@��child (���D�Onode_m)
		//�{�Ӭݥ��b���ӳ���
		if (end_point(0)->get_node()->is_visited(index)) is_0_i_tree = true;
		if (end_point(1)->get_node()->is_visited(index)) is_1_i_tree = true;
	}
       //���䳣�Oi-tree, �N��c-node�bembed inside������.
	//�Ϊ̬O...�u���@��node�B�Qvisited
	if (is_0_i_tree && is_1_i_tree) return INT_MAX; 
	if (is_0_i_tree) return 0;
	if (is_1_i_tree) return 1;
	//���i�ण�|�^�ǭ�, �]���@�w���@��Qvisited.
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