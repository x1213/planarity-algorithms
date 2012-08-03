//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of boundar_cycle.cpp.
//-----------------------------------------------------------------------------------

#ifndef BOUNDARY_CYCLE
#define BOUNDARY_CYCLE

#include "list_node.h"
#include "embedding.h"
#include "node.h"
#include "miscellaneous.h"

class node;
class list_node;

class boundary_cycle
{
public:
	boundary_cycle(node* c, node* i, node* t);
	~boundary_cycle();

	void debug_rbc();

	void set_embedding(embedding* emb);
	void contract_list_node();
	void recursively_flip(bool is_flip);

	void init_RBC(node* u, int dir1 = 1);
	void init_RBC(list_node* n0, list_node* n1, int end_dir0, int end_dir1);

	void add_p_node(int dir, node* add);
	void add_list(boundary_cycle* b, list_node* end_node, int dir, int start_dir, int end_dir);

	node* segment_one_check(int index);
	node* segment_two_check(int index, bool &is_terminated);
	pair<node*, node*> node_m_check(int index);
	pair<node*, node*> node_m_init_RBC(int index, boundary_cycle* p);

	node* head();
	node* c_node();
	bool is_flip();
	embedding* get_embedding();
	list_node* end_point(int i);
	void add_to_containment_list(boundary_cycle* b, bool f);
	int i_tree_location(int index);
	node* find_unvisited_child(int index);
	void mark();
	bool is_mark();
	node* get_node_t();
private:
	list_node* _end_point[2]; //0, 1分別表示左右
	node* _head; //c-node的parent
	node* _node_t; //是從_head的哪個child開始的
	node* _c; //c-node自己
	vector<pair<boundary_cycle*, bool> > _containment_list; //bool代表有無flip
	vector<list_node*> _possible_contract_list; //基本上就是boundary path上在此iteration中有做embedding的nodes
	embedding* _emb;
	bool _is_flip;
	int _mark;
};


#endif