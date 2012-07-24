//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#ifndef NODE
#define NODE

#include <utility>
#include <vector>

#include "list_node.h"
#include "embedding.h"
#include "miscellaneous.h"

using namespace::std;

class list_node;
class boundary_cycle;



class node
{
public:
	node(int i, node_type t = P_NODE);
	~node();
	void init_c_node(node* head, node* node_t, boundary_cycle* b);

	node_type type();
	int index();
	bool is_finished();
	void finish();
	void DFS_visit(vector<node*> &dfsList, int &index);
	void setMaxBackAnscenstorID();

	int maxBackEdgeID();

	int maxBackAnscenstorID();
	bool is_articulation_point();
	void reset_dfs_info();

	void split_arculation_point(vector<node*> &node_list, node* modified = 0);
	void add_adj(node* u);
	void set_parent(node* u);
	void setAdjList(vector<node*> &adjList);
	int postOrderIndex();
	int degree();
	node* adj(int i);
	bool is_root();
	void mark();
	bool isMarked();
	int num_curr_descendant();
	node* curr_descendant(int i);
	int num_descendant();
	node* descendant(int i);
	int num_remain_descendant();
	node* remain_descendant(int i);

	void get_remain_list(vector<node*> &list);
	void get_current_list(vector<node*> &list);
	void get_total_list(vector<node*> &list);

	node* parent();
	bool is_visited(int i);
	void visit(int i);

	node* segment_one_check(int index);
	node* segment_two_check(int index, bool &is_terminated);
	pair<node*, node*> node_m_check(int index, bool &is_valid);

	void add_node(node* u, int index);
	void clear_current_descendant();

	list_node* get_list_node();
	void set_list_node(list_node* n);
	boundary_cycle* get_RBC();

	void get_curr_descendantList(vector<node*> &i_tree, int index, node* &next);
	void get_curr_descendantList(vector<node*> &i_tree, int index);

	embedding* get_embedding();

	bool is_i_tree(int index);


	void add_to_final_embedding(vector<int> &emb_list);
	void add_to_remain_list(node* u);
	void init_remain_list();
	bool is_in_c_node();

	void output_embedding(bool is_flip);
private:
	node_type _type;
	int _Mark;
	int _index;
	int _visited; //_visited = i 表示上次被traverse是在i-iteration
	int _post_order_index;
	int _maxBackAnscenstorID;
	bool _is_done;
	node* _parent;
	vector<node*> _adjList;
	vector<node*> _descendantList;
	vector<node*> _curr_descendantList;
	vector<node*> _remain_descendantList; //剩下的descendant-list
	list_node* _list_node; //對應在c-node中的list-node
	boundary_cycle* _RBC; //如果此node是c-node的話
	embedding* _emb;
	pair<int, bool> _i_tree_test; //為了加速is_i_tree()用的, 儲存記錄.
};



#endif