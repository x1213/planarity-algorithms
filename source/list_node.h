//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of list_node.cpp.
//-----------------------------------------------------------------------------------

#ifndef LIST_NODE
#define LIST_NODE

class node;
class boundary_cycle;

class list_node
{
public:
	list_node(node* n);
	~list_node();
	list_node* n(int i);
	list_node* get_next(list_node* n);
	void contract();
	node* get_node();
	boundary_cycle* get_RBC();
	void set_RBC(boundary_cycle* c);
	void set_neighborhood(list_node* n0, list_node* n1);
	void set_neighbor(int i, list_node* u);
	bool is_end();
	static bool link(list_node* n1, list_node* n2);
	static list_node* list_node::NULL_0;
	static list_node* list_node::NULL_1;
private:
	node* _n; //對應原本的node
	boundary_cycle* _RBC;
	list_node* _neighbor[2]; //無方向性
};


#endif