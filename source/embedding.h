//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of embedding.cpp.
//-----------------------------------------------------------------------------------

#ifndef EMBEDDING
#define EMBEDDING

#include "list_node.h"

class node;
class list_node;
class boundary_cycle;


class embedding
{
public:
	embedding(node* u);
	~embedding();
	void clear();
	void add(node* u, int dir, boundary_cycle* b);
	void add(embedding* u, int dir, bool flip, boundary_cycle* b);
	int size();
	void set_flip(bool f);
	bool is_flip();
	list_node* end(int i);
	void flip_end_point();
	boundary_cycle* get_RBC();
	void set_RBC(boundary_cycle* b);

private:
	node* _n; //對應原本的node
	boundary_cycle* _RBC; //上次update時是在哪個RBC?
	list_node* _end_point[2]; //0, 1表開始與結束
	int _size; //embedding的大小, 如果等同adj-list大小的話, 表示結束了, 可以contract掉.
	bool _flip; //需不需要flip
	//0, 1是遵照list-node之neighbor:0,1方向而訂 <-無方向性
	//所以需要一個flip記是否要flip過才能轉到正確方向
};



#endif