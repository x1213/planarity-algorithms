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
	node* _n; //�����쥻��node
	boundary_cycle* _RBC; //�W��update�ɬO�b����RBC?
	list_node* _end_point[2]; //0, 1��}�l�P����
	int _size; //embedding���j�p, �p�G���Padj-list�j�p����, ��ܵ����F, �i�Hcontract��.
	bool _flip; //�ݤ��ݭnflip
	//0, 1�O���list-node��neighbor:0,1��V�ӭq <-�L��V��
	//�ҥH�ݭn�@��flip�O�O�_�nflip�L�~����쥿�T��V
};



#endif