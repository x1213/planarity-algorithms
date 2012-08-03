//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of miscellaneous.cpp.
//-----------------------------------------------------------------------------------

#ifndef MISCEL
#define MISCEL

#include <fstream>

enum label {
	NOT_VISITED = 0,
	ARTIFICIAL_EDGE = 1,
	BOUNDARY_PATH = 2,
	DELETED = 3
};

enum node_type {
	P_NODE = 0,
	C_NODE = 1,
	REPLICA_NODE = 2,
	AE_VIRTUAL_ROOT = 3
};

void initMark();

//0,1´«¦V
int switch_int(int);

void format_conversion(std::ifstream*, std::ofstream*);

#endif