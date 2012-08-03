//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
// The header of simple_graph.cpp.
//-----------------------------------------------------------------------------------

#ifndef _SIMP_GRAPH_H
#define _SIMP_GRAPH_H

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <climits>

#include "miscellaneous.h"
#include "simple_graph.h"

using namespace::std;

//basic data structure for graph that deal with every algorithm in the program.
class simple_graph
{
public:
	simple_graph() {};
	~simple_graph() {};
	void add_edge(int i, int j);
	void set_adj(int i, int j);
	void set_size(int i);
	bool read_edge_list(ifstream* in);
	void output_edge_list(ofstream* out);
	bool read_adj_list(ifstream* in);
	void output_adj_list(ofstream* out);

	int size();
	int degree(int i);
	int adj(int i, int j);
private:
	int _size;
	vector<vector<int> > _adj_list;
};

#endif