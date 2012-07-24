#include "simple_graph.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <utility>
#include <climits>

simple_graph::simple_graph() {};

void simple_graph::add_edge(int i, int j) {set_adj(i, j); set_adj(j, i);}
void simple_graph::set_adj(int i, int j) {_adj_list[i].push_back(j);}
void simple_graph::set_size(int i)
{
	_size = i;
	_adj_list.resize(i);
}

bool simple_graph::read_edge_list(ifstream* in)
{
    int node_num, n1, n2;
    //Number of nodes.
    (*in) >> node_num;
    //initialize all the nodes.
    set_size(node_num);
	_adj_list.resize(node_num);
    //Set the adj-list.
    while ((*in) >> n1 >> n2) {
	    add_edge(n1, n2);
	}
	return true;
}

void simple_graph::output_edge_list(ofstream* out)
{
	(*out) << _size << endl;
	for (int i = 0; i < _size; ++i) {
		for (int j = 0; j < _adj_list[i].size(); ++j) {
			if (_adj_list[i][j] > i) (*out) << i <<  " " << _adj_list[i][j] << endl;
		}
	}
}
bool simple_graph::read_adj_list(ifstream* in)
{
	int node_num, n1, n2;
	char dummy = 0;
	while (dummy != '=') (*in) >> dummy;
    (*in) >> node_num;
    //initialize all the nodes.
    set_size(node_num);
	//Set the adj-list.
	for (int i = 0; i < _size; ++i) {
		(*in) >> n1;
		if (i != n1-1) return false;
		dummy = 0;
		while (dummy != ':') (*in) >> dummy;
		while (true) {
			(*in) >> n2;
			if (n2 != 0) _adj_list[i].push_back(n2-1);
			else break;
		}
	}
	return true;
}
void simple_graph::output_adj_list(ofstream* out)
{
	(*out) << "N=" << _size << endl;
	for (int i = 0; i < _size; ++i) {
		(*out) << i+1 << ": ";
		for (int j = 0; j < _adj_list[i].size(); ++j) {
			(*out) << _adj_list[i][j]+1 << " "; 
		}
		(*out) << 0 << endl;
	}
}

int simple_graph::size() {return _size;}
int simple_graph::degree(int i) {return _adj_list[i].size();}
int simple_graph::adj(int i, int j) {return _adj_list[i][j];}