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


//-----------------------------------------------------------------------------------
// planarity testing
//-----------------------------------------------------------------------------------

bool planarity_testing(simple_graph* in, simple_graph* emb, simple_graph* obs)
{
	planarity_test pt;
	return pt.planarity_testing(in, emb, obs);
}

bool planarity_test::planarity_testing(simple_graph* in, simple_graph* emb, simple_graph* obs)
{
	read_from_graph(in);
	determine_biconnected_component();
	if (back_edge_traversal()) {
		c_node_flip();
		recover_embedding();
		output_embedding(emb);
		return true;
	}
	else {
		radix_sort_obstruction_list();
		output_obstruction(obs);
		return false;
	}
}

//-----------------------------------------------------------------------------------
// Graph IO
//-----------------------------------------------------------------------------------

void planarity_test::read_from_graph(simple_graph* g)
{
    //initialize all the nodes.
    for (int i = 0; i < g->size(); ++i) {
		_node_list.push_back(i);
	    _dfs_list.push_back(new node(i));
    }
	_adj_list.resize(g->size());
    //Set the adj-list.
    for (int i = 0; i < g->size(); ++i) {
		for (int j = 0; j < g->degree(i); ++j) {
	        _adj_list[i].push_back(g->adj(i, j));
		    _dfs_list[i]->add_adj(_dfs_list[g->adj(i, j)]);
		}
	}
}
void planarity_test::output_embedding(simple_graph* g)
{
	g->set_size(_embed_list.size());
	for (int i = 0; i < _embed_list.size(); ++i) {
		for (int j = 0; j < _embed_list[i].size(); ++j) {
			g->set_adj(i, _embed_list[i][j]);
		}
	}
}
void planarity_test::output_obstruction(simple_graph* g)
{
	g->set_size(_node_list.size());
	for (int i = 0; i < _obstruction.size(); ++i) {
		g->add_edge(_obstruction[i].first, _obstruction[i].second);
	}
}
