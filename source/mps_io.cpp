//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "mps.h"
#include "simple_graph.h"

//-----------------------------------------------------------------------------------
// Find MPS
//-----------------------------------------------------------------------------------

void find_mps(simple_graph* in, simple_graph* out) {
	maximal_planar_subgraph_finder m;
	m.find_mps(in, out);
}

bool maximal_planar_subgraph_finder::planarity_test(simple_graph* in) {
	read_from_graph(in);
	postOrderTraversal();
	sort_adj_list();
	determine_edges();
	return back_edge_traversal_without_deletion();
}

void maximal_planar_subgraph_finder::find_mps(simple_graph* in, simple_graph* out) {
	read_from_graph(in);
	postOrderTraversal();
	sort_adj_list();
	determine_edges();
	back_edge_traversal();
	output_to_graph(out);
}

//-----------------------------------------------------------------------------------
// Graph IO
//-----------------------------------------------------------------------------------

void maximal_planar_subgraph_finder::read_from_graph(simple_graph* g)
{
	for (int i = 0; i < g->size(); ++i) {
		_node_list.push_back(new mps_node(P_NODE));
	}
	for (int i = 0; i < g->size(); ++i) {
		_node_list[i]->set_id(i);
		for (int j = 0; j < g->degree(i); ++j) {
			_node_list[i]->add_adj(_node_list[g->adj(i, j)]);
		}
	}
}
void maximal_planar_subgraph_finder::output_to_graph(simple_graph* g)
{
	g->set_size(_node_list.size());
	for (int i = 0; i < _edge_list.size(); ++i) {
		g->add_edge(_edge_list[i].first->node_id(), _edge_list[i].second->node_id());
	}
	for (int i = 0; i < _back_edge_list.size(); ++i) {
		if (_is_back_edge_eliminate[i]) continue;
		g->add_edge(_back_edge_list[i].first->node_id(), _back_edge_list[i].second->node_id());
	}
}