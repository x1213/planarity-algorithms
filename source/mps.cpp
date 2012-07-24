//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include "mps.h"

//Empty constructor
maximal_planar_subgraph_finder::maximal_planar_subgraph_finder() {}

//Destructor
maximal_planar_subgraph_finder::~maximal_planar_subgraph_finder() {
	for (int i = 0; i < _node_list.size(); ++i) delete _node_list[i];
	for (int i = 0; i < _new_node_list.size(); ++i) delete _new_node_list[i];
}

mps_node* 
maximal_planar_subgraph_finder::get_new_node(node_type t) {
	_new_node_list.push_back(new mps_node(t));
	return _new_node_list[_new_node_list.size()-1];
}

//Determine the post-order-list by a DFS-traversal.
void 
maximal_planar_subgraph_finder::postOrderTraversal() {
	mps_node::init_mark();
	int postOrderID = 0;
	for (int i = 0; i < _node_list.size(); ++i) {
		if (!_node_list[i]->is_marked()) {
			_node_list[i]->DFS_visit(_post_order_list, postOrderID);
		}
	}
}

//Sort the adj-list of every node increasingly according to post-order-index.
void
maximal_planar_subgraph_finder::sort_adj_list() {
	vector<vector<mps_node*> > vecList;
	vecList.resize(_post_order_list.size());
	for (int i = 0; i < _post_order_list.size(); ++i) {
		for (int j = 0; j < _post_order_list[i]->degree(); ++j) {
			vecList[_post_order_list[i]->adj(j)->post_order_index()].push_back(_post_order_list[i]);
		}
	}
	for (int i = 0; i < _post_order_list.size(); ++i) {
		_post_order_list[i]->set_adj_list(vecList[i]);
	}
}

//Determine edge-list, and back-edge-list.
//Order the edges properly.
void 
maximal_planar_subgraph_finder::determine_edges() {
	for (int i = 0; i < _post_order_list.size(); ++i) {
		if (_post_order_list[i]->parent() == 0) continue;
		_post_order_list[i]->set_1st_label(_post_order_list[i]->parent()->post_order_index());
		_edge_list.push_back(pair<mps_node*, mps_node*> (_post_order_list[i]->parent(), _post_order_list[i]));
	}
	for (int i = 0; i < _post_order_list.size(); ++i) {
		for (int j = 0; j < _post_order_list[i]->degree(); ++j) {
			if (_post_order_list[i]->adj(j)->post_order_index() > i) break;
			if (_post_order_list[i]->adj(j)->get_1st_label() == i) continue;
			_back_edge_list.push_back(pair<mps_node*, mps_node*> (_post_order_list[i], _post_order_list[i]->adj(j)));
			_is_back_edge_eliminate.push_back(false);
		}
	}
	for (int i = 0; i < _post_order_list.size(); ++i) {
		_post_order_list[i]->set_1st_label(INT_MAX);
	}
}

//The main part of the whole algorithm: Back-edge-traversal
void 
maximal_planar_subgraph_finder::back_edge_traversal() {
	mps_node* i_node = 0;
	mps_node* current_node = 0;
	for (int i = 0; i < _back_edge_list.size(); ++i) {
		current_node = _back_edge_list[i].second;
		i_node = _back_edge_list[i].first;
		if (!back_edge_traversal(current_node, i_node->post_order_index())) _is_back_edge_eliminate[i] = true;
	}
}

//used in planarity testing
//return false if not planar.
bool maximal_planar_subgraph_finder::back_edge_traversal_without_deletion() {
	mps_node* i_node = 0;
	mps_node* current_node = 0;
	for (int i = 0; i < _back_edge_list.size(); ++i) {
		current_node = _back_edge_list[i].second;
		i_node = _back_edge_list[i].first;
		if (!back_edge_traversal(current_node, i_node->post_order_index())) return false;
	}
	return true;
}

//sub-function for the for-loop of back_edge_traversal().
bool maximal_planar_subgraph_finder::back_edge_traversal(mps_node* traverse_node, int index) {
	mps_node* parent_node; //The next node to traverse.
	//If the node has been deleted. 
	if (traverse_node == 0 || traverse_node->get_2nd_label() == DELETED) {
		return false;
	}
	//We have reached the i-node, stop.
	if (traverse_node->post_order_index() == index) {
		return true;
	}
	//Case 1
	if (traverse_node->get_2nd_label() == NOT_VISITED) {
		//1.1
		if (traverse_node->get_1st_label() == INT_MAX) {
			traverse_node->set_1st_label(index);
			parent_node = traverse_node->parent();
		}
		//1.2
		else if (traverse_node->get_1st_label() == index) {
			return true;
		}
		//1.3
		else if (traverse_node->get_1st_label() < index) {
			parent_node = construct(traverse_node);
			traverse_node->set_1st_label(index);
		}
	}
	//Case 2: Find the top-tier c-node.
	else {
		mps_node* my_c_node = find(traverse_node);
		make_essential(traverse_node, my_c_node);
		//2.1
		if (my_c_node->get_1st_label() == index) {
			parent_node = my_c_node;
		}
		//2.2
		else if (my_c_node->get_1st_label() < index) {
			mps_node* my_c_node_2 = construct(my_c_node, traverse_node);
			parent_node = my_c_node_2;
		}
		traverse_node->set_1st_label(index);
		traverse_node->set_2nd_label(NOT_VISITED);
	}
	if (back_edge_traversal(parent_node, index)) {
		if (parent_node != _post_order_list[index]) parent_node->add_child(traverse_node);
		return true;
	}
	else {
		eliminate(traverse_node);
		return false;
	}
}

//The p_node is originally a normal node in c_node's boundary cycle.
//Now we transfer it to be an essential node by the following steps:
//1. Create a replica-node of p_node to be representative of p_node in c_node.
//2. Take out the p_node from c_node, and then set the parent of p_node to be c_node.
//Note: We are not adding p_node to the c_node's children-list.
void 
maximal_planar_subgraph_finder::make_essential(mps_node* p_node, mps_node* c_node) {
	mps_node* sentinel = get_new_node(REPLICA_NODE);
	mps_node* n0 = p_node->neighbor(0);
	mps_node* n1 = p_node->neighbor(1);
	sentinel->init_replica(p_node, c_node);
	c_node->add_essential(sentinel);
	sentinel->set_to_boundary_path(n0, n1);
	sentinel->inherit_AE(p_node);
	n0->set_neighbor(n0->get_next(p_node), sentinel);
	n1->set_neighbor(n1->get_next(p_node), sentinel);
	p_node->set_neighbor((mps_node*)0, (mps_node*)0);
	p_node->set_parent(c_node);
}

//Find the top-tier c-node of the input node.
//Note: We don't set the input node to be essential node of the top-tier c-node.
//When terminated, the input node will be in the boundary cycle of top-tier c-node.
mps_node* maximal_planar_subgraph_finder::find(mps_node* n) {
	pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > boundary;
	mps_node* c_node_new = 0;
	int c_node_size = 0;
	mps_node* return_node = 0;
	if (n->parent() == 0) {
		//If n is already a node in boundary cycle.
		//Note: n must not be an essential node, otherwise it will never enter the function.
		//Find the first(nearest to n) essential node.
		boundary.first = parallel_search_sentinel(n, c_node_new);
	}
	else {
		//If n is not a node in boundary cycle.
		//It is in an Artificial edge.
		//Trim it.
		boundary = trim(n);
		//Find the first(nearest to n) essential node.
		boundary.first = parallel_search_sentinel(boundary.first.first, boundary.first.second, boundary.second.first, boundary.second.second, c_node_new);
	}
    //Find the c-node in the current hierachy .
    //If it is top-tier, return it.
	if (c_node_new != 0) return c_node_new;
	c_node_new = (boundary.first).first->get_c_node();
       
	//If not, find the two nearest essential node, eliminate the rest nodes.
	c_node_size = c_node_new->c_node_size();
	boundary.second = count_sentinel_elimination(boundary.first, c_node_size);
	//Go to the higher hierachy, and continue to find.
	if (c_node_new->get_2nd_label() == ARTIFICIAL_EDGE) {
		//A peculiar technic: 
		//Remove all the children of c_node_new but the one that should remains(Let it be u).
		//Remove all other essential nodes, pretend to be a c-node of size equals 2.
		//Call find(u).
		mps_node* u = 0;
		for (int i = 0; i < c_node_new->child_num(); ++i) {
			if (mps_node::is_same(boundary.first.first, c_node_new->child(i)) || mps_node::is_same(boundary.second.first, c_node_new->child(i))) {
				u = c_node_new->child(i);
			}
			else eliminate(c_node_new->child(i));
		}
		c_node_new->clear_children();
		c_node_new->add_child(u);
		c_node_new->clear_essential();
		c_node_new->add_essential(boundary.first.first);
		c_node_new->add_essential(boundary.second.first);
		return_node = find(u);
	}
	else return_node = find(c_node_new);
	//Merge the part of boundary cycle remains in current hierachy to the top-tier c-node.
	merge(boundary, c_node_new);
	return return_node;
}

//The list_node is a c-node. 
//The boundary indicates the part of the boundary cycle of c-node needs to be merge to the higher hierachy.
//Replace the list_node by boundary.
//Set list_node to be DELETED.
//Note: We do not eliminate anything in this function.
void 
maximal_planar_subgraph_finder::merge(pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > boundary, mps_node* list_node) {
	mps_node* n0 = list_node->neighbor(0);
	mps_node* n1 = list_node->neighbor(1);
	mps_node* s0, * s0_prev;
	mps_node* s1, * s1_prev;
	if (mps_node::is_same(boundary.first.first, n0)) {
		s0 = boundary.first.first;
		s0_prev = boundary.first.second;
		s1 = boundary.second.first;
		s1_prev = boundary.second.second;
	}
	else {
		s0 = boundary.second.first;
		s0_prev = boundary.second.second;
		s1 = boundary.first.first;
		s1_prev = boundary.first.second;
	}
	if (s0_prev == s1 && s1_prev == s0) {
	    n0->set_neighbor(n0->get_next(list_node), n1);
	    n1->set_neighbor(n1->get_next(list_node), n0);
	}
	else {
	    n0->set_neighbor(n0->get_next(list_node), s0_prev);
	    n1->set_neighbor(n1->get_next(list_node), s1_prev);
	    s0_prev->set_neighbor(s0_prev->get_next(s0), n0);
	    s1_prev->set_neighbor(s1_prev->get_next(s1), n1);
	}
	//Inherit AE.
	n0->inherit_AE(s0);
	n1->inherit_AE(s1);
	//Delete c-node
	list_node->set_2nd_label(DELETED);
}

//Set u and its subtree to be DELETED.
//If u has some AE, eliminate them.
//We don't do anything about u's parent, neighborhood.(Only children are affected.)
//If u is a p-node, we don't eliminate anything in the lower hierachy that corresponds to the same p-node.
//If u is a c-node, we eliminate all nodes in u's boundary cycle.
void 
maximal_planar_subgraph_finder::eliminate(mps_node* u) {
	if (u->get_2nd_label() == DELETED) return;
       u->set_2nd_label(DELETED);
	if (u->type() == C_NODE) {
		mps_node* list_node = u->get_a_list_node();
		mps_node* n0, * n0_prev;;
	    mps_node* temp = 0;
		n0 = list_node;
	    n0_prev = list_node->neighbor(0);
		while (true) {
			eliminate(n0);
			temp = n0;
			n0 = n0->get_next(n0_prev);
			n0_prev = temp;
			if (n0 == list_node) break;
		}
	}
	else if (u->type() == P_NODE) {
		for (int i = 0; i < u->degree(); ++i) {
			if (u->adj(i)->post_order_index() < u->post_order_index() && u->adj(i)->get_1st_label() == INT_MAX) eliminate(u->adj(i));
		}
	}
	if (u->AE(0) != 0) eliminate(u->AE(0));
	if (u->AE(1) != 0) eliminate(u->AE(1));
	for (int i = 0; i < u->child_num(); ++i) {
		eliminate(u->child(i));
	}
}

//Eliminate the AE of(u,v)-link that points to u.(If exists)
void 
maximal_planar_subgraph_finder::eliminate_AE(mps_node* u, mps_node* v) {
	int v_index = v->post_order_index();
	if (u->AE(0) != 0 && u->AE(0)->get_1st_label() == v_index) {
		eliminate (u->AE(0));
		u->set_AE(0, 0);
	}
	if (u->AE(1) != 0 && u->AE(1)->get_1st_label() == v_index) {
		eliminate (u->AE(1));
		u->set_AE(1, 0);
	}
}

//The input node u must not be c-node.
//The traversed node is in the AE = (up <- down).
//The returned boundary = [up, up_prev ..., down_prev, down].
//Direction: up it higher than down.
pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > 
maximal_planar_subgraph_finder::trim(mps_node* u) {
	mps_node* up = 0;
	mps_node* down = 0;
	//Since we may do c-node extension in the future, we need to memorize next in order to deduce prev.
	mps_node* up_next = 0;
	mps_node* down_next = 0;
	mps_node* new_AE_root = 0;
    //The index from small to large indicates the path that we traversed, note that u = node_list[0].
	vector<mps_node*> node_list;
	mps_node* curr = u;
	node_list.push_back(u);
	//Traverse upward.
	while (true) {
		curr = curr->parent();
		if (curr->type() == AE_VIRTUAL_ROOT) {
			up = curr->parent();
			//case 1: We are in a newly created c-node.
			//It has only one AE, and the two neighbor-pointer point to the same one.
			if (up->neighbor(0) == up->neighbor(1)) {
				down = up->neighbor(0);
				up->set_neighbor(down, node_list[node_list.size()-1]);
				down->set_neighbor(up, node_list[0]);
				curr->remove_child(node_list[node_list.size()-1]);
				//There's no other child, just delete the AE.
				if (curr->child_num() == 0) {
					up->set_AE(0, 0);
					up->set_AE(1, 0);
				}
			}
			//case 2: General case.
			else {
				if (up->neighbor(0)->post_order_index() == curr->get_1st_label()) down = up->neighbor(0);
				else down = up->neighbor(1);
				up->set_neighbor(up->get_next(down), node_list[node_list.size()-1]);
				down->set_neighbor(down->get_next(up), node_list[0]);
				curr->remove_child(node_list[node_list.size()-1]);
				eliminate_AE(up, down);
			}
			break;
		}
		node_list.push_back(curr);
	}
	//Set the "downward" AE of node_list[0].
	new_AE_root = get_new_node(AE_VIRTUAL_ROOT);
	new_AE_root->init_AE(node_list[0]);
	//Eliminate the children other than the path.
	for (int i = 1; i < node_list.size(); ++i) {
		for (int j = 0; j < node_list[i]->child_num(); ++j) {
			if (node_list[i]->child(j) != node_list[i-1]) eliminate(node_list[i]->child(j));
		}
	}
	//Set to the boundary path.
	if (node_list.size() == 1) node_list[0]->set_to_boundary_path(up, down);
	else {
		node_list[0]->set_to_boundary_path(down, node_list[1]);
		node_list[node_list.size()-1]->set_to_boundary_path(up, node_list[node_list.size()-2]);
	    for (int i = 1; i < node_list.size()-1; ++i) {
			node_list[i]->set_to_boundary_path(node_list[i-1], node_list[i+1]);
		}
	}
	//Set the next of up and down.
	up_next = up->get_next(node_list[node_list.size()-1]);
	down_next = down->get_next(node_list[0]);
	//Unfold the c-nodes in the node_list.
	for (int i = 0; i < node_list.size(); ++i) {
		if (node_list[i]->type() == C_NODE) c_node_extension(node_list[i]);
	}
	//Return the new boundary.
	return pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > (pair<mps_node*, mps_node*>(up, up->get_next(up_next)), pair<mps_node*, mps_node*>(down, down->get_next(down_next)));
}

//The trim's sub-function.
//The input c-node does not contain any children nor parent, but it has two neighbors, which is originally c-node's parent and one child.
//If c-node's size equals 2, then we don't need to unfold it.
//Otherwise, it must has size equals 3. 
//And then we find that redundent essential node, and remove the nodes that need not remains.
//Merge the remain part to the higher hierachy.
void 
maximal_planar_subgraph_finder::c_node_extension(mps_node* c_node) {
	//size == 2
	if (c_node->c_node_size() == 2) return;
	//size == 3
	mps_node* sentinel = 0;
	for (int i = 0; i < c_node->c_node_size(); ++i) {
		if (!mps_node::is_same(c_node->essential(i), c_node->neighbor(0)) && !mps_node::is_same(c_node->essential(i),  c_node->neighbor(1))) {
			sentinel = c_node->essential(i);
			break;
		}
	}
	eliminate(sentinel);
	//The two other essential nodes and their subsequent neighbor.
	pair<mps_node*, mps_node*> sentinel_0; 
	pair<mps_node*, mps_node*> sentinel_1;
	mps_node* n0, * n0_prev = sentinel;
	mps_node* n1, * n1_prev = sentinel;
	mps_node* temp = 0;
	n0 = sentinel->neighbor(0);
	n1 = sentinel->neighbor(1);
	while (true) {//Toward the direction of n0.
		if (n0->is_sentinel()) {//If we meet a essential node, stop, don't remove it.
			sentinel_0 = pair<mps_node*, mps_node*> (n0, n0->get_next(n0_prev));
			break;
		}
		eliminate(n0);
		temp = n0;
		n0 = n0->get_next(n0_prev);
		n0_prev = temp;
	}
	while (true) {//Toward the direction of n0.
		if (n1->is_sentinel()) {//If we meet a essential node, stop, don't remove it.
			sentinel_1 = pair<mps_node*, mps_node*> (n1, n1->get_next(n1_prev));
			break;
		}
		eliminate(n1);
		temp = n1;
		n1 = n1->get_next(n1_prev);
		n1_prev = temp;
	}

	//Remember to remove the AE toward two essential nodes that is in the delete region. 
	eliminate_AE(sentinel_0.first, sentinel_0.first->get_next(sentinel_0.second));
	eliminate_AE(sentinel_1.first, sentinel_1.first->get_next(sentinel_1.second));
	//Reset the neighborhood of two essential nodes.
	sentinel_0.first->set_neighbor(sentinel_1.first, sentinel_0.second);
	sentinel_1.first->set_neighbor(sentinel_0.first, sentinel_1.second);
	//Merge to upper boundary cycle.
	merge(pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> >(sentinel_0, sentinel_1), c_node);
}

//u is a normal p-node.
//We'll do the work of elimination, and renewing of children-list.
void 
maximal_planar_subgraph_finder::recursively_shaving(mps_node* u) { 
	mps_node* parent_node = 0;
	mps_node* node_x = 0;
	pair<mps_node*, mps_node*> new_two_child;
	vector<mps_node*> new_child_list;
	//p-node
	if (u->type() == P_NODE) {
		for (int i = 0; i < u->child_num(); ++i) recursively_shaving(u->child(i));
	}
	//c-node
	else {
		//We don't need to shave if u has only one child.
		if (u->child_num() == 1) {
			recursively_shaving(u->child(0));
			return;
		}
		//More than one child.
		parent_node = u->parent();
		//Find node_x, and shave it.
		for (int i = 0; i < u->c_node_size(); ++i) {
			if (mps_node::is_same(u->essential(i), parent_node)) {
				node_x = u->essential(i);
				new_two_child = shave(node_x);
				break;
			}
		}
		//Reset children-list and essential node.
		for (int i = 0; i < u->child_num(); ++i) {
			if (mps_node::is_same(u->child(i), new_two_child.first) || mps_node::is_same(u->child(i), new_two_child.second)) new_child_list.push_back(u->child(i));
			else eliminate(u->child(i));
		}
		u->clear_children();
		u->clear_essential();
		u->add_essential(node_x);
		u->add_essential(new_two_child.first);
		u->add_essential(new_two_child.second);
		u->add_child(new_child_list[0]);
		u->add_child(new_child_list[1]);
		for (int i = 0; i < u->child_num(); ++i) recursively_shaving(u->child(i));
	}
}

//In this function, we only deal with inner part of c-node.
//x,y,z are essential nodes, let y,z be x's nearest essential nodes in w's boundary cycle.
//Anything outside [x,y], and[x,z] will be eliminated.
//Definition of y_prev, z_prev: ..., y, y_prev, ..., x, ..., z_prev, z, ...
//Return pair = (y,z). Note: What we return is the replica-node in the inner part of c-node.
//The work of deleting children will be done by recursively_shaving().
pair<mps_node*, mps_node*> 
maximal_planar_subgraph_finder::shave(mps_node* x) {
    //c-node.
	mps_node* c_node = x->get_c_node(); 
    //No need to shave if child_num == 1.
	if (c_node->child_num() == 1) return pair<mps_node*, mps_node*>((mps_node*)0, (mps_node*)0); 
    //sentinel_1 = (y, y_prev). Note: At this time, c-node must has type equals ARTIFICIAL_EDGE, so no problem here.
	pair<mps_node*, mps_node*> sentinel_1 = parallel_search_sentinel(x, c_node);
	//sentinel_2 = (z, z_prev). Same as above.
	pair<mps_node*, mps_node*> sentinel_2 = count_sentinel_elimination(sentinel_1, c_node->child_num());
	return pair<mps_node*, mps_node*>(sentinel_1.first, sentinel_2.first);
}

//Use parallel_search to find essential nodes. Return (essential nodes that we find, its prev).
//x is not in the searching region.
//If the c-node found is top-tier, then set all the nodes during searching a pointer to c-node, set c to be that c-node, and return pair be all null.
pair<mps_node*, mps_node*> 
maximal_planar_subgraph_finder::parallel_search_sentinel(mps_node* x, mps_node* &c) {
	mps_node* n0, * n0_prev = x;
	mps_node* n1, * n1_prev = x;
	n0 = x->neighbor(0);
	n1 = x->neighbor(1);
	return parallel_search_sentinel(n0, n0_prev, n1, n1_prev, c);
}

//Another version of parallel search: n0, n0_prev, ..., n1_prev, n1
//searching region = (...n0] [n1...). Find the nearest essential node.
//return (essential nodes that we find, its prev).
pair<mps_node*, mps_node*> 
maximal_planar_subgraph_finder::parallel_search_sentinel(mps_node* n0, mps_node* n0_prev, mps_node* n1, mps_node* n1_prev, mps_node* & c) {
	mps_node* temp = 0;
	vector<mps_node*> traversed;
	while (true) {
		//If c-node is top-tier.   
        //note: If c points to a c-node traversed in some previous iteration, then it must not be top-tier, so it'll not pass the if-condition.
		if (n0->get_c_node() != 0 && n0->get_c_node()->get_2nd_label() == NOT_VISITED) {
			c = n0->get_c_node();
			break;
		}
		if (n1->get_c_node() != 0 && n1->get_c_node()->get_2nd_label() == NOT_VISITED) {
			c = n1->get_c_node();
			break;
		}
		//If an essential-node found.
		if (n0->is_sentinel()) return pair<mps_node*, mps_node*>(n0, n0_prev);
		if (n1->is_sentinel()) return pair<mps_node*, mps_node*>(n1, n1_prev);
		//Just a normal node..
		traversed.push_back(n0);
		traversed.push_back(n1);
		temp = n0;
		n0 = n0->get_next(n0_prev);
		n0_prev = temp;
		temp = n1;
		n1 = n1->get_next(n1_prev);
		n1_prev = temp;
	}

	//If the c-node found is top-tier, then assign all the traversed node a pointer to c-node.
	for (int i = 0; i < traversed.size(); ++i) traversed[i]->set_c_node(c);
	return pair<mps_node*, mps_node*>((mps_node*)0, (mps_node*)0);
}

// sentinel_1= (y, y_prev)
// return pair = (z, z_prev)
// ..., y_prev, y,[ ...(contains num_sentinel-2 essential nodes)...], z, z_prev, ... : eliminate the [...] part.
// Which means, num_sentinel = Number of essential nodes in the region [y, z].
// Note: y, z(Of course, and their prev,) will not be eliminated.
// Note: All the node that correspond to the same one as deleted node in higher hierachy will not be affected.
// The boundary cycle of c-node will be re-connected, AE be properly handled.
// Do nothing outside the c-node.
pair<mps_node*, mps_node*> maximal_planar_subgraph_finder::count_sentinel_elimination(pair<mps_node*, mps_node*> sentinel_1, int num_sentinel) {
	pair<mps_node*, mps_node*> sentinel_2; //(z, z_prev)
	int count = 1;//Count the essential nodes traversed.
	mps_node* n0 = sentinel_1.first->get_next(sentinel_1.second), * n0_prev = sentinel_1.first;//Going one step further.
	mps_node* temp = 0;
	while (true) {
		if (n0->is_sentinel()) {
			++count;//counter
			if (count == num_sentinel) {//We have reached y. Note: We will not eleminate y.
				sentinel_2.first = n0;
				sentinel_2.second = n0->get_next(n0_prev);
				break;
			}
		}
		eliminate(n0);
		temp = n0;
		n0 = n0->get_next(n0_prev);
		n0_prev = temp;
	}
	//Remember to eliminate AE toward two essential nodes that is in the deleted region.
	eliminate_AE(sentinel_2.first, sentinel_2.first->get_next(sentinel_2.second));
	eliminate_AE(sentinel_1.first, sentinel_1.first->get_next(sentinel_1.second));
	//Reset neighborhood of two essential nodes.
	sentinel_2.first->set_neighbor(sentinel_1.first, sentinel_2.second);
	sentinel_1.first->set_neighbor(sentinel_2.first, sentinel_1.second);
	return sentinel_2;
}

//Used when u has label equals <i, 0>, i<j, where j is current iteration. 
//Create a c-node with u being first essential node, and i being head. Return it.
//We'll done the parent-linke of (u -> c-node -> node_i).
//We don't create child-link here.
//Default label of newly contructed c-node is (INT_MAX, NOT_VISITED).
mps_node*
maximal_planar_subgraph_finder::construct(mps_node* u) {
	//Basic works.
	int i_label = u->get_1st_label();
	mps_node* node_i = _post_order_list[u->get_1st_label()];
	parenting_labeling_shaving(u, node_i);

	//Get some new nodes.
	mps_node* i_sentinel = get_new_node(REPLICA_NODE);
	mps_node* u_sentinel = get_new_node(REPLICA_NODE);
	mps_node* new_c_node = get_new_node(C_NODE);
	mps_node* new_AE_root = get_new_node(AE_VIRTUAL_ROOT);

	//Setting of replica-nodes.
	i_sentinel->init_replica(node_i, new_c_node);
	u_sentinel->init_replica(u, new_c_node);
	for (int i = 0; i < u->child_num(); ++i) {
		u_sentinel->add_child(u->child(i));
	}
	new_AE_root->init_AE(u_sentinel);

	//Neighborhood setting of replica-nodes in c-node.
	i_sentinel->set_to_boundary_path(u_sentinel, u_sentinel);
	u_sentinel->set_to_boundary_path(i_sentinel, i_sentinel);

	//Default label of c-node.
	new_c_node->set_1st_label(INT_MAX);
	new_c_node->set_2nd_label(NOT_VISITED);

	//Set essential node of c-node.
	new_c_node->add_essential(i_sentinel);
	new_c_node->add_essential(u_sentinel);

	//Parenting
	new_c_node->set_parent(node_i);
	u->set_parent(new_c_node);

	//Clear children-list of u_node. (which has benn transfered to AE inside c-node.)
	u->clear_children();

	return new_c_node;
	}

//The case when the first explored node is c-node (The input parameter c). 
//The p-node that trigger c(The input parameter p), has p->c parent-link already, and p is essential(not essential before triggered).
//But we don't have c->p child-link yet.
//We are not going to establish that child-link in this function. (Will be done in BET's main loop.)
//Set c to be DELETED.
mps_node* 
maximal_planar_subgraph_finder::construct(mps_node* c, mps_node* p) {
	//Basic works.
	int i_label = c->get_1st_label();
	mps_node* node_i = _post_order_list[c->get_1st_label()];
	parenting_labeling_shaving(p, node_i);
	//note: Now, c must have exactly two children left, and c has a parent-link to p, p has achild link to c, too. 
	//Remember to handle them later.

	//Get some new nodes.
	mps_node* i_sentinel = get_new_node(REPLICA_NODE);
	mps_node* new_c_node = get_new_node(C_NODE);
	i_sentinel->init_replica(node_i, new_c_node);

	//Strategy: Build thisboundary cycle first: (i, child(0), c, child(1), i).
	//And then find the two replica-node corresponding to the two children in c, and merge.
	mps_node* ch0 = c->child(0);
	mps_node* ch1 = c->child(1);
	mps_node* AE_root_0 = get_new_node(AE_VIRTUAL_ROOT);
	mps_node* AE_root_1 = get_new_node(AE_VIRTUAL_ROOT);
	AE_root_0->init_AE(ch0);
	AE_root_1->init_AE(ch1);
	i_sentinel->set_to_boundary_path(ch0, ch1);
	ch0->set_to_boundary_path(i_sentinel, c);
	ch1->set_to_boundary_path(i_sentinel, c);
	c->set_to_boundary_path(ch0, ch1);
		
	//find the boundary in c, merge!
	mps_node* sent_0;
	mps_node* sent_1;
	mps_node* sent_p;
	for (int i = 0; i < c->c_node_size(); ++i) {
		if (mps_node::is_same(c->essential(i), ch0)) sent_0 = c->essential(i);
		else if (mps_node::is_same(c->essential(i), ch1)) sent_1 = c->essential(i);
		else if (mps_node::is_same(c->essential(i), p)) sent_p = c->essential(i);
	}
	merge(pair<pair<mps_node*, mps_node*>, pair<mps_node*, mps_node*> > (pair<mps_node*, mps_node*>(sent_0, sent_0->get_next(sent_1)), pair<mps_node*, mps_node*>(sent_1, sent_1->get_next(sent_0))), c);

	//Set essential-node of c-node.
	new_c_node->add_essential(i_sentinel);
	new_c_node->add_essential(sent_p);

	//Default label of c-node.
	new_c_node->set_1st_label(INT_MAX);
	new_c_node->set_2nd_label(NOT_VISITED);

	//Parenting.
	new_c_node->set_parent(node_i);

	//p-node, p_sent.
	sent_p->set_c_node(new_c_node);
	p->clear_children();
	p->set_parent(new_c_node);

	//Delete c-node
	c->set_2nd_label(DELETED);

	return new_c_node;
}

//Some basic works in constructing c-node.
//u is the first explored node in the newly constructed c-node.
//In the case of newly constructed c-node itself is c-node, u will be the p-node that trigger the c-node.
//And in this case, p->c parent-link has been established, but c->p child-link not.
void 
maximal_planar_subgraph_finder::parenting_labeling_shaving(mps_node* u, mps_node* node_i) {
	//reverse parent-children relation in [u, node_i] as following. 
	//(u-> ... ->y->i) -> (u<- ... <-y , i).
	vector<mps_node*> u_i_path;
	u_i_path.push_back(u);
	while (true) {
		u_i_path.push_back(u_i_path[u_i_path.size()-1]->parent());
		if (u_i_path[u_i_path.size()-1] == node_i) break;
	}
	for (int i = 0; i < u_i_path.size()-2; ++i) {
		u_i_path[i]->add_child(u_i_path[i+1]);
		u_i_path[i+1]->set_parent(u_i_path[i]);
	}
	for (int i = 0; i < u_i_path.size()-2; ++i) {
		for (int j = 0; j < u_i_path[i+1]->child_num(); ++j) {
			if (u_i_path[i+1]->child(j) == u_i_path[i]) {
				u_i_path[i+1]->remove_child(j);
			}
		}
	}
	u_i_path[0]->set_parent(0);

	//BFS-traversal, all labeled to <i,1>, and then shave the c-node.
	u->recursively_labeling();
	recursively_shaving(u);
}
