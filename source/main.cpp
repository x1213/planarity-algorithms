//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <climits>
#include <ctime>
#include "mps.h"
#include "simple_graph.h"
#include "planarity_test.h"

class simple_graph;

using namespace std;

bool planarity_testing(simple_graph*, simple_graph*, simple_graph*);
void find_mps(simple_graph*, simple_graph*);

simple_graph __in_g;
simple_graph __out_g;
simple_graph __obs_g;
simple_graph __gen_g;

//n nodes, with the probability of existence of each edge being p.
void random_graph_generator(int n, double p, simple_graph* g) {
	g->set_size(n);
	for (int i = 0; i < n; ++i) {
		for (int j = i+1; j < n; ++j) {
			if ((double)rand()/RAND_MAX <= p) g->add_edge(i, j);
		}
	}
}

//Complete graph K_n.
void complete_graph_generator(int n, simple_graph* g) {
	g->set_size(n);
	for (int i = 0; i < n; ++i) {
		for (int j = i+1; j < n; ++j) {
			g->add_edge(i, j);
		}
	}
}

void planarityTest(ifstream* in, ofstream* out) {
	__in_g.read_adj_list(in);
	planarity_test* p = new planarity_test();
	if (p->planarity_testing(&__in_g, &__out_g, &__obs_g)) __out_g.output_adj_list(out);
	else __obs_g.output_adj_list(out);
	delete p;
}

//-----------------------------------------------------------------------------------
// Main function.
//-----------------------------------------------------------------------------------

/*
int main() {
		ifstream in;
	    in.open("Data/in.txt");
	    ofstream out;
	    out.open("Data/out.txt");
		
		//random_graph_generator(400, 0.1, &__gen_g);
		//__gen_g.output_adj_list(&out);
		
		//
		//__in_g.read_adj_list(&in);
		//find_mps(&__in_g, &__out_g);
		planarityTest(&in, &out);
		//__out_g.output_adj_list(&out);
		//
		return 0;
}
*/

int main(int argc, char* argv[]) {
	srand(time(0));

	if (argc == 1) {
		cout << "Usages:" << endl;
		cout << "======================" << endl;
		cout << "planarity -mps <infile> <outfile>" << endl;
		cout << "  Process the infile, and output the resulting maximal" << endl
			 << "  planar subgraph in the outfile." << endl;
		cout << "planarity -pt <infile> <embedding> <obstruction>" << endl;
		cout << "  Process the infile, and output the planar embedding" << endl
			 << "  if the input graph is planar, otherwise a Kuratowski"<< endl
			 << "  subgraph is isolated." << endl;
		cout << "planarity -gen <infile> <outfile>" << endl;
		cout << "  Generate a random graph as the spec given in the infile." << endl;
		while (true) {
			int x = 0;
			cin >> x;
		}
	}
	if (argv[1][1] == 'm') {
	    ifstream in;
	    in.open(argv[2]);
	    ofstream out;
	    out.open(argv[3]);
		__in_g.read_adj_list(&in);
		find_mps(&__in_g, &__out_g);
		__out_g.output_adj_list(&out);
	}
	else if (argv[1][1] == 'g') {
	    ifstream in;
	    in.open(argv[2]);
	    ofstream out;
	    out.open(argv[3]);
		int n;
		double p;
		in >> n;
		in >> p;
		random_graph_generator(n, p, &__gen_g);
		__gen_g.output_adj_list(&out);
	}
	else if (argv[1][1] == 'p') {
	    ifstream in;
		ofstream out;
	    in.open(argv[2]);
		__in_g.read_adj_list(&in);
		if (planarity_testing(&__in_g, &__out_g, &__obs_g)) {
			out.open(argv[3]);
			__out_g.output_adj_list(&out);
		}
		else {
			out.open(argv[4]);
			__obs_g.output_adj_list(&out);
		}
	}
	return 0;
}

