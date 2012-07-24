//-----------------------------------------------------------------------------------
// Implementation of a planarity testing algorithm via PC-tree.
//-----------------------------------------------------------------------------------

#include <fstream>
#include <vector>

#include "miscellaneous.h"
using namespace std;

int __markRef = 1;
void initMark() {
	++__markRef;
}

//0,1´«¦V
int switch_int(int i) {
	if (i == 0) return 1;
	else return 0;
}

void format_conversion(ifstream* in, ofstream* out) {
	int node_num, n1, n2;
	vector<vector<int> > vec;
    //Number of nodes.
	(*in) >> node_num;
	vec.resize(node_num);
	    //Set the adj-list.
	while ((*in) >> n1 >> n2) {
	    vec[n1].push_back(n2);
	    vec[n2].push_back(n1);
	}
    (*out) << "N=" << node_num << endl;
	for (int i = 0; i < node_num; ++i) {
		(*out) << i+1 << ": ";
		for (int j = 0; j < vec[i].size(); ++j) {
			(*out) << vec[i][j]+1 << " ";
	    }
		(*out) << "0" << endl;
	}
}
