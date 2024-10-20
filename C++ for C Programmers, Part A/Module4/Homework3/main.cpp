#include "graph.h"
#include <iostream>
using namespace std;

void mstTest(string graphFileName)
{
  Graph graph(graphFileName);
  cout << graph << endl;
  mst_pair p = graph.getMst();
  cout << "MST cost = " << p.first << ", tree = " << p.second << endl;
}

int main()
{
  mstTest("mst_example.txt");
  mstTest("mst_prim_example.txt");
  mstTest("mst_kruskal_example.txt");
  mstTest("mst_data.txt");

  return 0;
}
