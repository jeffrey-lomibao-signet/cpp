#include "graph.h"
#include <iostream>
using namespace std;

void mstTest(string graphFileName)
{
  Graph graph(graphFileName);
  cout << graph << endl;
  cout << "MST = " << graph.getMst() << endl;
}

int main()
{
  mstTest("mst_example.txt");
  mstTest("mst_prim_example.txt");
  mstTest("mst_kruskal_example.txt");
  mstTest("mst_data.txt");

  return 0;
}
