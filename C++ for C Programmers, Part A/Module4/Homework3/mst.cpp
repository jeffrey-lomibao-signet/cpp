#include "graph.h"
#include <iostream>
using namespace std;

int main()
{
  Graph gMst("mst_data.txt");
  cout << gMst << endl;

  Graph gMstExample("mst_example.txt");
  cout << gMstExample << endl;

  Graph gMstPrim("mst_prim_example.txt");
  cout << gMstPrim << endl;

  Graph gMstKruskal("mst_kruskal_example.txt");
  cout << gMstKruskal << endl;

  return 0;
}
