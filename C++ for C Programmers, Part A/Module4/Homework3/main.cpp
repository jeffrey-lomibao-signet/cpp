#include "graph.h"
#include <iostream>
using namespace std;

int main()
{
  Graph gMstExample("mst_example.txt");
  cout << gMstExample << endl;
  cout << "MST = " << gMstExample.getMst() << endl;

  Graph gMstPrim("mst_prim_example.txt");
  cout << gMstPrim << endl;
  cout << "MST = " << gMstPrim.getMst() << endl;

  Graph gMstKruskal("mst_kruskal_example.txt");
  cout << gMstKruskal << endl;
  cout << "MST = " << gMstKruskal.getMst() << endl;

  Graph gMst("mst_data.txt");
  cout << gMst << endl;
  cout << "MST = " << gMst.getMst() << endl;

  return 0;
}
