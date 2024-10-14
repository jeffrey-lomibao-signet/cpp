#include "graph.h"
#include <iostream>
using namespace std;

int main()
{
  Graph gMst("mst_data.txt");
  cout << gMst << endl;

  return 0;
}
