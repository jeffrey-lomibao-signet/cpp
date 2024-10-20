#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <vector>

using namespace std;

enum class Node: int { 
  A, B, C, D, E, F, G, H, I
};
constexpr Node NO_NODE = Node(-1);
using NodeEnum = Node;

ostream& operator<<(ostream& out, const Node& n) {
  int i = int(n);
  if(i >= 0 and i < 26)
    out << char('A' + i);
  else
    out << i;

  return out;
}

Node operator++(Node& n) { 
  n = static_cast<Node>((static_cast<int>(n) + 1));
  return n; 
}
Node operator++(Node& n, int) {
  Node temp = n;
  n = Node(int(n) + 1);
  return temp;
}

ostream &operator<<(ostream &out, const vector<Node> &nodes)
{
  cout << "[";
  for (auto n = nodes.begin(); n != nodes.end(); ++n)
  {
    cout << *n;
    if ((nodes.size() > 1) && (n != (nodes.end() - 1)))
    {
      cout << ",";
    }
  }
  cout << "]";
  return out;
}

#endif
