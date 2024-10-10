#ifndef GRAPH_TYPE_H
#define GRAPH_TYPE_H

#include <iostream>
using namespace std;

enum class GraphType
{
  DIRECTED,
  UNDIRECTED
};

ostream &operator<<(ostream &out, const GraphType &type)
{
  switch (type)
  {
  case GraphType::DIRECTED:
    out << "Directed";
    break;
  case GraphType::UNDIRECTED:
    out << "Undirected";
    break;
  }
  return out;
}

#endif
