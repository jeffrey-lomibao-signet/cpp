#ifndef EDGE_H
#define EDGE_H

#include "node.h"
#include "distance.h"
#include <iomanip>

using namespace std;

class Edge
{
  friend class EdgeList;
  friend ostream &operator<<(ostream &out, const Edge &e);

public:
  Edge(Node n, Distance d) : node{n} { setDistance(d); }
  Node getNode() { return node; }
  Distance getDistance() { return distance; }
  void setNode(Node n) { node = n; }
  void setDistance(Distance d) { distance = d; }
  void setEdge(Node n, Distance d)
  {
    setNode(n);
    setDistance(d);
  }
  Edge getEdge() { return Edge(node, distance); }

private:
  Node node;
  Distance distance;
};

ostream &operator<<(ostream &out, const Edge &e)
{
  out << "(" << e.node << "," << setprecision(3) << e.distance << ")";
  return out;
}

#endif
