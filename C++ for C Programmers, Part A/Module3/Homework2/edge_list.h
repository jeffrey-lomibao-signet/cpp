#ifndef EDGE_LIST_H
#define EDGE_LIST_H
#include "edge.h"

class EdgeList
{
public:
  vector<Edge> &getEdges() { return edges; };
  void addEdge(Edge e) { edges.push_back(e); }
  void deleteEdge(Node node);
  size_t getNumEdges() { return edges.size(); }
  bool isNodePresent(const Node node);
  Distance getEdgeValue(const Node node);
  void setEdgeValue(const Node node, Distance distance);
  friend ostream &operator<<(ostream &out, const EdgeList &v);

private:
  vector<Edge> edges;
};

ostream &operator<<(ostream &out, const EdgeList &v)
{
  for (auto e : v.edges)
  {
    cout << " " << e;
  }
  return out;
}

bool EdgeList::isNodePresent(Node node)
{
  bool edgeFound{false};
  for (auto e : edges)
  {
    if (e.getNode() == node)
    {
      edgeFound = true;
      break;
    }
  }
  return edgeFound;
}

void EdgeList::deleteEdge(Node node)
{
  for (auto e = edges.begin(); e != edges.end(); ++e)
  {
    if (e->getNode() == node)
    {
      edges.erase(e);
      break;
    }
  }
}

Distance EdgeList::getEdgeValue(const Node node)
{
  for (auto e = edges.begin(); e != edges.end(); ++e)
  {
    if (e->getNode() == node)
    {
      return e->getDistance();
    }
  }
  return 0;
}

void EdgeList::setEdgeValue(const Node node, Distance distance)
{
  for (auto e = edges.begin(); e != edges.end(); ++e)
  {
    if (e->getNode() == node)
    {
      return e->setDistance(distance);
    }
  }
}

#endif
