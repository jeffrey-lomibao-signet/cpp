#ifndef SHORTEST_PATH_H
#define SHORTEST_PATH_H

#include "graph.h"
#include "priority_queue.h"
#include <cassert>

using namespace std;

using Path = vector<Node>;
class ShortestPath
{
  friend ostream &operator<<(ostream &out, const vector<Distance> &v);

public:
  ShortestPath(Graph &g) : g{g} {};
  ~ShortestPath() { dist.clear(); prev.clear(); shortestPath.clear(); }

  // vertices(List): list of vertices in G(V,E).
  vector<EdgeList> &getVertices();

  // path(u, w): find shortest path between u-w and
  // returns the sequence of vertices representing shortest path u-v1-v2-…-vn-w.
  const Path &path(Node u, Node w);

  // path_size(u, w): return the path cost associated with the shortest path.
  Distance pathSize(Node u, Node w);

private:
  Graph &g;
  PriorityQueue pq;
  Node start{NO_NODE}, destination{NO_NODE};
  size_t numNodes{0};
  vector<Distance> dist; // list of shortest distances to nodes
  vector<Node> prev; // list of previous nodes with shortest distance to a node
  Path shortestPath;

  void initShortestPathSearch(Node u, Node w);
  void initShortestDistanceToNodesList();
  void initPreviousNodesList();
  void updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt);
  void traverseNeighbors(Node u);
  Distance calcTotalDistanceToNeighbor(Node u, Node v);
  const Path &createShortestPathFromPrevNodesList();
};

ostream &operator<<(ostream &out, const vector<Distance> &v)
{
  cout << "[ ";
  Node n{Node(0)};
  for (auto d : v)
  {
    cout << "(" << n << ":" << setprecision(3) << d << ") ";
    ++n;
  }
  cout << "]";
  return out;
}

vector<EdgeList> &ShortestPath::getVertices()
{
  return g.vertices;
}

void ShortestPath::initShortestPathSearch(Node u, Node w)
{
  numNodes = g.getNumVertices();
  assert(numNodes > 0);
  start = u;
  destination = w;
  pq.initialize(numNodes, start);
#ifdef ENABLE_DEBUG
  cout << "pq: " << pq << endl;
#endif
  initShortestDistanceToNodesList();
  initPreviousNodesList();
}

void ShortestPath::initShortestDistanceToNodesList()
{
  dist.clear();
  for (size_t i{0}; i < numNodes; i++)
  {
    dist.push_back(MAX_DISTANCE);
  }
  dist.at(size_t(start)) = 0;
#ifdef ENABLE_DEBUG
  cout << "dist: " << dist << endl;
#endif
}

void ShortestPath::initPreviousNodesList()
{
  prev.clear();
  for (size_t i{0}; i < numNodes; ++i)
  {
    prev.push_back(NO_NODE);
  }
#ifdef ENABLE_DEBUG
  cout << "prev: " << prev << endl;
#endif
}

void ShortestPath::updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt)
{
  dist[size_t(v)] = alt;
  prev[size_t(v)] = u;
#ifdef ENABLE_DEBUG
  cout << "dist: " << dist << endl;
  cout << "prev: " << prev << endl;
#endif
}

void ShortestPath::traverseNeighbors(Node u)
{
  vector<Node> neighbors = g.neighbors(u);
#ifdef ENABLE_DEBUG
  cout << "neighbors: " << neighbors << endl;
#endif
  for (auto v : neighbors)
  {
    Distance alt = calcTotalDistanceToNeighbor(u, v);
    if (alt < dist[size_t(v)])
    {
      updateMinDistanceAndPreviousNodesLists(u, v, alt);
      pq.changePriority(v, alt);
#ifdef ENABLE_DEBUG
      cout << "pq: " << pq << endl;
#endif
    }
  }
}

Distance ShortestPath::calcTotalDistanceToNeighbor(Node u, Node v)
{
  Distance alt{dist[size_t(u)]};
  alt += g.getEdgeValue(u, v);
#ifdef ENABLE_DEBUG
  cout << "alt: " << u << "," << v << "," << alt << endl;
#endif
  return alt;
}

const Path &ShortestPath::createShortestPathFromPrevNodesList()
{
  shortestPath.clear();
  Node u{destination};
  if (prev[size_t(u)] != NO_NODE or u == start)
  {
    while (u != NO_NODE)
    {
      shortestPath.insert(shortestPath.begin(), u);
      u = prev[size_t(u)];
    }
  }
  return shortestPath;
}

const Path &ShortestPath::path(Node u, Node w)
{
  // Use Dijkstra's algorithm as described here:
  // "https://en.wikipedia.org/wiki/Dijkstra's_algorithm"
  initShortestPathSearch(u, w);
  while (!pq.isEmpty())
  {
#ifdef ENABLE_DEBUG
    cout << "===============" << endl;
#endif
    Node x{pq.minPriority()};
    if (x == destination or x == NO_NODE)
      break;
    traverseNeighbors(x);
  }
#ifdef ENABLE_DEBUG
  cout << "===============" << endl;
#endif
  return createShortestPathFromPrevNodesList();
}

Distance ShortestPath::pathSize(Node u, Node w)
{
  Path &sp{shortestPath};
  if (u != start or w != destination)
  {
    sp = path(u, w);
  }
  Distance distance{0};
  size_t numNodes{sp.size()}, i{0};
  while (numNodes-- > 1)
  {
    distance += g.getEdgeValue(sp[i], sp[i + 1]);
    ++i;
  }
  return distance;
}

#endif
