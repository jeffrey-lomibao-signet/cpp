#ifndef GRAPH_RANDOM_H
#define GRAPH_RANDOM_H

#include "graph.h"
#include <random>
#include <ctime>
using namespace std;

Graph createRandomGraph(size_t numNodes, double density, Distance min, Distance max)
{
  default_random_engine e(time(0));
  uniform_real_distribution<Distance> randomDistance(min, max);
  uniform_int_distribution<int> randomNode(0, numNodes - 1);
  Graph g(numNodes, GraphType::UNDIRECTED, "Random");
  g.addEdge(Node(0), Node(randomNode(e)), randomDistance(e));
  while (g.density() < density)
  {
    g.addEdge(Node(randomNode(e)), Node(randomNode(e)), randomDistance(e));
  }
  cout << g;
  return g;
}

#endif
