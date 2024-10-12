#ifndef GRAPH_WIKIPEDIA_H
#define GRAPH_WIKIPEDIA_H

#include "graph.h"

Graph createWikipediaGraph() {
  constexpr int NUM_WIKI_NODES = int(NodeEnum::F) + 1; 
  Graph g(NUM_WIKI_NODES, GraphType::UNDIRECTED, "Wikipedia");

  g.addEdge(NodeEnum::A, NodeEnum::B, 7);
  g.addEdge(NodeEnum::A, NodeEnum::C, 9);
  g.addEdge(NodeEnum::A, NodeEnum::F, 14);

  g.addEdge(NodeEnum::B, NodeEnum::C, 10);
  g.addEdge(NodeEnum::B, NodeEnum::D, 15);
  
  g.addEdge(NodeEnum::C, NodeEnum::D, 11);
  g.addEdge(NodeEnum::C, NodeEnum::F, 2);

  g.addEdge(NodeEnum::D, NodeEnum::E, 6);
  g.addEdge(NodeEnum::E, NodeEnum::F, 9);
  g.addEdge(NodeEnum::F, NodeEnum::E, 9);

  cout << g;
  return g;
}

#endif
