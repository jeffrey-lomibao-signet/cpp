#ifndef GRAPH_EXAMPLE_H
#define GRAPH_EXAMPLE_H

#include "graph.h"

Graph createExampleGraph() {
  constexpr int NUM_EXAMPLE_NODES = int(NodeEnum::T) + 1;
  Graph g(NUM_EXAMPLE_NODES, GraphType::DIRECTED, "Example");

  g.addEdge(NodeEnum::S, NodeEnum::A, 4);
  g.addEdge(NodeEnum::S, NodeEnum::B, 3);
  g.addEdge(NodeEnum::S, NodeEnum::D, 7);

  g.addEdge(NodeEnum::A, NodeEnum::C, 1);

  g.addEdge(NodeEnum::B, NodeEnum::D, 4);
  g.addEdge(NodeEnum::B, NodeEnum::S, 3);

  g.addEdge(NodeEnum::C, NodeEnum::D, 3);
  g.addEdge(NodeEnum::C, NodeEnum::E, 1);

  g.addEdge(NodeEnum::D, NodeEnum::F, 5);
  g.addEdge(NodeEnum::D, NodeEnum::T, 3);

  g.addEdge(NodeEnum::E, NodeEnum::G, 2);
  g.addEdge(NodeEnum::E, NodeEnum::T, 4);

  g.addEdge(NodeEnum::G, NodeEnum::T, 3);
  g.addEdge(NodeEnum::G, NodeEnum::E, 2);

  g.addEdge(NodeEnum::T, NodeEnum::F, 5);

  cout << g;
  return g;
}

#endif
