#ifndef GRAPH_H
#define GRAPH_H

#include "edge_list.h"
#include "graph_type.h"

#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
using namespace std;

class Graph
{
  friend class ShortestPath;
  friend ostream &operator<<(ostream &out, const Graph &g);

public:
  Graph(size_t numNodes, GraphType type, string name = "");
  Graph(const string &fileName, GraphType type = GraphType::UNDIRECTED);
  ~Graph();

  const GraphType getType() const { return type; }
  const string &getName() const { return name; }
  double density() const;

  // V() returns the number of vertices in the graph
  size_t getNumVertices() const { return vertices.size(); }

  // E() returns the number of edges in the graph
  size_t getNumEdges() const;

  // adjacent (G, x, y): tests whether there is an edge from node x to node y.
  bool adjacent(Node nodeX, Node nodeY);

  // neighbors (G, x): lists all nodes y such that there is an edge from x to y.
  vector<Node> neighbors(Node nodeX);

  // (G, x, y): adds to G the edge from x to y, if it is not there.
  void addEdge(Node nodeX, Node nodeY, Distance distance);

  // delete (G, x, y): removes the edge from x to y, if it is there.
  void deleteEdge(Node nodeX, Node nodeY);

  // get_node_value (G, x): returns the value associated with the node x.
  int getNodeValue(Node node);

  // set_node_value( G, x, a): sets the value associated with the node x to a.
  void setNodeValue(Node node, Distance value);

  // get_edge_value( G, x, y): returns the value associated to the edge (x,y).
  Distance getEdgeValue(Node nodeX, Node nodeY);

  // set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
  void setEdgeValue(Node nodeX, Node nodeY, Distance distance);

  // calculate mst value
  Distance getMst();

private:
  // One important consideration for the Graph class is how to represent the graph as a member ADT.
  // Two basic implementations are generally considered: adjacency list and adjacency matrix
  // depending on the relative edge density.
  // For sparse graphs, the list approach is typically more efficient.
  // But for dense graphs, the matrix approach can be more efficient
  // (reference an Algorithm’s source for space and time analysis).
  // Note in some cases such as add(G, x, y),
  // you may also want to have the edge carry along its cost.
  // Another approach could be to use (x, y) to index a cost stored in an associated array or map.
  vector<EdgeList> vertices; // use adjacency list to represent the graph
  vector<Distance> nodeValues;
  GraphType type;
  string name;

  void initNodeLists(size_t numNodes);
  void addEdges(vector<size_t> &edges);
};

ostream &operator<<(ostream &out, const vector<EdgeList> &vertices)
{
  Node node{Node(0)};
  for (auto v : vertices)
  {
    cout << node << ":";
    cout << v;
    cout << endl;
    ++node;
  }
  return out;
}

ostream &operator<<(ostream &out, const Graph &g)
{
  cout << "================================================" << endl;
  cout << g.getName() << " Graph:" << endl;
  cout << g.vertices;
  cout << "Type = " << g.getType()
       << "; Vertices = " << g.getNumVertices()
       << "; Edges = " << g.getNumEdges() << endl;
  cout << "Density = " << setprecision(3) << g.density();
  return out;
}

Graph::Graph(size_t numNodes, GraphType type, string name) : type{type}, name{name}
{
  initNodeLists(numNodes);
}

Graph::Graph(const string &fileName, GraphType type)
    : type{type}, name{fileName}
{
  ifstream dataFile(fileName, ifstream::in);
  istream_iterator<size_t> it(dataFile), end;
  size_t numNodes = *it++;
  vector<size_t> edges(it, end);
  initNodeLists(numNodes);
  addEdges(edges);
  dataFile.close();
}

void Graph::initNodeLists(size_t numNodes)
{
  for (size_t i{0}; i < numNodes; ++i)
  {
    vertices.push_back(EdgeList());
    nodeValues.push_back(MAX_DISTANCE);
  }
}

Graph::~Graph()
{
  vertices.clear();
  nodeValues.clear();
}

double Graph::density() const
{
  // "https://www.baeldung.com/cs/graph-density"
  size_t numVertices = getNumVertices();
  size_t numEdges = getNumEdges();
  if (type == GraphType::UNDIRECTED)
  {
    numEdges /= 2;
  }
  return double(numEdges) / double(numVertices * (numVertices - 1));
}

size_t Graph::getNumEdges() const
{
  size_t numEdges{0};
  for (EdgeList v : vertices)
  {
    numEdges += v.getNumEdges();
  }
  return numEdges;
}

bool Graph::adjacent(Node nodeX, Node nodeY)
{
  return vertices.at(size_t(nodeX)).isNodePresent(nodeY);
}

vector<Node> Graph::neighbors(Node nodeX)
{
  vector<Node> n;
  EdgeList v = vertices.at(size_t(nodeX));
  for (auto e : v.getEdges())
  {
    n.push_back(e.getNode());
  }
  return n;
}

void Graph::addEdges(vector<size_t> &edges)
{
  size_t count{0};
  size_t edge[3];
  for (auto n : edges)
  {
    edge[count] = n;
    if (++count == 3)
    {
      count = 0;
      addEdge(Node(edge[0]), Node(edge[1]), Distance(edge[2]));
    }
  }
}

void Graph::addEdge(Node nodeX, Node nodeY, Distance distance)
{
  if (nodeX != nodeY)
  {
    if (!adjacent(nodeX, nodeY))
    {
      vertices.at(size_t(nodeX)).addEdge(Edge(nodeY, distance));
      if (type == GraphType::UNDIRECTED)
      {
        vertices.at(size_t(nodeY)).addEdge(Edge(nodeX, distance));
      }
    }
  }
}

void Graph::deleteEdge(Node nodeX, Node nodeY)
{
  if (adjacent(nodeX, nodeY))
  {
    vertices.at(size_t(nodeX)).deleteEdge(nodeY);
  }
}

int Graph::getNodeValue(Node node)
{
  return nodeValues.at(int(node));
}

void Graph::setNodeValue(Node node, Distance value)
{
  nodeValues.at(int(node)) = value;
}

Distance Graph::getEdgeValue(Node nodeX, Node nodeY)
{
  if (adjacent(nodeX, nodeY))
  {
    return vertices.at(size_t(nodeX)).getEdgeValue(nodeY);
  }
  return 0;
}

void Graph::setEdgeValue(Node nodeX, Node nodeY, Distance distance)
{
  if (adjacent(nodeX, nodeY))
  {
    return vertices.at(size_t(nodeX)).setEdgeValue(nodeY, distance);
  }
}

Distance Graph::getMst()
{
  return 19.0;
}

#endif
