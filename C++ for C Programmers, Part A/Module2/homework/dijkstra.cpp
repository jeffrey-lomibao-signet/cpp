#include <iostream>
#include <vector>
#include <cassert>

using namespace std;

//=============================================================================
enum class Node { S, A, B, C, D, E, F, G, T };
constexpr int NUM_NODES = int(Node::T) + 1; 
constexpr int INVALID_NODE = -1;

ostream& operator<<(ostream& out, const Node& n) {
  switch(n) {
  case Node::S: out << "S"; break;
  case Node::A: out << "A"; break;
  case Node::B: out << "B"; break;
  case Node::C: out << "C"; break;
  case Node::D: out << "D"; break;
  case Node::E: out << "E"; break;
  case Node::F: out << "F"; break;
  case Node::G: out << "G"; break;
  case Node::T: out << "T"; break;
  }
  return out;
}

Node operator++(Node& n) { 
  n = static_cast<Node>((static_cast<int>(n) + 1) % NUM_NODES);
  return n; 
}
Node operator++(Node& n, int) {
  Node temp = n;
  n = Node((int(n) + 1) % NUM_NODES);
  return temp;
}

//=============================================================================
class Edge {
public:
  Edge(Node n, int d = 0)
    : node{n}, distance{d} {}

  Node getNode() { return node; }
  int getDistance() { return distance; }
  void setNode(Node n) { node = n; }
  void setDistance(int d) { distance = d; }
  void setEdge(Node n, int d) { setNode(n); setDistance(d); }
  Edge getEdge() { return Edge(node, distance); }

  friend ostream& operator<<(ostream& out, const Edge& e) {
    out << "(" << e.node << "," << e.distance << ")";
    return out;
  }
  Node node;
  int distance; 
};

//=============================================================================
class Vertex {
public:
  Vertex() {};
  void addEdge(Edge e) { v.push_back(e); }
  void deleteEdge(Node node);
  int getNumEdges() { return v.size(); }
  bool isNodePresent(const Node node);
  int getEdgeValue(const Node node);
  void setEdgeValue(const Node node, int distance);
  friend ostream& operator<<(ostream& out, const Vertex& v);

private:
  vector<Edge> v;
};

bool Vertex::isNodePresent(Node node) {
  bool edgeFound{false};
  for (auto e: v) {
    if (e.getNode() == node) {
      edgeFound = true;
      break;
    }
  }
  return edgeFound;
}

void Vertex::deleteEdge(Node node) {
  for (auto e = v.begin(); e != v.end(); ++e) {
    if (e->getNode() == node) {
      v.erase(e);
      break;
    }
  }
}

int Vertex::getEdgeValue(const Node node) {
  for (auto e = v.begin(); e != v.end(); ++e) {
    if (e->getNode() == node) {
      return e->getDistance();
    }
  }
  return INVALID_NODE;
}

void Vertex::setEdgeValue(const Node node, int distance) {
  for (auto e = v.begin(); e != v.end(); ++e) {
    if (e->getNode() == node) {
      return e->setDistance(distance);
    }
  }
}

ostream& operator<<(ostream& out, const Vertex& v) {
  for (auto e: v.v) {
    cout << " " << e; 
  }
  return out;
}

//=============================================================================
class Graph {
public:
  int getNumVertices() { return g.size(); } // V() returns the number of vertices in the graph
  int getNumEdges(); // E() returns the number of edges in the graph
  bool isNodePresent(Node node); //tests whether a node is present in the graph
  bool isEdgePresent(Node nodeX, Node nodeY);// adjacent (G, x, y): tests whether there is an edge from node x to node y.
  const Vertex& getNeighborsList(Node nodeX); // neighbors (G, x): lists all nodes y such that there is an edge from x to y.
  void addNode(Node node);
  void addEdge(Node nodeX, Node nodeY, int distance); // (G, x, y): adds to G the edge from x to y, if it is not there.
  void deleteEdge(Node nodeX, Node nodeY); // delete (G, x, y): removes the edge from x to y, if it is there.
  int getNodeValue(Node node); // get_node_value (G, x): returns the value associated with the node x.
  // setNodeValue is not needed since we are using the vertex index as the node value
  // set_node_value( G, x, a): sets the value associated with the node x to a.
  int getEdgeValue(Node nodeX, Node nodeY); // get_edge_value( G, x, y): returns the value associated to the edge (x,y).
  void setEdgeValue(Node nodeX, Node nodeY, int distance);// set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.

  friend ostream& operator<<(ostream& out, const Graph& g);

private:
  // One important consideration for the Graph class is how to represent the graph as a member ADT. 
  // Two basic implementations are generally considered: adjacency list and adjacency matrix depending on the relative edge density. 
  // For sparse graphs, the list approach is typically more efficient, but for dense graphs, the matrix approach can be more efficient 
  // (reference an Algorithm’s source for space and time analysis). 
  // Note in some cases such as add(G, x, y) you may also want to have the edge carry along its cost. 
  // Another approach could be to use (x, y) to index a cost stored in an associated array or map.
  vector<Vertex> g; // use adjacency list to represent the graph
};

int Graph::getNumEdges() {
  int numEdges{0};
  for(Vertex v: g) {
    numEdges += v.getNumEdges();
  }
  return numEdges;
}

bool Graph::isNodePresent(Node node) {
  return int(node) < getNumVertices();
}

bool Graph::isEdgePresent(Node nodeX, Node nodeY) {
  bool edgeFound{false};
  if(isNodePresent(nodeX)) {
    edgeFound = g.at(size_t(nodeX)).isNodePresent(nodeY);
  }
  return edgeFound;
}

const Vertex& Graph::getNeighborsList(Node nodeX) {
  assert(isNodePresent(nodeX));
  return g.at(size_t(nodeX));
}

void Graph::addNode(Node node) {
  if(!isNodePresent(node)) {
    Vertex v;
    g.push_back(v);
  }
}

void Graph::addEdge(Node nodeX, Node nodeY, int distance) {
  Edge e{nodeY, distance};
  if (!isNodePresent(nodeX)) {
    Vertex v;
    v.addEdge(e);
    g.push_back(v);
  }
  else if (!isEdgePresent(nodeX, nodeY)) {
    g.at(size_t(nodeX)).addEdge(e);
  }
}

void Graph::deleteEdge(Node nodeX, Node nodeY) {
  if (isEdgePresent(nodeX, nodeY)) {
    g.at(size_t(nodeX)).deleteEdge(nodeY);
  }
}

int Graph::getNodeValue(Node node) {
  if (isNodePresent(node))
    return int(node); 
  return INVALID_NODE;
}

int Graph::getEdgeValue(Node nodeX, Node nodeY) {
  if (isEdgePresent(nodeX, nodeY)) {
    return g.at(size_t(nodeX)).getEdgeValue(nodeY);
  }
  return INVALID_NODE;
}

void Graph::setEdgeValue(Node nodeX, Node nodeY, int distance) {
  if (isEdgePresent(nodeX, nodeY)) {
    return g.at(size_t(nodeX)).setEdgeValue(nodeY, distance);
  }
}

ostream& operator<<(ostream& out, const Graph& g) {
  Node node{Node(0)};
  for (auto v: g.g) {
    cout << node << ":";
    cout << v;
    cout << endl;
    ++node;
  }
  return out;
}


//=============================================================================
void addExampleGraph(Graph& g) {
  g.addEdge(Node::S, Node::A, 4);
  g.addEdge(Node::S, Node::B, 3);
  g.addEdge(Node::S, Node::D, 7);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::A, Node::C, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::B, Node::D, 4);
  g.addEdge(Node::B, Node::S, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::C, Node::D, 3);
  g.addEdge(Node::C, Node::E, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::D, Node::F, 5);
  g.addEdge(Node::D, Node::T, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::E, Node::G, 2);
  g.addEdge(Node::E, Node::T, 4);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addNode(Node::F);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::G, Node::T, 3);
  g.addEdge(Node::G, Node::E, 2);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.addEdge(Node::T, Node::F, 5);
  g.addEdge(Node::S, Node::T, 20);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  cout << "Graph:" << endl << g;

  g.deleteEdge(Node::S, Node::T);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  cout << "Graph:" << endl << g;

  cout << "S->A = " << g.getEdgeValue(Node::S, Node::A) << endl;
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;
  g.setEdgeValue(Node::G, Node::T, 5);
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;
}

//=============================================================================
int main() {
  Graph g;
  addExampleGraph(g);
  return 0;
}
