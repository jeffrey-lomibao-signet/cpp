#include <iostream>
#include <vector>
#include <cassert>

using namespace std;

//=============================================================================
enum class Node { S, A, B, C, D, E, F, G, T };
constexpr int NUM_NODES = int(Node::T) + 1; 
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
  void add(Edge e) { v.push_back(e); }
  int getNumEdges() { return v.size(); }
  bool isNodePresent(const Node n);
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

ostream& operator<<(ostream& out, const Vertex& v) {
  for (auto e: v.v) {
    cout << " " << e; 
  }
  return out;
}

//=============================================================================
class Graph {
public:
  int getNumVertices() { return g.size(); }; // V() returns the number of vertices in the graph
  int getNumEdges(); // E() returns the number of edges in the graph
  bool isNodePresent(Node node); //tests whether a node is present in the graph
  bool isEdgePresent(Node nodeX, Node nodeY);// adjacent (G, x, y): tests whether there is an edge from node x to node y.
  const Vertex& getNeighborsList(Node nodeX); // neighbors (G, x): lists all nodes y such that there is an edge from x to y.
  void add(Node nodeX, Node nodeY, int distance); // (G, x, y): adds to G the edge from x to y, if it is not there.
  void add(Node node);
// delete (G, x, y): removes the edge from x to y, if it is there.
// get_node_value (G, x): returns the value associated with the node x.
// set_node_value( G, x, a): sets the value associated with the node x to a.
// get_edge_value( G, x, y): returns the value associated to the edge (x,y).
// set_edge_value (G, x, y, v): sets the value associated to the edge (x,y) to v.
// One important consideration for the Graph class is how to represent the graph as a member ADT. Two basic implementations are generally considered: adjacency list and adjacency matrix depending on the relative edge density. For sparse graphs, the list approach is typically more efficient, but for dense graphs, the matrix approach can be more efficient (reference an Algorithm’s source for space and time analysis). Note in some cases such as add(G, x, y) you may also want to have the edge carry along its cost. Another approach could be to use (x, y) to index a cost stored in an associated array or map.
  friend ostream& operator<<(ostream& out, const Graph& g);

private:
  vector<Vertex> g;
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

void Graph::add(Node nodeX, Node nodeY, int distance) {
  Edge e{nodeY, distance};
  if (!isNodePresent(nodeX)) {
    Vertex v;
    v.add(e);
    g.push_back(v);
  }
  else if (!isEdgePresent(nodeX, nodeY)) {
    g.at(size_t(nodeX)).add(e);
  }
}

void Graph::add(Node node) {
  if(int(node) >= getNumVertices()) {
    Vertex v;
    g.push_back(v);
  }
}

const Vertex& Graph::getNeighborsList(Node nodeX) {
  assert(isNodePresent(nodeX));
  return g.at(size_t(nodeX));
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
  g.add(Node::S, Node::A, 4);
  g.add(Node::S, Node::B, 3);
  g.add(Node::S, Node::D, 7);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::A, Node::C, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::B, Node::D, 4);
  g.add(Node::B, Node::S, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::C, Node::D, 3);
  g.add(Node::C, Node::E, 1);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::D, Node::F, 5);
  g.add(Node::D, Node::T, 3);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::E, Node::G, 2);
  g.add(Node::E, Node::T, 4);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::F);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::G, Node::T, 3);
  g.add(Node::G, Node::E, 2);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;

  g.add(Node::T, Node::F, 5);
  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
}

//=============================================================================
int main() {
  Graph g;
  addExampleGraph(g);
  cout << "Graph:" << endl << g;
  return 0;
}
