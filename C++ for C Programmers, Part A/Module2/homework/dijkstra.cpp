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
  default: out << int(n);
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

ostream& operator<<(ostream& out, const vector<Node>& nodes) {
  cout << "[";
  for (auto n = nodes.begin(); n != nodes.end(); ++n) {
      cout << *n;
      if ((nodes.size() > 1) && (n != (nodes.end() - 1))) {
        cout << ",";
      } 
  }
  cout << "]";
  return out;
}

//=============================================================================
using Distance = size_t;
constexpr Distance MAX_DISTANCE = SIZE_MAX;
class Edge {
  friend class Vertex;
  friend ostream& operator<<(ostream& out, const Edge& e);

public:
  Edge(Node n, Distance d): node{n}, distance{d} {}
  Node getNode() { return node; }
  Distance getDistance() { return distance; }
  void setNode(Node n) { node = n; }
  void setDistance(Distance d) { distance = d; }
  void setEdge(Node n, Distance d) { setNode(n); setDistance(d); }
  Edge getEdge() { return Edge(node, distance); }

private:  
  Node node;
  Distance distance;
};

ostream& operator<<(ostream& out, const Edge& e) {
  out << "(" << e.node << "," << e.distance << ")";
  return out;
}

//=============================================================================
class Vertex {
  friend class Graph;

public:
  vector<Edge>& getEdges() { return edges; };
  void addEdge(Edge e) { edges.push_back(e); }
  void deleteEdge(Node node);
  int getNumEdges() { return edges.size(); }
  bool isNodePresent(const Node node);
  Distance getEdgeValue(const Node node);
  void setEdgeValue(const Node node, Distance distance);
  friend ostream& operator<<(ostream& out, const Vertex& v);
  Node getNodeWithShortestDistance();
  
private:
  vector<Edge> edges;
};

ostream& operator<<(ostream& out, const Vertex& v) {
  for (auto e: v.edges) {
    cout << " " << e; 
  }
  return out;
}

bool Vertex::isNodePresent(Node node) {
  bool edgeFound{false};
  for (auto e: edges) {
    if (e.getNode() == node) {
      edgeFound = true;
      break;
    }
  }
  return edgeFound;
}

void Vertex::deleteEdge(Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      edges.erase(e);
      break;
    }
  }
}

Distance Vertex::getEdgeValue(const Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->getDistance();
    }
  }
  return 0;
}

void Vertex::setEdgeValue(const Node node, Distance distance) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->setDistance(distance);
    }
  }
}

Node Vertex::getNodeWithShortestDistance() {
  Node node{Node(INVALID_NODE)};
  size_t numEdges = edges.size();
  if (edges.size() > 0) {
    Distance minDistance{edges.at(0).getDistance()};
    node = edges.at(0).getNode();
    for (size_t i{1}; i < numEdges; ++i) {
      if (edges.at(i).getDistance() < minDistance) {
        node = edges.at(i).getNode();
      }
    }
  }
  return node;
}

//=============================================================================
class Graph {
  friend class ShortestPath;
  friend ostream& operator<<(ostream& out, const Graph& g);

public:
  Graph(size_t numNodes);
  // V() returns the number of vertices in the graph
  int getNumVertices() { return vertices.size(); }

  // E() returns the number of edges in the graph
  int getNumEdges();

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
  vector<Vertex> vertices; // use adjacency list to represent the graph
  vector<Distance> nodeValues;
};

ostream& operator<<(ostream& out, const Graph& g) {
  Node node{Node(0)};
  for (auto v: g.vertices) {
    cout << node << ":";
    cout << v;
    cout << endl;
    ++node;
  }
  return out;
}

Graph::Graph(size_t numNodes) {
  for (size_t i{0}; i < numNodes; ++i) {
    vertices.push_back(Vertex());
    nodeValues.push_back(MAX_DISTANCE);
  }
}

int Graph::getNumEdges() {
  int numEdges{0};
  for(Vertex v: vertices) {
    numEdges += v.getNumEdges();
  }
  return numEdges;
}

bool Graph::adjacent(Node nodeX, Node nodeY) {
  bool edgeFound{false};
  edgeFound = vertices.at(size_t(nodeX)).isNodePresent(nodeY);
  return edgeFound;
}

vector<Node> Graph::neighbors(Node nodeX) {
  vector<Node> n;
  Vertex v = vertices.at(size_t(nodeX));
  for (auto e: v.getEdges()) {
    n.push_back(e.getNode());
  }
  return n;
}

void Graph::addEdge(Node nodeX, Node nodeY, Distance distance) {
  if (!adjacent(nodeX, nodeY)) {
    vertices.at(size_t(nodeX)).addEdge(Edge(nodeY, distance));
  }
}

void Graph::deleteEdge(Node nodeX, Node nodeY) {
  if (adjacent(nodeX, nodeY)) {
    vertices.at(size_t(nodeX)).deleteEdge(nodeY);
  }
}

int Graph::getNodeValue(Node node) {
  return nodeValues.at(int(node)); 
}

void Graph::setNodeValue(Node node, Distance value) {
  nodeValues.at(int(node)) = value; 
}

Distance Graph::getEdgeValue(Node nodeX, Node nodeY) {
  if (adjacent(nodeX, nodeY)) {
    return vertices.at(size_t(nodeX)).getEdgeValue(nodeY);
  }
  return 0;
}

void Graph::setEdgeValue(Node nodeX, Node nodeY, Distance distance) {
  if (adjacent(nodeX, nodeY)) {
    return vertices.at(size_t(nodeX)).setEdgeValue(nodeY, distance);
  }
}

//=============================================================================
class ShortestPath {
  friend ostream& operator<<(ostream& out, const vector<Distance>& v);

public:
  ShortestPath(Graph& g): g{g} {};
  vector<Vertex>& getVertices(); // vertices(List): list of vertices in G(V,E).
  vector<Node>& path(Node u, Node w); // path(u, w): find shortest path between u-w and returns the sequence of vertices representing shortest path u-v1-v2-…-vn-w.
  Distance pathSize(Node u, Node w); // path_size(u, w): return the path cost associated with the shortest path.

private:
  Graph& g;
  void initShortestPathSearch(Node u, Node w);
  Node start, destination;
  size_t numNodes;
  vector<Node> Q; // list of nodes that have not been visited
  void initNodesNotVisitedList();
  bool isNodeNotVisited(Node n);
  vector<Distance>dist; // list of shortest distances to nodes
  void initShortestDistanceToNodesList();
  vector<Node> prev; // list of previous nodes with shortest distance to a node
  void initPreviousNodesList();
  vector<Node> S; // list of nodes that is the shortest path
  vector<Node>& createShortestPathFromPrevNodesList();
  Node findNodeWithMinDistance();
  void markNodeAsVisited(Node u);
  void traverseNeighbors(Node u);
  Distance calcTotalDistanceToNeighbor(Node u, Node v);
  void updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt);
};

ostream& operator<<(ostream& out, const vector<Distance>& v) {
  cout << "[ ";
  Node n{Node(0)};
  for (auto d: v) {
    cout << "(" << n << ":" << d << ") ";
    ++n;
  }
  cout << "]";
  return out;
}

vector<Vertex>& ShortestPath::getVertices() {
  return g.vertices;
}

void ShortestPath::initShortestPathSearch(Node u, Node w) {
  // make sure graph is not empty
  numNodes = g.getNumVertices();
  assert(numNodes > 0);
  start = u;
  destination = w;
  initNodesNotVisitedList();
  initShortestDistanceToNodesList();
  initPreviousNodesList();
}

void ShortestPath::initNodesNotVisitedList() {
  Q.clear();
  for (size_t i{0}; i < numNodes; ++i) {
    Q.push_back(Node(i));
  }
  cout << "Q: " << Q << endl;
}

bool ShortestPath::isNodeNotVisited(Node n) {
  bool found{false};
  for (Node node: Q) {
    if (node == n) {
      found = true;
      break;
    }
  }
  return found;
}

void ShortestPath::initShortestDistanceToNodesList() {
  dist.clear();
  for (size_t i{0}; i < numNodes; i++) {
    dist.push_back(MAX_DISTANCE);
  }
  dist.at(size_t(start)) = 0;
  cout << "dist: " << dist << endl;
}

void ShortestPath::initPreviousNodesList() {
  prev.clear();
  for (size_t i{0}; i < numNodes; ++i) {
    prev.push_back(Node(INVALID_NODE));
  }
  cout << "prev: " << prev << endl;
}

vector<Node>& ShortestPath::createShortestPathFromPrevNodesList() {
  S.clear(); // empty sequence
  Node u{destination}; // u = target
  if (prev[size_t(u)] != Node(INVALID_NODE) or u == start) { // if pre[u] is defined or u = source
    while (u != Node(INVALID_NODE)) { // while u is defined
      S.insert(S.begin(), u); // insert u at beginning of S
      u = prev[size_t(u)]; // u = prev[u]
    }
  }
  return S;
}

Node ShortestPath::findNodeWithMinDistance() {
  Distance min = MAX_DISTANCE;
  Node u{Node(INVALID_NODE)};
  for (auto i{Q.begin()}; i < Q.end(); ++i) {
    Node n = *i;
    if(dist[size_t(n)] < min) {
      min = dist[size_t(n)];
      u = n;
    }
  }
  cout << "u: " << u << endl;
  return u;
}

void ShortestPath::markNodeAsVisited(Node u) {
  // remove u from Q
  for (auto i{Q.begin()}; i < Q.end(); ++i) {
    if (*i == u) {
      Q.erase(i);
      break;
    }
  }
  cout << "Q: " << Q << endl;
}

void ShortestPath::updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt) {
  if (alt < dist[size_t(v)]) { // if alt < dist[v]
    dist[size_t(v)] = alt; // dist[v] = alt
    cout << "dist: " << dist << endl;
    prev[size_t(v)] = u; // prev[v] = u
    cout << "prev: " << prev << endl;
  }
}

void ShortestPath::traverseNeighbors(Node u) {
  // for each neighbor v of u still in Q
  vector<Node> neighbors = g.neighbors(u);
  cout << "neighbors: " << neighbors << endl;
  for (auto v: neighbors) {
    if (isNodeNotVisited(v)) {
      Distance alt = calcTotalDistanceToNeighbor(u,v);
      updateMinDistanceAndPreviousNodesLists(u,v,alt);
    }
  }
}

Distance ShortestPath::calcTotalDistanceToNeighbor(Node u, Node v) {
  // alt = dist[u] + G.edges(u,v)
  Distance alt{dist[size_t(u)]};
  alt += g.getEdgeValue(u, v);
  cout << "alt: " << u << "," << v << "," << alt << endl;
  return alt;
}

vector<Node>& ShortestPath::path(Node u, Node w) {
  // Use Dijkstra's algorithm as described here:
  // "https://en.wikipedia.org/wiki/Dijkstra's_algorithm"
  initShortestPathSearch(u,w);
  while( Q.size() > 0) { // while Q is not empty
    cout << "===============" << endl;
    Node u{findNodeWithMinDistance()};
    if (u == destination or u == Node(INVALID_NODE))
      break;
    markNodeAsVisited(u);
    traverseNeighbors(u);
  }
  cout << "===============" << endl;
  return createShortestPathFromPrevNodesList();
}

Distance ShortestPath::pathSize(Node u, Node w) {
  vector<Node>& nodes{S};
  if (u != start or w != destination) {
    nodes = path(start, destination);
  }
  Distance distance{0};
  size_t numNodes{nodes.size()}, index{0};
  while (numNodes-- > 1) {
    distance += g.getEdgeValue(nodes[index], nodes[index+1]);
    ++index;
  }
  return distance;
}

//=============================================================================
Graph createExampleGraph() {
  Graph g(NUM_NODES);

  g.addEdge(Node::S, Node::A, 4);
  g.addEdge(Node::S, Node::B, 3);
  g.addEdge(Node::S, Node::D, 7);

  g.addEdge(Node::A, Node::C, 1);

  g.addEdge(Node::B, Node::D, 4);
  g.addEdge(Node::B, Node::S, 3);

  g.addEdge(Node::C, Node::D, 3);
  g.addEdge(Node::C, Node::E, 1);

  g.addEdge(Node::D, Node::F, 5);
  g.addEdge(Node::D, Node::T, 3);

  g.addEdge(Node::E, Node::G, 2);
  g.addEdge(Node::E, Node::T, 4);

  g.addEdge(Node::G, Node::T, 3);
  g.addEdge(Node::G, Node::E, 2);

  g.addEdge(Node::T, Node::F, 5);

  g.addEdge(Node::S, Node::T, 20);
  g.deleteEdge(Node::S, Node::T);

  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  cout << "Example Graph:" << endl << g;

  return g;
}

void testExampleGraph() {
  Graph g{createExampleGraph()};

  cout << "S->A = " << g.getEdgeValue(Node::S, Node::A) << endl;
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;
  g.setEdgeValue(Node::G, Node::T, 5);
  cout << "G->T = " << g.getEdgeValue(Node::G, Node::T) << endl;

  Node start{Node::S}, dest{Node::T};
  ShortestPath sp(g);
  auto path = sp.path(start, dest);
  cout << "shortest path = " << path << endl;
  Distance distance = sp.pathSize(start, dest);
  cout << "distance = " << distance << endl;
}

//=============================================================================
Graph createWikipediaGraph() {
  Graph g(NUM_NODES);

  g.addEdge(Node::A, Node::B, 7);
  g.addEdge(Node::A, Node::C, 9);
  g.addEdge(Node::A, Node::F, 14);

  g.addEdge(Node::B, Node::C, 10);
  g.addEdge(Node::B, Node::D, 15);
  
  g.addEdge(Node::C, Node::D, 11);
  g.addEdge(Node::C, Node::F, 2);

  g.addEdge(Node::D, Node::E, 6);
  g.addEdge(Node::E, Node::F, 9);
  g.addEdge(Node::F, Node::E, 9);

  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  cout << "Wikipedia Graph:" << endl << g;
  return g;
}

void testWikipediaGraph() {
  Graph g{createWikipediaGraph()};
  ShortestPath sp(g);
 
  Node start{Node::A}, dest{Node::E};
  auto path = sp.path(start, dest);
  cout << "shortest path = " << path << endl;
  Distance distance = sp.pathSize(start, dest);
  cout << "distance = " << distance << endl;
}

//=============================================================================

// Basic problem:  
// Write a set of constructors for declaring and initializing a graph. 
// An edge will have a positive cost that is its distance. 
// Have a procedure that produces a randomly generated set of edges with positive distances.  
// Assume the graphs are undirected. 
// The random graph procedure should have edge density as a parameter,
// and the distance range as a parameter. 
// So a graph whose density is 0.1 would have 10% of its edges picked at random.
// Its edge distance would be selected at random from the distance range. 
// The procedure should run through all possible undirected edges, say (i,j). 
// Place the edge in the graph if a random probability calculation is less than the density. 
// Compute for a set of randomly generated graphs an average shortest path. 

Graph createRandomGraph(size_t numNodes) {
  Graph g(numNodes);

  cout << "Vertices = " << g.getNumVertices() << "; Edges = " << g.getNumEdges() << endl;
  cout << "Random Graph:" << endl << g;
  return g;
}

//=============================================================================
int main() {
  testExampleGraph();
  testWikipediaGraph();

  return 0;
}
