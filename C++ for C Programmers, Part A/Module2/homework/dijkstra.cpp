#include <iostream>
#include <vector>
#include <cassert>
#include <string>
#include <iomanip>

using namespace std;

//=============================================================================
enum class Node: int { 
  A, B, C, D, E, F, G, H, I, J,
  A1, B1, C1, D1, E1, F1, G1, H1, I1, J1,
  A2, B2, C2, D2, E2, F2, G2, H2, I2, J2,
  A3, B3, C3, D3, E3, F3, G3, H3, I3, J3,
  A4, B4, C4, D4, E4, F4, G4, H4, I4, J4,
};
constexpr Node NO_NODE = Node(-1);
vector<string> nodeDescriptors{
  "A","B","C","D","E","F","G","H","I","J",
  "A1","B1","C1","D1","E1","F1","G1","H1","I1","J1",
  "A2","B2","C2","D2","E2","F2","G2","H2","I2","J2",
  "A3","B3","C3","D3","E3","F3","G3","H3","I3","J3",
  "A4","B4","C4","D4","E4","F4","G4","H4","I4","J4",
};

ostream& operator<<(ostream& out, const Node& n) {
  int i = int(n);
  if (i >= 0 and size_t(i) < nodeDescriptors.size()) {
    out << nodeDescriptors[i];
  }
  else {
    out << i;
  }
  return out;
}

Node operator++(Node& n) { 
  n = static_cast<Node>((static_cast<int>(n) + 1));
  return n; 
}
Node operator++(Node& n, int) {
  Node temp = n;
  n = Node(int(n) + 1);
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
#include<limits>
using Distance = double;
constexpr Distance MAX_DISTANCE = numeric_limits<Distance>::max();
class Edge {
  friend class EdgeList;
  friend ostream& operator<<(ostream& out, const Edge& e);

public:
  Edge(Node n, Distance d): node{n} { setDistance(d); }
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
  out << "(" << e.node << "," << setprecision(3) << e.distance << ")";
  return out;
}

//=============================================================================
class EdgeList {
  friend class Graph;

public:
  vector<Edge>& getEdges() { return edges; };
  void addEdge(Edge e) { edges.push_back(e); }
  void deleteEdge(Node node);
  size_t getNumEdges() { return edges.size(); }
  bool isNodePresent(const Node node);
  Distance getEdgeValue(const Node node);
  void setEdgeValue(const Node node, Distance distance);
  friend ostream& operator<<(ostream& out, const EdgeList& v);
  
private:
  vector<Edge> edges;
};

ostream& operator<<(ostream& out, const EdgeList& v) {
  for (auto e: v.edges) {
    cout << " " << e; 
  }
  return out;
}

bool EdgeList::isNodePresent(Node node) {
  bool edgeFound{false};
  for (auto e: edges) {
    if (e.getNode() == node) {
      edgeFound = true;
      break;
    }
  }
  return edgeFound;
}

void EdgeList::deleteEdge(Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      edges.erase(e);
      break;
    }
  }
}

Distance EdgeList::getEdgeValue(const Node node) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->getDistance();
    }
  }
  return 0;
}

void EdgeList::setEdgeValue(const Node node, Distance distance) {
  for (auto e = edges.begin(); e != edges.end(); ++e) {
    if (e->getNode() == node) {
      return e->setDistance(distance);
    }
  }
}

//=============================================================================
enum class GraphType { DIRECTED, UNDIRECTED };
ostream& operator<<(ostream& out, const GraphType& type) {
  switch(type) {
  case GraphType::DIRECTED:
    out << "Directed"; break;
  case GraphType::UNDIRECTED:
    out << "Undirected"; break;
  }
  return out;
}

//=============================================================================
class Graph {
  friend class ShortestPath;
  friend ostream& operator<<(ostream& out, const Graph& g);

public:
  Graph(size_t numNodes, GraphType type, string name="");
  const GraphType getType() const { return type; }
  const string& getName() const { return name; }
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
};

ostream& operator<<(ostream& out, const vector<EdgeList>& vertices) {
  Node node{Node(0)};
  for (auto v: vertices) {
    cout << node << ":";
    cout << v;
    cout << endl;
    ++node;
  }
  return out;
}

ostream& operator<<(ostream& out, const Graph& g) {
  cout << "================================================" << endl;
  cout << g.getName() << " Graph:" << endl;
  cout << g.vertices;
  cout << "Type = " << g.getType() 
    << "; Vertices = " << g.getNumVertices() 
    << "; Edges = " << g.getNumEdges() << endl;
  cout << "Density = " << setprecision(3) << g.density() << endl;
  return out;
}

double Graph::density() const {
  // "https://www.baeldung.com/cs/graph-density"
  size_t numVertices = getNumVertices();
  double numEdges = getNumEdges();
  return numEdges / double(numVertices * (numVertices - 1)); 
}

Graph::Graph(size_t numNodes, GraphType type, string name):type{type}, name{name} {
  for (size_t i{0}; i < numNodes; ++i) {
    vertices.push_back(EdgeList());
    nodeValues.push_back(MAX_DISTANCE);
  }
}

size_t Graph::getNumEdges() const {
  size_t numEdges{0};
  for(EdgeList v: vertices) {
    numEdges += v.getNumEdges();
  }
  if (type == GraphType::UNDIRECTED) {
    numEdges /= 2;
  }  
  return numEdges;
}

bool Graph::adjacent(Node nodeX, Node nodeY) {
  return vertices.at(size_t(nodeX)).isNodePresent(nodeY);
}

vector<Node> Graph::neighbors(Node nodeX) {
  vector<Node> n;
  EdgeList v = vertices.at(size_t(nodeX));
  for (auto e: v.getEdges()) {
    n.push_back(e.getNode());
  }
  return n;
}

void Graph::addEdge(Node nodeX, Node nodeY, Distance distance) {
  if (nodeX != nodeY) {
    if (!adjacent(nodeX, nodeY))  {
      vertices.at(size_t(nodeX)).addEdge(Edge(nodeY, distance));
      if (type == GraphType::UNDIRECTED) {
        vertices.at(size_t(nodeY)).addEdge(Edge(nodeX, distance));
      }
    }
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
class PriorityQueueElement {
  friend class PriorityQueue;

public: 
  PriorityQueueElement(Node n, Distance d): node{n}, value{d} {}
  Node getNode() { return node; }
  Distance getValue() { return value; }
  
  bool operator<(const PriorityQueueElement& other) const { return value < other.value; }

private:
  Node node;
  Distance value;
};

//=============================================================================
#include <algorithm>
class PriorityQueue {
  friend ostream& operator<<(ostream& out, const PriorityQueue& pq);

public:
  PriorityQueue() {}
  // create list with starting node at the top
  void initialize(size_t numNodes, Node start);

  void add(Node n, Distance d);
  bool isEmpty() { return size() == 0; };
  
  // chgPrioirity(PQ, priority): changes the priority (node value) of queue element.
  void changePriority(Node n, Distance d);

  // minPrioirty(PQ): removes the top element of the queue.
  Node minPriority();

  // contains(PQ, queue_element): does the queue contain queue_element.
  bool contains(PriorityQueueElement qe);

  // Insert(PQ, queue_element): insert queue_element into queue
  void insert(PriorityQueueElement qe);

  // top(PQ):returns the top element of the queue.
  PriorityQueueElement top() { return q.at(0); }

  // size(PQ): return the number of queue_elements.
  size_t size() const { return q.size(); }

private:
  vector<PriorityQueueElement> q;
};

ostream& operator<<(ostream& out, const vector<PriorityQueueElement>& pq) {
  cout << pq.size() << " [ ";
  for (auto qe: pq) {
    cout << "(" << qe.getNode() << ":" << qe.getValue() << ") ";
  }
  cout << "]";
  return out;
}

ostream& operator<<(ostream& out, const PriorityQueue& pq) {
  cout << pq.q;
  return out;
}

void PriorityQueue::initialize(size_t numNodes, Node start) {
  q.clear();
  add(start, 0);
  for (size_t i{0}; i < numNodes; ++i) {
    if (i != size_t(start)) {
      add(Node(i), MAX_DISTANCE);
    }
  }
}

void PriorityQueue::add(Node n, Distance d) {
  q.push_back(PriorityQueueElement(n,d));
}

Node PriorityQueue::minPriority() {
  if (isEmpty()) {
    return NO_NODE;
  }
  Node n{q[0].getNode()};
  q.erase(q.begin());
#ifdef ENABLE_DEBUG
  cout << "q: " << q << endl;
#endif
  return n;
}

void PriorityQueue::changePriority(Node n, Distance d) {
  for (auto& x: q) {
    if (x.node == n) {
      x.value = d;
      break;
    }
  }
  sort(q.begin(), q.end());
}

bool PriorityQueue::contains(PriorityQueueElement qe) {
  for (auto x: q) {
    if (x.node == qe.node)
      return true;
  }
  return false;
}

void PriorityQueue::insert(PriorityQueueElement qe) {
  if(!contains(qe))
    q.insert(q.begin(), qe); 
}

//=============================================================================
using Path = vector<Node>;
class ShortestPath {
  friend ostream& operator<<(ostream& out, const vector<Distance>& v);

public:
  ShortestPath(Graph& g): g{g} {};
  // vertices(List): list of vertices in G(V,E).
  vector<EdgeList>& getVertices();
  
  // path(u, w): find shortest path between u-w and 
  // returns the sequence of vertices representing shortest path u-v1-v2-…-vn-w.
  const Path& path(Node u, Node w);

  // path_size(u, w): return the path cost associated with the shortest path.
  Distance pathSize(Node u, Node w);

private:
  Graph& g;
  PriorityQueue pq;
  void initShortestPathSearch(Node u, Node w);
  Node start{NO_NODE}, destination{NO_NODE};
  size_t numNodes{0};
  vector<Distance>dist; // list of shortest distances to nodes
  void initShortestDistanceToNodesList();
  vector<Node> prev; // list of previous nodes with shortest distance to a node
  void initPreviousNodesList();
  Path shortestPath;
  void updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt);
  void traverseNeighbors(Node u);
  Distance calcTotalDistanceToNeighbor(Node u, Node v);
  const Path& createShortestPathFromPrevNodesList();
};

ostream& operator<<(ostream& out, const vector<Distance>& v) {
  cout << "[ ";
  Node n{Node(0)};
  for (auto d: v) {
    cout << "(" << n << ":" << setprecision(3) << d << ") ";
    ++n;
  }
  cout << "]";
  return out;
}

vector<EdgeList>& ShortestPath::getVertices() {
  return g.vertices;
}

void ShortestPath::initShortestPathSearch(Node u, Node w) {
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

void ShortestPath::initShortestDistanceToNodesList() {
  dist.clear();
  for (size_t i{0}; i < numNodes; i++) {
    dist.push_back(MAX_DISTANCE);
  }
  dist.at(size_t(start)) = 0;
#ifdef ENABLE_DEBUG
  cout << "dist: " << dist << endl;
#endif
}

void ShortestPath::initPreviousNodesList() {
  prev.clear();
  for (size_t i{0}; i < numNodes; ++i) {
    prev.push_back(NO_NODE);
  }
#ifdef ENABLE_DEBUG
  cout << "prev: " << prev << endl;
#endif
}

void ShortestPath::updateMinDistanceAndPreviousNodesLists(Node u, Node v, Distance alt) {
  dist[size_t(v)] = alt;
  prev[size_t(v)] = u;
#ifdef ENABLE_DEBUG
  cout << "dist: " << dist << endl;
  cout << "prev: " << prev << endl;
#endif
}

void ShortestPath::traverseNeighbors(Node u) {
  vector<Node> neighbors = g.neighbors(u);
#ifdef ENABLE_DEBUG
  cout << "neighbors: " << neighbors << endl;
#endif  
  for (auto v: neighbors) {
    Distance alt = calcTotalDistanceToNeighbor(u,v);
    if (alt < dist[size_t(v)]) {
      updateMinDistanceAndPreviousNodesLists(u,v,alt);
      pq.changePriority(v, alt);
#ifdef ENABLE_DEBUG
      cout << "pq: " << pq << endl;
#endif
    }
  }
}

Distance ShortestPath::calcTotalDistanceToNeighbor(Node u, Node v) {
  Distance alt{dist[size_t(u)]};
  alt += g.getEdgeValue(u, v);
#ifdef ENABLE_DEBUG
  cout << "alt: " << u << "," << v << "," << alt << endl;
#endif
  return alt;
}

const Path& ShortestPath::createShortestPathFromPrevNodesList() {
  shortestPath.clear();
  Node u{destination};
  if (prev[size_t(u)] != NO_NODE or u == start) {
    while (u != NO_NODE) {
      shortestPath.insert(shortestPath.begin(), u);
      u = prev[size_t(u)];
    }
  }
  return shortestPath;
}

const Path& ShortestPath::path(Node u, Node w) {
  // Use Dijkstra's algorithm as described here:
  // "https://en.wikipedia.org/wiki/Dijkstra's_algorithm"
  initShortestPathSearch(u,w);
  while(!pq.isEmpty()) {
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

Distance ShortestPath::pathSize(Node u, Node w) {
  Path& sp{shortestPath};
  if (u != start or w != destination) {
    sp = path(u, w);
  }
  Distance distance{0};
  size_t numNodes{sp.size()}, i{0};
  while (numNodes-- > 1) {
    distance += g.getEdgeValue(sp[i], sp[i+1]);
    ++i;
  }
  return distance;
}

//=============================================================================
Graph createExampleGraph() {
  constexpr int NUM_NODES = int(Node::I) + 1; 
  Graph g(NUM_NODES, GraphType::DIRECTED, "Example");

  g.addEdge(Node::H, Node::A, 4);
  g.addEdge(Node::H, Node::B, 3);
  g.addEdge(Node::H, Node::D, 7);

  g.addEdge(Node::A, Node::C, 1);

  g.addEdge(Node::B, Node::D, 4);
  g.addEdge(Node::B, Node::H, 3);

  g.addEdge(Node::C, Node::D, 3);
  g.addEdge(Node::C, Node::E, 1);

  g.addEdge(Node::D, Node::F, 5);
  g.addEdge(Node::D, Node::I, 3);

  g.addEdge(Node::E, Node::G, 2);
  g.addEdge(Node::E, Node::I, 4);

  g.addEdge(Node::G, Node::I, 3);
  g.addEdge(Node::G, Node::E, 2);

  g.addEdge(Node::I, Node::F, 5);

  g.addEdge(Node::H, Node::I, 20);
  g.deleteEdge(Node::H, Node::I);

  cout << g;
  return g;
}

//=============================================================================
Graph createWikipediaGraph() {
  constexpr int NUM_NODES = int(Node::F) + 1; 
  Graph g(NUM_NODES, GraphType::UNDIRECTED, "Wikipedia");

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

  cout << g;
  return g;
}

//=============================================================================
#include <random>
#include <ctime>
Graph createRandomGraph(size_t numNodes, double density, Distance min, Distance max) {
  default_random_engine e(time(0));
  std::uniform_real_distribution<Distance> randomDistance(min, max);
  uniform_int_distribution<int> randomNode(0,numNodes-1);

  Graph g(numNodes, GraphType::UNDIRECTED, "Random");
  g.addEdge(Node::A, Node(randomNode(e)), randomDistance(e));
  while (g.density() < density) {
    g.addEdge(Node(randomNode(e)), Node(randomNode(e)), randomDistance(e));
  }
  cout << g;
  return g;
}

//=============================================================================
#include <tuple>
using PathInfo = tuple<const Path, Distance>;
PathInfo testShortestPath(Graph& g, Node start, Node dest) {
  ShortestPath sp(g);
  auto path = sp.path(start, dest);
  cout << "shortest path = " << path << endl;
  Distance distance = sp.pathSize(start, dest);
  cout << "distance = " << distance << endl;
  return {path, distance};
}

double calcAveragePathLength(Graph& g) {
  ShortestPath sp(g);
  Distance sum{0};
  size_t count{0};
  for (size_t i{1}; i < g.getNumVertices(); ++i) {
    Distance d = sp.pathSize(Node(0), Node(i));
    if(d != 0) {
      sum += d;
      ++count;
    }
  }
  return double(sum) / double(count);
}

int main() {
  Path path;
  Distance distance;

  Graph gExample{createExampleGraph()};
  tie(path, distance) = testShortestPath(gExample, Node::H, Node::I);
  const Path EXAMPLE_PATH = {Node::H,Node::A,Node::C,Node::E,Node::I};
  assert(path == EXAMPLE_PATH and distance == 10);

  Graph gWiki{createWikipediaGraph()};
  tie(path, distance) = testShortestPath(gWiki, Node::A, Node::E);
  const Path WIKI_PATH = {Node::A,Node::C,Node::F,Node::E};
  assert(path == WIKI_PATH and distance == 20);

  Graph gRandom1{createRandomGraph(50, 0.2, 1.0, 10.0)};
  cout << "Average path length = " << setprecision(3) << calcAveragePathLength(gRandom1) << endl;

  Graph gRandom2{createRandomGraph(50, 0.4, 1.0, 10.0)};
  cout << "Average path length = " << setprecision(3) << calcAveragePathLength(gRandom2) << endl;
  
  return 0;
}
