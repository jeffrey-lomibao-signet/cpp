#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <ctime>
#include <cstdlib>
using namespace std;
//==============================================================================
// Generaldefinitions
//==============================================================================
// INFINITisusedtorepresentnoedge/pathbetweentwonodes
const int INFINIT = 999999;
// Overloadoperator<<toprintlist<char>variables
ostream &operator<<(ostream &output, list<char> L)
{
  list<char>::iterator i;
  for (i = L.begin(); i != L.end(); ++i)
    output << *i << "";
  return output;
}
// Convertnodenumbersintochars(from0..51toA..Za..z)
inline char vertIntToChar(int n)
{
  if (n < 26)
    return static_cast<char>('A' + n);
  else
    return static_cast<char>('a' + n - 26);
}
//==============================================================================
// Nodedefinitions
// Usedtostoreinformationaboutnodes/edgesintheadjacencylistofagraph
// AdjacencylistsisalistofNodes(identifiedbynumbersfrom0to51)
// Eachnodecontainsalistofneighborscontainingedgeweight
//==============================================================================
typedef struct strNode Node;
struct strNode
{
  int number;
  int weight;
  list<Node> edges;
};
//==============================================================================
// GraphClass
// RepresentaGraphthroughanadjacencylist
//==============================================================================
class Graph
{
public:
  Graph();
  Graph(int numVertices, int initialValue);
  char get_node_value(int x);
  void set_node_value(char x, char name);
  int get_edge_value(char x, char y);
  void set_edge_value(char x, char y, int value);
  bool adjacent(char x, char y);
  list<char> neighbors(char x);
  int V();
  int E();
  list<char> vertices();
  void show();

private:
  int numV;                   // NumberofnodesoftheGraph
  int numE;                   // NumberofedgesoftheGraph
  vector<char> nodeNames;     // Mapnodenumbersintonodenames
  map<char, int> nodeNumbers; // Mapnodenamesintonodenumbers
  list<Node> adjList;         // AdjacencylistrepresentingtheGraph
};
// DefaultconstructorofGraphClass
// Createanemptygraph
Graph::Graph()
{
  numV = 0;
  numE = 0;
  adjList.clear();
}
// ConstructorofGraphClass
// Initializenumberofnodes
// Createadjacencylistwithallnodesandemptyedgelist
Graph::Graph(int numVertices, int initialValue = INFINIT)
{
  // Createnodesandlinkittodefaultnames(0..51->A..Za..z)
  numV = numVertices;
  numE = 0;
  nodeNames.resize(numVertices);
  for (int x = 0; x < numV; ++x)
  {
    nodeNames[x] = vertIntToChar(x);
    nodeNumbers[vertIntToChar(x)] = x;
  }
  // Createadjacencylistwithallnodesandemptyedgelist
  adjList.clear();
  for (int i = 0; i < numVertices; ++i)
  {
    Node newNode;
    newNode.number = i;
    newNode.weight = 0;
    newNode.edges.clear();
    adjList.push_back(newNode);
  }
}
// Returnnodenamelinkedtonodenumberx
char Graph::get_node_value(int x)
{
  return nodeNames[x];
}
// Changenodename(from'x'to'name')
void Graph::set_node_value(char x, char name)
{
  int posX = nodeNumbers[x]; // Getthenumberofnode'x'
  nodeNames[posX] = name;    // Linknodenumberto'name'
  nodeNumbers[name] = posX;  // Link'name'tonodenumber
}
// Returnedgeweightbetween'x'and'y'
// ReturnINFINITYifedgedoesn'texist
int Graph::get_edge_value(char x, char y)
{
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    if ((*i).number == nodeNumbers[x])
      for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
      {
        if ((*j).number == nodeNumbers[y])
          return (*j).weight;
      }
  }
  return INFINIT;
}
// Setedgeweightbetween'x'and'y'
void Graph::set_edge_value(char x, char y, int value)
{
  bool found;
  // Add'y'inthelistof'x'neighbors(ifdoesn'texist)
  // Setedgeweighttovalue
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    if ((*i).number == nodeNumbers[x])
    {
      found = false;
      for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
      {
        if ((*j).number == nodeNumbers[y])
        {
          (*j).weight = value;
          found = true;
        }
      }
      if (!found)
      {
        Node newNodeY;
        newNodeY.number = nodeNumbers[y];
        newNodeY.weight = value;
        newNodeY.edges.clear();
        (*i).edges.push_back(newNodeY);
      }
    }
  }
  // Add'x'inthelistof'y'neighbors(ifdoesn'texist)
  // Setedgeweighttovalue
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    if ((*i).number == nodeNumbers[y])
    {
      found = false;
      for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
      {
        if ((*j).number == nodeNumbers[x])
        {
          (*j).weight = value;
          found = true;
        }
      }
      if (!found)
      {
        Node newNodeX;
        newNodeX.number = nodeNumbers[x];
        newNodeX.weight = value;
        newNodeX.edges.clear();
        (*i).edges.push_back(newNodeX);
        ++numE; // Incrementthenumberofedgesinthegraph
      }
    }
  }
}
// Returntrueif'x'and'y'areneighborsandfalseotherwise
bool Graph::adjacent(char x, char y)
{
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    if ((*i).number == nodeNumbers[x])
    {
      for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
      {
        if ((*j).number == nodeNumbers[y])
        {
          return true;
        }
      }
    }
  }
  return false;
}
// Returnalist<char>containingthelistofneighborsof'x'
list<char> Graph::neighbors(char x)
{
  list<char> adjNodes;
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    if ((*i).number == nodeNumbers[x])
    {
      for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
      {
        adjNodes.push_back(nodeNames[(*j).number]);
      }
    }
  }
  return adjNodes;
}
// ReturnthenumberofnodesintheGraph
int Graph::V()
{
  return numV;
}
// ReturnthenumberofedgesintheGraph
int Graph::E()
{
  return numE;
}
// Returnalist<char>containingallnodesintheGraph
list<char> Graph::vertices()
{
  list<char> nodes;
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    nodes.push_back(nodeNames[(*i).number]);
  }
  return nodes;
}
// PrintoutadjacencylistrepresentingtheGraph
void Graph::show()
{
  cout << " ";
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
    cout << "" << nodeNames[(*i).number];
  cout << endl;
  for (list<Node>::iterator i = adjList.begin(); i != adjList.end(); ++i)
  {
    cout << "" << nodeNames[(*i).number];
    int shift = 0;
    for (list<Node>::iterator j = (*i).edges.begin(); j != (*i).edges.end(); ++j)
    {
      int walk = (*j).number - shift;
      for (int k = 0; k < walk; ++k)
      {
        cout << "-";
        shift++;
      }
      cout << "" << (*j).weight;
      shift++;
    }
    while (shift < numV)
    {
      cout << "-";
      shift++;
    }
    cout << endl;
  }
}
//==============================================================================
// NodeInfoDefinitions
// Usedtostoreinformationaboutnodes,pathsandmindistsinpriorityqueue
//==============================================================================
struct strNodeInfo
{
  char nodeName; // Nodename
  int minDist;   // ShortestpathfoundtonodeName
  char through;  // NodethatprecedenodeNameintheshortestpath
};
typedef struct strNodeInfo NodeInfo;
// CompareNodeInfobynodeName
bool compareNodeName(NodeInfo &n1, NodeInfo &n2)
{
  if (n1.nodeName < n2.nodeName)
    return true;
  return false;
}
// CompareNodeInfobyminDist
bool compareMinDist(NodeInfo &n1, NodeInfo &n2)
{
  if (n1.minDist < n2.minDist)
    return true;
  return false;
}
// ReturntrueiftwoNodeInfohavethesamenodeNameandfalseotherwise
bool operator==(NodeInfo &n1, NodeInfo &n2)
{
  if (n1.nodeName == n2.nodeName)
    return true;
  return false;
}
//==============================================================================
// PriorityQueueClass
// Storesknowninformationaboutnodenames,mindistancesandpaths
// Orderedbymindistances
//==============================================================================
class PriorityQueue
{
public:
  PriorityQueue();
  void chgPriority(NodeInfo n);
  void minPriority();
  bool contains(NodeInfo n);
  bool isBetter(NodeInfo n);
  void insert(NodeInfo n);
  NodeInfo top();
  int size();

private:
  list<NodeInfo> pq; // Listofknownnodes/pathsorderedbyminDist
};
// ConstructorofPriorityQueueClass
// Createsanemptylistofnodes
PriorityQueue::PriorityQueue()
{
  pq.clear();
}
// Changeinformation('minDist'and'through')ofanodenamed'n'inpriorityqueue
void PriorityQueue::chgPriority(NodeInfo n)
{
  for (list<NodeInfo>::iterator i = pq.begin(); i != pq.end(); ++i)
    if ((*i) == n)
    {
      (*i).minDist = n.minDist;
      (*i).through = n.through;
    }
  pq.sort(compareMinDist);
}
// RemovethenodewithlowerminDistfrompriorityqueue
void PriorityQueue::minPriority()
{
  if (!pq.empty())
  {
    pq.pop_front();
  }
}
// Returnetrueifthereisanodenamed'n'inpriorityqueueandfalseotherwise
bool PriorityQueue::contains(NodeInfo n)
{
  for (list<NodeInfo>::iterator i = pq.begin(); i != pq.end(); ++i)
    if ((*i).nodeName == n.nodeName)
      return true;
  return false;
}
// Returntrueifnode'n'hasalowerminDistthanthenodewiththesamenameinthepriorityqueueandfalseotherwise
bool PriorityQueue::isBetter(NodeInfo n)
{
  for (list<NodeInfo>::iterator i = pq.begin(); i != pq.end(); ++i)
    if ((*i).nodeName == n.nodeName)
      if ((*i).minDist > n.minDist)
        return true;
  return false;
}
// Insertnode'n'intopriorityqueue
void PriorityQueue::insert(NodeInfo n)
{
  pq.push_back(n);
  pq.sort(compareMinDist);
}
// ReturnthenodewithlowerminDistinpriorityqueue(withoutremovingitfromthequeue))
NodeInfo PriorityQueue::top()
{
  NodeInfo n = {' ', 0};
  if (!pq.empty())
  {
    list<NodeInfo>::iterator i = pq.begin();
    n.nodeName = (*i).nodeName;
    n.minDist = (*i).minDist;
    n.through = (*i).through;
  }
  return n;
}
// Returnthenumberofelementsinthepriorityqueue
int PriorityQueue::size()
{
  return pq.size();
}
//==============================================================================
// ShortestPathClass
// ImplementsDijkstra'sAlgorithmtofindshortestpathsbetweentwonodes
//==============================================================================
class ShortestPath
{
public:
  ShortestPath();
  ShortestPath(Graph g);
  list<char> path(char u, char w);
  int path_size(char u, char w);

private:
  Graph graph; // GraphusedbyDiajkstra'sAlgorithm
};
// ConstructorofShortestPathClass(donothing)
ShortestPath::ShortestPath()
{
}
// ConstructorofShortestPathClassthatstoresGraphusedbyDijkstra'sAlgorithm
ShortestPath::ShortestPath(Graph g)
{
  graph = g;
}
// Returnalist<char>containingthelistofnodesintheshortestpathbetween'u'and'w'
list<char> ShortestPath::path(char u, char w)
{
  // Initializecandidateslistwithallnodes
  list<char> candidates = graph.vertices(), desiredPath;
  list<NodeInfo> minPaths;
  PriorityQueue p;
  NodeInfo lastSelected, n;
  // Calculateshortestpathfrom'u'to'w'(Dijkstra'sAlgorithm)
  candidates.remove(u);      // Remove'u'fromcandidateslist
  lastSelected.nodeName = u; // Set'u'aslastSelected
  lastSelected.minDist = 0;
  lastSelected.through = u;
  minPaths.push_back(lastSelected); // Add'u'tominPathlist
  while ((!candidates.empty()) && (lastSelected.nodeName != w))
  {
    // ForeachnodeincandidatelistcalculatethecosttoreachthatcandidatethroughlastSelected
    for (list<char>::iterator i = candidates.begin(); i != candidates.end(); ++i)
    {
      n.nodeName = *i;
      n.minDist = lastSelected.minDist + graph.get_edge_value(lastSelected.nodeName, *i);
      n.through = lastSelected.nodeName;
      if (!p.contains(n)) // Addcandidatetopriorityqueueifdoesn'texist
        p.insert(n);
      else if (p.isBetter(n)) // UpdatecandidateminDistinpriorityqueueifabetterpathwasfound
        p.chgPriority(n);
    }
    lastSelected = p.top();                   // SelectthecandidatewithminDistfrompriorityqueue
    p.minPriority();                          // Removeitfromthepriorityqueue
    minPaths.push_back(lastSelected);         // AddthecandidatewithmindistancetominPathlist
    candidates.remove(lastSelected.nodeName); // Removeitfromcandidateslist
  }
  // Gobackwardfrom'w'to'u'addingnodesinthatpathtodesiredPathlist
  lastSelected = minPaths.back();
  desiredPath.push_front(lastSelected.nodeName);
  while (lastSelected.nodeName != u)
  {
    for (list<NodeInfo>::iterator i = minPaths.begin(); i != minPaths.end(); ++i)
      if ((*i).nodeName == lastSelected.through)
      {
        lastSelected = (*i);
        desiredPath.push_front(lastSelected.nodeName);
      }
  }
  return desiredPath;
}
// Returnthesizeoftheshortestpathbetween'u'and'w'
int ShortestPath::path_size(char u, char w)
{
  int pathCost = 0;
  list<char> sp;
  char current, next;
  // Calculatetheshortestpathfrom'u'to'w'andthensumupedgeweightsinthispath
  sp = path(u, w);
  current = sp.front();
  sp.pop_front();
  for (list<char>::iterator i = sp.begin(); i != sp.end(); ++i)
  {
    next = (*i);
    pathCost += graph.get_edge_value(current, next);
    current = next;
  }
  return pathCost;
}
//==============================================================================
// MonteCarloClass
// Usedtogeneraterandomgraphsandrunsimulations
//==============================================================================
class MonteCarlo
{
public:
  MonteCarlo();
  Graph randomGraph(int vert, double density, int minDistEdge, int maxDistEdge);
  void run(Graph g);

private:
};
// ConstructorofMonteCarloClass
// Initializestheseedofrandomnumbergenerator
MonteCarlo::MonteCarlo()
{
  srand(time(NULL));
}
// Returnarandom Graphgeneratedwithnumberofnodes,densityandedgeweightrangeinformed
Graph MonteCarlo::randomGraph(int numVert, double density, int minDistEdge, int maxDistEdge)
{
  int randDistEdge;
  char srcVert, dstVert;
  Graph g(numVert);
  for (int i = 0; i < g.V(); ++i)
    for (int j = i + 1; j < g.V(); ++j)
    {
      double p = ((static_cast<double>(rand())) / RAND_MAX); // Generaterandomprobability
      if (p < density)                                       // Ifrandomprobabilityislessthandensity,edge(i,j)willbeset
      {
        randDistEdge = rand() % (maxDistEdge - minDistEdge) + minDistEdge; // Generaterandomedgeweight
        srcVert = vertIntToChar(i);
        dstVert = vertIntToChar(j);
        g.set_edge_value(srcVert, dstVert, randDistEdge);
      }
    }
  return g;
}
// Runasimulationfindingtheshortestpathsinagivengraph
void MonteCarlo::run(Graph g)
{
  static int turn = 0;
  cout << endl
       << "===RUNNINGSIMULATIONNo." << ++turn << "===" << endl;
  // Printoutgraphinformation
  double d = static_cast<double>(g.E()) / ((static_cast<double>(g.V()) * static_cast<double>(g.V()) - 1) / 2) * 100; // Calculaterealdensityreached
  cout << "Vertices:" << g.V() << endl;
  cout << "Edges:" << g.E() << "(density:" << d << "%)" << endl;
  cout << "Graph:" << endl;
  g.show();
  // Printoutshortestpathinformation
  list<char> v = g.vertices();
  cout << endl
       << "Vertices:" << v << endl;
  int reachVert = 0, sumPathSize = 0, avgPathSize = 0;
  ShortestPath sp(g);
  for (list<char>::iterator i = ++v.begin(); i != v.end(); ++i)
  {
    char src = v.front();
    char dst = (*i);
    list<char> p = sp.path(src, dst);
    int ps = sp.path_size(src, dst);
    if (ps != INFINIT)
      cout << "ShortestPath(" << src << "to" << dst << "):" << ps << "->" << p << endl;
    else
      cout << "ShortestPath(" << src << "to" << dst << "):" << "**UNREACHABLE**" << endl;
    if (ps != INFINIT)
    {
      reachVert++;       // Sumupreachednodes
      sumPathSize += ps; // Sumupshortestpathsfound
    }
  }
  // Calculateaverageshortestpathandprintitout
  if (reachVert != 0)
    avgPathSize = sumPathSize / reachVert;
  else
    avgPathSize = 0;
  cout << endl
       << "AVG Shortest Path Size (reachVert:" << reachVert << " -sumPathSize:" << sumPathSize << "): " << avgPathSize << endl;
}
//==============================================================================
// MainFunction
//==============================================================================
int main()
{
  MonteCarlo simulation;
  Graph g;
  // Createsagraphwith50nodes/density20%andthenrunsimulation
  g = simulation.randomGraph(50, 0.2, 1, 10);
  simulation.run(g);
  // Createsagraphwith50nodes/density40%andthenrunsimulation
  g = simulation.randomGraph(50, 0.4, 1, 10);
  simulation.run(g);
  return 0;
}
