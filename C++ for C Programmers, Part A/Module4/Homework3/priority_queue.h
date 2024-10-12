#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include "priority_queue_element.h"

#include <algorithm>
using namespace std;

class PriorityQueue
{
  friend ostream &operator<<(ostream &out, const PriorityQueue &pq);

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

ostream &operator<<(ostream &out, const vector<PriorityQueueElement> &pq)
{
  cout << pq.size() << " [ ";
  for (auto qe : pq)
  {
    cout << "(" << qe.getNode() << ":" << qe.getValue() << ") ";
  }
  cout << "]";
  return out;
}

ostream &operator<<(ostream &out, const PriorityQueue &pq)
{
  cout << pq.q;
  return out;
}

void PriorityQueue::initialize(size_t numNodes, Node start)
{
  q.clear();
  add(start, 0);
  for (size_t i{0}; i < numNodes; ++i)
  {
    if (i != size_t(start))
    {
      add(Node(i), MAX_DISTANCE);
    }
  }
}

void PriorityQueue::add(Node n, Distance d)
{
  q.push_back(PriorityQueueElement(n, d));
}

Node PriorityQueue::minPriority()
{
  if (isEmpty())
  {
    return NO_NODE;
  }
  Node n{q[0].getNode()};
  q.erase(q.begin());
#ifdef ENABLE_DEBUG
  cout << "q: " << q << endl;
#endif
  return n;
}

void PriorityQueue::changePriority(Node n, Distance d)
{
  for (auto &x : q)
  {
    if (x.node == n)
    {
      x.value = d;
      break;
    }
  }
  sort(q.begin(), q.end());
}

bool PriorityQueue::contains(PriorityQueueElement qe)
{
  for (auto x : q)
  {
    if (x.node == qe.node)
      return true;
  }
  return false;
}

void PriorityQueue::insert(PriorityQueueElement qe)
{
  if (!contains(qe))
    q.insert(q.begin(), qe);
}

#endif
