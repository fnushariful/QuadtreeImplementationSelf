//
// Created by FNU Shariful on 6/15/22.
//

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>
#include <fstream>
#include <boost/functional/hash.hpp> // hashing pairs
#include <boost/heap/fibonacci_heap.hpp> // ordering

#include <CGAL/boost/iterator/counting_iterator.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>
#include <fstream>
#include <boost/functional/hash.hpp> // hashing pairs
#include <boost/heap/fibonacci_heap.hpp> // ordering

#include <CGAL/boost/iterator/counting_iterator.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Kernel/global_functions.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Vector_2.h>
#include <CGAL/spatial_sort.h>
#include <CGAL/Spatial_sort_traits_adapter_2.h>
#include <CGAL/Quadtree.h>

#ifndef QUADTREEIMPLEMENTATION_QUADTREENEIGHBORFINDER_H
#define QUADTREEIMPLEMENTATION_QUADTREENEIGHBORFINDER_H

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2 Point;
typedef K::Vector_2 Vector_2;

typedef CGAL::Quadtree<K, std::vector<Point>> Quadtree;
typedef CGAL::Orthtree_traits_2<K> Quadtree1;
typedef CGAL::Orthtrees::Preorder_traversal Preorder_traversal;
typedef Quadtree ::Node Node;

using namespace std;

void findLeafNeighbors_North(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_North(cur[0], neighbors);
  findLeafNeighbors_North(cur[1], neighbors);
}

void findLeafNeighbors_West(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_West(cur[3], neighbors);
  findLeafNeighbors_West(cur[1], neighbors);
}

void findLeafNeighbors_South(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_South(cur[2], neighbors);
  findLeafNeighbors_South(cur[3], neighbors);
}

void findLeafNeighbors_SE(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_SE(cur[2], neighbors);
}

void findLeafNeighbors_SW(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_SW(cur[3], neighbors);
}

void findLeafNeighbors_East(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_East(cur[0], neighbors);
  findLeafNeighbors_East(cur[2], neighbors);
}

void findLeafNeighbors_NE(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_NE(cur[0], neighbors);
}

void findLeafNeighbors_NW(Node cur, vector<Node> &neighbors){
  if(cur.is_leaf()){
    neighbors.emplace_back(cur);
    return;
  }
  findLeafNeighbors_NW(cur[1], neighbors);
}

#endif // QUADTREEIMPLEMENTATION_QUADTREENEIGHBORFINDER_H
