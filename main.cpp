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
//#include <CGAL/Orthtree_traits_2.h>

#include "QuadtreeNeighborFinder.h"

using namespace std;

typedef std::pair<long, long> longIntPair;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2 Point;
typedef K::Vector_2 Vector_2;

typedef CGAL::Quadtree<K, std::vector<Point>> Quadtree;
typedef CGAL::Orthtree_traits_2<K> Quadtree1;
typedef CGAL::Orthtrees::Preorder_traversal Preorder_traversal;
typedef Quadtree ::Node Node;


void findNeighborBottomLeftCellRightPerimeter(Node bottomLeft, vector<pair<string ,int>> &vectorOfBottomLeft,map<Node,string> &translator)
{
    if( bottomLeft.is_leaf() ) {
        vectorOfBottomLeft.emplace_back(make_pair(translator[bottomLeft],0));
        return ;
    }

    findNeighborBottomLeftCellRightPerimeter(bottomLeft[1],vectorOfBottomLeft,translator);
    findNeighborBottomLeftCellRightPerimeter(bottomLeft[3],vectorOfBottomLeft,translator);
}

void findNeighborBottomLeftCellUpPerimeter(Node bottomUp,vector<pair<string ,int>> &vectorOfBottomUp,map<Node,string> &translator)
{
    if( bottomUp.is_leaf() )
    {
        vectorOfBottomUp.emplace_back(make_pair(translator[bottomUp],2));
        return ;
    }
    findNeighborBottomLeftCellUpPerimeter(bottomUp[2],vectorOfBottomUp,translator);
    findNeighborBottomLeftCellUpPerimeter(bottomUp[3],vectorOfBottomUp,translator);
}

void findNeighborBottomRightCellLeftPerimeter(Node bottomUp,vector<pair<string ,int>> &vectorOfBottomRight,map<Node,string> &translator)
{
    if( bottomUp.is_leaf() )
    {
        vectorOfBottomRight.emplace_back(make_pair(translator[bottomUp],1));
        return ;
    }
    findNeighborBottomRightCellLeftPerimeter(bottomUp[0],vectorOfBottomRight,translator);
    findNeighborBottomRightCellLeftPerimeter(bottomUp[2],vectorOfBottomRight,translator);
}

void findNeighborBottomRightCellUpPerimeter(Node bottomUp,vector<pair<string ,int>> &vectorOfBottomUp,map<Node,string> &translator)
{
    if( bottomUp.is_leaf() )
    {
        vectorOfBottomUp.emplace_back(make_pair(translator[bottomUp],2));
        return ;
    }
    findNeighborBottomRightCellUpPerimeter(bottomUp[2],vectorOfBottomUp,translator);
    findNeighborBottomRightCellUpPerimeter(bottomUp[3],vectorOfBottomUp,translator);
}

void findNeighborTopLeftCellRightPerimeter(Node topLeft,vector<pair<string ,int>> &vectorOfTopUp,map<Node,string> &translator)
{
  if( topLeft.is_leaf() )
  {
      vectorOfTopUp.emplace_back(make_pair(translator[topLeft],0));
      return ;
  }
    findNeighborTopLeftCellRightPerimeter(topLeft[1],vectorOfTopUp,translator);
    findNeighborTopLeftCellRightPerimeter(topLeft[3],vectorOfTopUp,translator);
}

void findNeighborTopLeftCellBottomPerimeter(Node topLeft,vector<pair<string ,int>> &vectorOfTopBottom,map<Node,string>translator)
{
  if( topLeft.is_leaf() )
  {
      vectorOfTopBottom.emplace_back(make_pair(translator[topLeft],3));
      return ;
  }
    findNeighborTopLeftCellBottomPerimeter(topLeft[0],vectorOfTopBottom,translator);
    findNeighborTopLeftCellBottomPerimeter(topLeft[1],vectorOfTopBottom,translator);
}

void findNeighborTopRightCellLeftPerimeter(Node topRight,vector<pair<string ,int>> &vectorOfTopRight,map<Node,string> &translator)
{
    if( topRight.is_leaf())
    {
       vectorOfTopRight.emplace_back(make_pair(translator[topRight],1));
       return;
    }
    findNeighborTopRightCellLeftPerimeter(topRight[0],vectorOfTopRight,translator);
    findNeighborTopRightCellLeftPerimeter(topRight[2],vectorOfTopRight,translator);
}
void findNeighborTopRightCellBottomPerimeter(Node topRight,vector<pair<string ,int>> &vectorOfTopRight,map<Node,string> translator)
{
    if( topRight.is_leaf() )
    {
       vectorOfTopRight.emplace_back(make_pair(translator[topRight],3));
       return;
    }
    findNeighborTopRightCellBottomPerimeter(topRight[0],vectorOfTopRight,translator);
    findNeighborTopRightCellBottomPerimeter(topRight[1],vectorOfTopRight,translator);
}



int main() {
    vector<Point> pp;
    for( int i=0; i<200; i++ )
    {
        pp.emplace_back(10+i,10+i);
    }
    Quadtree quadtree(pp);
    quadtree.refine(5,70);
//    cout<<quadtree<<endl;
    cout<<"Node "<<endl;
//    cout<<quadtree<<endl;

    int currentNode = 01;
    vector<Node> neighbors;
    cout<<quadtree[0]<<endl;
    findLeafNeighbors_North(quadtree[0],neighbors);
    for( auto x: neighbors)
    {
      cout<<x<<endl;
    }
    neighbors.clear();
    findLeafNeighbors_South(quadtree[currentNode],neighbors);
    for( auto x: neighbors)
    {
      cout<<x<<endl;
    }
    neighbors.clear();
    findLeafNeighbors_East(quadtree[currentNode],neighbors);
    for( auto x: neighbors)
    {
      cout<<x<<endl;
    }
    neighbors.clear();
    findLeafNeighbors_West(quadtree[currentNode],neighbors);
    for( auto x: neighbors)
    {
      cout<<x<<endl;
    }
    neighbors.clear();
    cout<<"Hello "<<endl;
    cout<<quadtree<<endl;

//    Node cur = quadtree[currentNode].adjacent_node(10);
////    cout<<"Cur "<<cur<<endl;
//    cout<<"node0 "<<quadtree.root()<<endl;
//    cout<<"node1 "<<quadtree[1]<<endl;
//    cout<<"node2 "<<quadtree[2]<<endl;
//    cout<<"node3 "<<quadtree[3]<<endl;
//    cout<<" Right "<<quadtree[currentNode][0].adjacent_node(11).is_null()<<endl;
////    cout<<" Right "<<quadtree[currentNode].adjacent_node(01)<<endl;
//    cout<<" Down "<<quadtree[currentNode].adjacent_node(00).is_null()<<endl;
////    cout<<" UP "<<quadtree[currentNode].adjacent_node(100)<<endl;
//    cout<<" UP "<<quadtree[currentNode].adjacent_node(01)<<endl;
//    cout<<"  LEFT"<<quadtree[currentNode].adjacent_node(01)<<endl;
////    cout<<" Down "<<quadtree[currentNode].adjacent_node(10)<<endl;
////    cout<<"cur "<< cur<<endl;
//    cout<<"Good "<<endl;
////    cout<<cur <<" "<<cur[0]<<" "<<cur[1]<<" "<<cur[2]<<" "<<cur[3]<<endl;
////    cout<<cur.local_coordinates().to_ulong()<<endl;
////    cout<<cur.global_coordinates().at(0)<<endl;
////    cout<<cur.global_coordinates().at(1)<<endl;
//
//    vector<pair<string ,int>> vectorOfBottom;
//    vector<pair<string ,int>> vectorOfBottomUp;
//    map<Node,string> translator;
//    if( !cur.is_leaf() ) {
//        findNeighborBottomLeftCellRightPerimeter(cur[0][99], vectorOfBottom, translator);
//        findNeighborBottomLeftCellUpPerimeter(cur[0], vectorOfBottom, translator);
//        findNeighborBottomRightCellLeftPerimeter(cur[1], vectorOfBottom, translator);
//        findNeighborBottomRightCellUpPerimeter(cur[1], vectorOfBottom, translator);
//        findNeighborTopLeftCellRightPerimeter(cur[2], vectorOfBottom, translator);
//        findNeighborTopLeftCellBottomPerimeter(cur[2], vectorOfBottom, translator);
//        findNeighborTopRightCellLeftPerimeter(cur[3], vectorOfBottom, translator);
//        findNeighborTopRightCellBottomPerimeter(cur[3], vectorOfBottom, translator);
//    }
//    for( auto x: vectorOfBottom ) cout<<x.first<<" "<<x.second<<endl;

//    findNeighborBottomLeftCellUpPerimeter(quadtree.root()[0],vectorOfBottom,translator);
//    findNeighborBottomRightCellLeftPerimeter(quadtree.root()[1],vectorOfBottom,translator);
//    findNeighborBottomRightCellUpPerimeter(quadtree.root()[1],vectorOfBottom,translator);
//    findNeighborTopLeftCellRightPerimeter(quadtree.root()[2],vectorOfBottom,translator);
//    findNeighborTopLeftCellBottomPerimeter(quadtree.root()[2],vectorOfBottom,translator);
//    findNeighborTopRightCellLeftPerimeter(quadtree.root()[3],vectorOfBottom,translator);
//    findNeighborTopRightCellBottomPerimeter(quadtree.root()[3],vectorOfBottom,translator);

//    for( auto x: vectorOfBottom ) cout<<x.first<<" "<<x.second<<endl;

//    for (auto &node : quadtree.traverse<Preorder_traversal>())
//        std::cout << node <<" "<<node.local_coordinates().to_ulong()<< std::endl;
    return 0;
}
