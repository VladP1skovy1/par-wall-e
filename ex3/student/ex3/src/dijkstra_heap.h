/*
 * File name: dijkstra_heap.h
 * Date:      2008/07/30 11:52
 * Author:    Jan Faigl
 */

#ifndef __DIJKSTRA_HEAP_H__
#define __DIJKSTRA_HEAP_H__

/**
    When writing Dijkstra's algorithm, you will need some data structure
    where to store single nodes, order the nodes by some criterions (cost)
    and pick the best in each iteration. Therefore CHeap is here.
    It implements heap data structure, keeps data in order you will need
    them and when a node is updated, heap is sorted again.

    There is of course way how to implements Dijkstra without this class,
    but its recommended to use it since it can simplify your work a lot.


    How to create new heap?
    -----------------------
    CHeap is class template class with cost type not implicitly defined.
    This allows user to specify his own type of cost. For example the simpliest
    cases are integer or float(double) cost type:

        CHeap<int> myIntHeap(sizeOfHeap);
        CHeap<double> myDoubleHeap(sizeOfHeap);


    Useful methods for Dijkstra's algorithm
    ---------------------------------------
    At first, we should clarify, why to use the heap. The Dijkstra's algo-
    rith keep all the nodes in priority queue (heap) ordered by their dis-
    tance from the source (start node). In first iteration has only start
    node distance 0, for others it is infinity. The in each iteration the
    node with the highes priority (the lowest distance) is chosen and pro-
    cessed.
    For further details of the methods, see annotation below. This is only
    a simple guide that should show, which methods will be probably used
    in planning.

    add(n, cost) - Can be used when filling the heap with all the map cells
                   E.g. node_id = y * map_width + x;
                        heap.add(node_id, INT_MAX)
                    INT_MAX and other data type limits are defined in <climits>
    update(n, cost) - When new distance to the node is determined, it can be
                      easily updated using update(n, cost) method.
    getFirst() - Get the closest node (node with the lowest cost).
                 Be aware, that the node is removed from the heap
*/

namespace imr {
   namespace dijkstra {
      template<class Weight> class CHeap {
        private:
            /**
                This structure represents one node of the heap. Meaning of
                its variables is following
                IDX - index of the node in array
                node - identification of the node (e.g. id of cell in the map)
                cost - cost of the path from the beginning to this node
            */
            struct SHeap {
               long int IDX;
               long int node;
               Weight cost;
            };

            long int size;          // size of the heap (number of array's elements)
            SHeap * heap;           // array containing heap's nodes
            long int heapNumber;    // current "pointer" to the first free position in array

            /**
                Get parent node of the node i

                @param i - index of the node in the array
                @return index of parent node
            */
            long int get_parent(long int i);

            /**
                Get left neighbour node of the node i

                @param i - index of the node in the array
                @return index of parent node
            */
            long int get_left(long int i);

            /**
                Swap two nodes in the array

                @param i, j indeces of the nodes to be swapped
            */
            void swap(long int i, long int j);

        public:
            CHeap(long int size);

            ~CHeap(void);

            /**
                Add node to the heap. After addition the heap is sorted again.

                @param n    identification of the node
                @param cost cost of the path from the beginning to this node
            */
            void add(long int n, Weight cost);

            /**
                Get the first node of the heap. The first node is said to be the
                node with the lowest cost (top of the heap tree).
                The node is REMOVED from the heap.

                @return the first node identification (SHeap.node)
            */
            long int getFirst(void);

            /**
                Move a node down in the tree
            */
            void down(void);

            /**
                Update the cost of a node n and sort the heap again.

                @param n identification of the updated node
                @param cost new cost of the node
            */
            void update(long int n, Weight cost);

            /**
                Checks the heap consistency. This method IS NOT FULLY IMPLEMENTED yet
                and is not supposed to be used, unless user wants to implement it and
                check the heap.
            */
            void is_valid(long int n);

            /**
                Get index of a node n (specified by its ID)

                @param n identification of the node
                @return index of the node in the array
            */
            long int & getIDX(long int n);
        };

/**
    Include template implementation. This is important for compiler, because it
    could not work without access to complete template code.
*/

#include "dijkstra_heap.tpp"

   } //end namespace dijkstra
} //end namespace imr

#endif

/* end of heap.h */
