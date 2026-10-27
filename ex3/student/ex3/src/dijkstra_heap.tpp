#include "dijkstra_heap.h"

#define HEAPNODE(i) heap[i].node
#define HEAPIDX(i) heap[i].IDX
#define HEAPCOST(i) heap[i].cost

using namespace imr::dijkstra;

template<class Weight> CHeap<Weight>::CHeap(long int size) : size(size){
   heap = 0;
   heapNumber = 0;
   heap = new SHeap[size];
   for (long int i = 0; i < size; i++) {
      HEAPIDX(i) = -1;
   }
}

template<class Weight> CHeap<Weight>::~CHeap(void) {
   delete[] heap;
}


template<class Weight> long int CHeap<Weight>::get_parent(long int i) {
   return (i-1) >> 1;
}

template<class Weight> long int CHeap<Weight>::get_left(long int i) {
   return (i<<1) + 1;
}


//#define HEAPCOST(i) nodes[HEAPNODE(i)].cost
template<class Weight> void CHeap<Weight>::swap(long int i, long int j) {
   //#define HEAP_SWAP(i,j) HEAPIDX(HEAPNODE(i)) = j; HEAPIDX(HEAPNODE(j)) = i; t = HEAPNODE(i); HEAPNODE(i) = HEAPNODE(j); HEAPNODE(j) = t; t = HEAPCOST(i); HEAPCOST(i) = HEAPCOST(j); HEAPCOST(j) = t;
   //#define HEAP_SWAP(i,j) HEAPIDX(HEAPNODE(i)) = j; HEAPIDX(HEAPNODE(j)) = i; t = HEAPNODE(i); HEAPNODE(i) = HEAPNODE(j); HEAPNODE(j) = t;
   long int t;
   HEAPIDX(HEAPNODE(i)) = j;
   HEAPIDX(HEAPNODE(j)) = i;
   t = HEAPNODE(i);
   HEAPNODE(i) = HEAPNODE(j);
   HEAPNODE(j) = t;
   Weight w = HEAPCOST(i);
   HEAPCOST(i) = HEAPCOST(j);
   HEAPCOST(j) = w;
}

template<class Weight> void CHeap<Weight>::add(long int n, Weight cost) {
   register long int parent;
   register long int index;
   HEAPNODE(heapNumber) = n;
   HEAPCOST(heapNumber) = cost;
   HEAPIDX(n) = heapNumber;
   index = heapNumber;
   parent = get_parent(index);
   while (index >= 1 && (HEAPCOST(parent) > HEAPCOST(index))) {
      swap(parent, index);
      index  = parent;
      parent = get_parent(index);
   }
   heapNumber++;
}

template<class Weight> long int CHeap<Weight>::getFirst(void) {
   long int ret;
   if (heapNumber == 0) {
      ret = -1;
   } else {
      ret = HEAPNODE(0);
      heapNumber--;
      HEAPNODE(0) = HEAPNODE(heapNumber);
      HEAPCOST(0) = HEAPCOST(heapNumber);
      HEAPIDX(HEAPNODE(0)) = 0;
      down();
   }
   return ret;
}

template<class Weight> void CHeap<Weight>::down(void) {
   register long int index;
   register long int hl, hr;
   long int best ;
   index = 0;
   hl = get_left(index);
   if (hl >= heapNumber) {
   } else {
      while(hl < heapNumber) {
         hr = hl+1;
         if (HEAPCOST(index) > HEAPCOST(hl)) {
            best = hl;
         } else {
            best = index;
         }
         if (hr < heapNumber && HEAPCOST(best) > HEAPCOST(hr)) {
            best = hr;
         }
         if (best != index) { // lower value found
            swap(index, best);
            index = best;
            hl = get_left(index);
         } else {
            break;
         }
      }
   }
   //check_heap(0, heap, nodes);
}

template<class Weight> void CHeap<Weight>::update(long int n, Weight cost) {
   register long int index;
   register long int parent;
   index = HEAPIDX(n);
   HEAPCOST(index) = cost;
   if (index < heapNumber) {
      parent = get_parent(index);
      while (index >= 1 && HEAPCOST(index) < HEAPCOST(parent)) {  //swap with parent
         swap(index, parent);
         index = parent;
         parent = get_parent(index);
      }
   }
   // check_heap(0, heap, nodes);
}

template<class Weight> void CHeap<Weight>::is_valid(long int n) {
   long int l = 2*n+1;
   long int r = 2*n+2;
   if (l < heapNumber) {
      if (HEAPCOST(l) < HEAPCOST(n)) {
         //TODO assert
         /*	 fprlong intf(stderr, "HEAP PROPERTY VIOLENCE l:%d n:%d cost %d %d\n", l, n,
             HEAPCOST(l), HEAPCOST(n));
             */
      } else {
         is_valid(l);
      }
   }
   if (r < heapNumber) {
      if (HEAPCOST(r) < HEAPCOST(n)) {
         //TODO assert
         // fprlong intf(stderr, "HEAP PROPERTY VIOLENCE r:%d n:%d\n", r, n);
      }
      is_valid(r);
   }
}

template<class Weight> long int & CHeap<Weight>::getIDX(long int n) {
   return HEAPIDX(n);
}
