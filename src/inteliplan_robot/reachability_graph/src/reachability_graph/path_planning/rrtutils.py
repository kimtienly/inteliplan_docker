from rtree import index
import numpy as np

class Node:
    def __init__(self, coords):
        self.p = np.array(coords)
        self.parent = None
        self.cost = np.inf
        self.q = None
        
    def __len__(self):
        return len(self.p)

    def __getitem__(self, i):
        return self.p[i]

    def __repr__(self):
        return 'Node({}, {})'.format(self.p, self.cost)
    
    def add_joints(self,q):
        self.q = q
    
    def add_cost(self,cost):
        self.cost = cost

# Rtree to store Nodes
class Rtree:
    def __init__(self, dim):
        self.dim = dim
        self.node_list = []
        self.idx = self.get_tree(dim)
        self.len = 0

    @staticmethod
    def get_tree(dim):
        '''Initialise the tree'''
        p = index.Property()
        p.dimension = dim
        p.dat_extension = 'data'
        p.idx_extension = 'index'
        return index.Index(properties=p)

    def add(self, new_node):
        '''add nodes to tree'''
        self.node_list.append(new_node)
        self.idx.insert(self.len, (*new_node.p,))
        self.len += 1

    def k_nearest(self, node, k, exclude = None):
        '''Returns k-nearest nodes to the given node'''
        near_ids = self.idx.nearest((*node.p,), k)
        for i in near_ids:
            if exclude:
                for node in exclude:
                    if node == self.node_list.p:
                        continue
            yield self.node_list[i]

    def nearest(self, node):
        '''Returns nearest node to the given node'''
        near_ids = self.idx.nearest((*node.p,), 1)
        id = list(near_ids)[0]
        return self.node_list[id]

    def all(self):
        return self.node_list
      
    def remove_node(self, node):
        '''Remove a node from the tree'''
        # find the index of the node in the node_list
        node_index = self.node_list.index(node)
        
        # remove the node from the node_list
        self.node_list[node_index] = None
