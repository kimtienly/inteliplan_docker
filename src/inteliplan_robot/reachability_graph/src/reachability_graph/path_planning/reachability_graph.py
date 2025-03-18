import numpy as np
import networkx as nx
from .rrtutils import *
import rospy
# np.random.seed(1)

class ReachabilityGraph():
    def __init__(self, Map, robot=None,
                 num_sample=200):
        self.map = Map
        self.num_sample = num_sample
        self.dim = 3
        self.robot = robot
        self.graph = nx.Graph()
        self.tree = Rtree(self.dim)
        self.node_enhancement = True
        self.enhanced_graph = nx.Graph()
        self.enhanced_tree = Rtree(self.dim)
        self.enhanced_node_count = 0

        self.points_for_debugging=[]
        self.edges_for_debugging=[]

    def build_graph(self, node_enhancement = True):
        '''build graph for multi-query'''
        self.node_enhancement = node_enhancement
        # Add nodes by sampling
        while self.tree.len < self.num_sample:
            new_node = Node(self.sample())
            if self.validate_IK(new_node):
                self.graph.add_node(new_node, weight = new_node.cost)
                self.tree.add(new_node)
                if node_enhancement:
                    new_node = self.enhance_node(new_node, sub_sample = 1)
                    self.enhanced_graph.add_node(new_node, weight = new_node.cost)
                    self.enhanced_tree.add(new_node)
                
        # Connect near neighbors       
        for node in self.tree.all():
            self.connect_neighbors(node, self.graph, self.tree, check_collision=False)
        for node in self.enhanced_tree.all():
            self.connect_neighbors(node, self.enhanced_graph, self.enhanced_tree, check_collision=False)

        if node_enhancement:
            print('Successfully enhanced {} nodes in total {} nodes'.format(self.enhanced_node_count, self.num_sample))

        # Remove nodes if no edges connected to it found
        # for graph_,tree_ in zip((self.graph, self.enhanced_graph),(self.tree, self.enhanced_tree)):
        #     node_to_remove = []
        #     for node in graph_.nodes:
        #         if len(graph_.edges(node)) <= 1: # 1 because neighbors considers the node itself
        #             node_to_remove.append(node)
        #     for node in node_to_remove:    
        #         graph_.remove_node(node)
        #         tree_.remove_node(node)

    def plan(self, start, goal, node_enhancement = True, return_base = False):
        def enhance_graph():
            # improve graph
            ind_to_override = []
            node_to_override = []
            for ind, node in enumerate([start_,goal_]):
                # number_of_nodes_to_add = 4
                number_of_nodes_to_add = 1
                # print(node," is considering to enhance before trying to plan, which connections=", len(self.enhanced_graph.edges(node))-1)
                if len(self.enhanced_graph.edges(node)) <= 1: # 1 means no edges, because neighbors considers the node itself
                    while number_of_nodes_to_add >= 0:
                        # print('radius ',0.03*number_of_nodes_to_add+0.03)
                        node_to_add = self.enhance_node(node, radius=0.03*number_of_nodes_to_add+0.02,sub_sample=2,  exclude_center=True)
                        # condition = self.robot.check_collision_free(node.q, node_to_add.q) 
                        # print('Node to add: {}'.format(node_to_add))
                        # if condition: # only process this new node if it is able to connect to the inspecting node
                        self.enhanced_graph.add_node(node_to_add, weight = node_to_add.cost)
                        self.enhanced_tree.add(node_to_add)        
                        neighbors = self.find_neighbors(node_to_add, self.enhanced_tree, 16)     
                        # neighbors = self.find_neighbors(node_to_add, self.enhanced_tree, 5)     
                        if self.connect_neighbors(node_to_add, self.enhanced_graph, self.enhanced_tree):
                            number_of_nodes_to_add -= 1
                            self.points_for_debugging.append(node_to_add.p)
                            connected_edges = [edge for edge in self.enhanced_graph.edges if node_to_add in edge]
                            self.edges_for_debugging+=connected_edges          
                        else:
                            rospy.logerr("The proposed node is not able to connect to the inspecting node {}".format(node))
                            

                    # print(node," now has connections=", len(self.enhanced_graph.edges(node))-1)
                    if len(self.enhanced_graph.edges(node)) <= 1:
                        ind_to_override.append(ind)
                        node_to_override.append(node_to_add)
            return ind_to_override, node_to_override

        print('Start planning')
        start_ = Node(start)
        goal_ = Node(goal)
             
        if not self.node_enhancement:
            node_enhancement = False
        # Check if there is a feasible joint in start and goal pose
        for node in [start_,goal_]:
            if not self.validate_IK(node, th=10000000000000000):
                rospy.logerr('Error: Failed to generate joint poses for start or goal position: {}'.format(node.p))
                pass
                # return None, None

        # Add start and goal to the built graph
        # self.enhanced_graph = self.graph
        # self.enhanced_tree = self.treesetConfiguration_qAll(t, waypoints(waypoint_ind));
        # if node_enhancement:
        #     self.enhanced_graph = self.enhanced_graph
        #     self.enhanced_tree = self.enhanced_tree
        for node in [start_, goal_]:
            # if not self.enhanced_tree.check_existence(node.p):
            #     rospy.logwarn("new node added: {}".format(node.p))
            self.enhanced_graph.add_node(node, weight = node.cost)
            self.enhanced_tree.add(node)
            # else:
            #     rospy.logwarn("node {} is already in the tree!".format(node.p))
        for node in [start_, goal_]:
            self.connect_neighbors(node, self.enhanced_graph, self.enhanced_tree, num_near_neighbor=21)
            connected_edges = [edge for edge in self.enhanced_graph.edges if node in edge]
            self.edges_for_debugging+=connected_edges   

        # self.graph = self.enhanced_graph
        # self.tree = self.enhanced_tree 
        # if node_enhancement:
        #     self.enhanced_graph = self.enhanced_graph
        #     self.enhanced_tree = self.enhanced_tree
        loop = 4

        final_base_path = []
        while True:               
            try:
                node_path = nx.dijkstra_path(
                    self.enhanced_graph, source=start_, target=goal_)
                length = nx.dijkstra_path_length(
                    self.enhanced_graph, source=start_, target=goal_)
                
                # Construct path from nodes
                final_path = []
                final_path.append(start)
                final_base_path.append(start_.q[:3])
                collision_free = True
                # for node in node_path:   
                for i in range(len(node_path)-1): 
                    # check path collision
                    if not self.robot.check_collision_free(node_path[i].q, node_path[i+1].q):
                        rospy.logerr("edge to remove: {} to {}".format(node_path[i], node_path[i+1]))
                        # self.enhanced_graph.remove_node(node)
                        # self.enhanced_tree.remove_node(node)
                        self.enhanced_graph.remove_edge(node_path[i], node_path[i+1])
                        # print("edge removed")
                        collision_free = False
                        # break                    
                    # print(node.cost)
                    final_path.append(node_path[i].p)
                    final_base_path.append(node_path[i].q[:3])
                if collision_free:
                    final_path.append(node_path[i+1].p)
                    final_base_path.append(node_path[i+1].q[:3])
                    final_path.append(goal)
                    final_base_path.append(goal_.q[:3])
                    if return_base:
                        return np.array(final_path), length, np.array(final_base_path)
                    else:
                        return np.array(final_path), length, np.array(final_path)
                print("path is not collision free.. replanning.. ")
            except Exception as e:
                rospy.logerr('Failed to generate path: {}'.format(e))
                if loop ==0:
                    break
                loop -= 1
            ind_to_override, node_to_override = enhance_graph()
            for i, ind in enumerate(ind_to_override):
                if ind == 0:
                    rospy.logerr("assigning new start node {} replacing the original {}".format(node_to_override[i],start_))
                    start_ = node_to_override[i]
                else:
                    rospy.logerr("assigning new goal node {} replacing the original {}".format(node_to_override[i],goal_))     
                    goal_ = node_to_override[i]

        rospy.logerr("SAD NEWS: Failed after all trials: {} to {}!".format(start, goal))
        return [], 0, []    

    
    def connect_neighbors(self, node, graph, tree,  num_near_neighbor = 16, check_collision = True):
        neighbors = self.find_neighbors(node, tree, num_near_neighbor)
        cnt=0
        # if len(list((neighbors)))!=0:
        connected = False # check if the node is connected to current sets
        while num_near_neighbor <=21:
        # while num_near_neighbor <=11:
            for neighbor in neighbors:
                
                if neighbor is not None:
                    # print('checking connection between ',node.p, neighbor.p)
                    # if (self.map.collision(node.p, neighbor.p) != True): # Cart interpolated path between node and neighbor
                    # start_base = np.asarray(node.q[:2].tolist()+[0.258])
                    # end_base = np.asarray(neighbor.q[:2].tolist()+[0.258])
                    # if (self.map.collision(start_base, end_base) != True):

                    if check_collision:
                        condition = self.robot.check_collision_free(node.q, neighbor.q)                    
                    else:
                        condition = True
                    
                    if condition:
                        weight = self.dist(node, neighbor) 
                        # print('weight: ', weight)
                        # print("node cost ",node.cost/2,neighbor.cost/2)
                        graph.add_edge(node, neighbor, weight=weight)
                        # if weight<1000:
                        #     graph.add_edge(node, neighbor, weight=weight+node.cost/2+neighbor.cost/2)
                        # else:
                        cnt+=1
                        connected = True
                    # print('connected: ', condition)
                    # else:
                    #     rospy.logwarn("collision ")
                  
            # else:
            #     rospy.logwarn("Cannot connect node {} to graph".format(node.p))
            # print('remove {} edge'.format(cnt))
            if not connected:
                rospy.logwarn("extending neighbor as no connection found ")
                num_near_neighbor += 5
                neighbors = self.find_neighbors(node, tree, num_near_neighbor, exclude=neighbors)
            else:
                break
        # print('Found {} connection to existing graph'.format(cnt))
        return connected

    def find_neighbors(self, node, tree, num_near_neighbor = 11, exclude = None):
         # 5 if excluding current nodes
        neighbors = tree.k_nearest(node, num_near_neighbor, exclude)
        return neighbors
        
    def dist(self, from_node, to_node):
        # Cartesian space
        # cart_dist = np.linalg.norm(from_node.p - to_node.p)
        # Configuration space
        conf_dist = np.linalg.norm(from_node.q - to_node.q)
        start_base = np.asarray(from_node.q[:2].tolist()+[0.2])
        end_base = np.asarray(to_node.q[:2].tolist()+[0.2])
        if (self.map.collision(start_base, end_base) == True):
            conf_dist+=10**18
        # return cart_dist + conf_dist
        return conf_dist

    def sample(self):
        # Sample random point inside boundaries
        sample = []
        for i in range(3):
            lower, upper = self.map.bounds[i]
            sample.append(np.random.uniform(low=lower, high=upper))
        return sample
        cart_dist = np.linalg.norm(from_node.p - to_node.p)
    def draw_path(self, ax, path, color_ = 'b'):
        '''draw the path if available'''
        ax.clear()
        self.draw_graph(ax)
        if path is None:
            print("path not available")
        else:
            ax.plot(*np.array(path).T, '-',
                    color=color_, zorder=7)
        if len(self.points_for_debugging)!=0:
            x_values, y_values, z_values = np.array(self.points_for_debugging).T
            ax.scatter3D(x_values, y_values, z_values, color='g', zorder=7)
            print("edges for debugging:", len(self.edges_for_debugging))
            for edge in self.edges_for_debugging:
                x1 = edge[0].p[0]
                x2 = edge[1].p[0]
                y1 = edge[0].p[1]
                y2 = edge[1].p[1]
                z1 = edge[0].p[2]
                z2 = edge[1].p[2]
                ax.plot([x1, x2], [y1, y2], [z1, z2], color='g', linewidth=1, alpha = 1)
        return ax
    
    def draw_graph(self, ax, scale_factor=1, node_enhancement = True):
        '''draw the graph if available'''
        self.map.plotobs(ax)
        xs = []
        ys = []
        zs = []
        color_ = 'b'
        graph_to_draw = [self.enhanced_graph]
        if (not self.node_enhancement) or (not node_enhancement):
            graph_to_draw = [self.graph]
        # for graph_ in [self.graph, self.enhanced_graph]:
        for graph_ in graph_to_draw:
            for node in list(graph_.nodes):
                xs.append(scale_factor*node.p[0])
                ys.append(scale_factor*node.p[1])
                zs.append(scale_factor*node.p[2])
            ax.scatter(xs, ys, zs, marker='o', s=1)

            for edge in list(graph_.edges):
                x1 = scale_factor*edge[0].p[0]
                x2 = scale_factor*edge[1].p[0]
                y1 = scale_factor*edge[0].p[1]
                y2 = scale_factor*edge[1].p[1]
                z1 = scale_factor*edge[0].p[2]
                z2 = scale_factor*edge[1].p[2]
                ax.plot([x1, x2], [y1, y2], [z1, z2], color=color_, linewidth=0.6, alpha = 1)
            color_ = 'g'
            

    def validate_IK(self, node, th = 2000):
        if (self.map.collision(node.p, node.p) == True):
            return False
        joints, cost = self.robot.getIK_withoutAngles(node.p) 
        cost /= 1
        # print(cost)
        # print(joints)
        node.add_joints(joints)
        node.add_cost(cost)
        if (cost>th):
            return False   
        return True

    def enhance_node(self, node, radius = 0.05, sub_sample = 5, exclude_center=False):
        ''' Choose node with best IK around a node by random sampling within radius or with gaussian sampling''' 

        lower, upper = [-radius,radius]
        if not exclude_center:
            best_cost = node.cost
            best_node = node
        else:
            best_cost = None
            best_node = None
        if radius <= 0:
            return_node = Node([node.p[0], node.p[1], node.p[2]+0.1])
            print('radius is ',radius, ' returning the hardcoded node ', return_node )
            print(self.validate_IK(return_node))
            return return_node
        for i in range(sub_sample):
            # uniform sampling
            # new_node = Node(node.p + np.random.uniform(low=lower, high=upper, size=(self.dim,)))
            
            # gaussian sampling
            cov_matrix = [[radius, 0, 0], [0, radius, 0], [0, 0, radius]]
            new_p = np.random.multivariate_normal(node.p, cov_matrix, size=1)[0] 
            new_node = Node(new_p)
            
            if self.validate_IK(new_node):
                if exclude_center: # if trying to find one connection with the current
                    condition = self.robot.check_collision_free(node.q, new_node.q) # check
                    if condition ==False:
                        rospy.logerr("not getting this node as it is unable to connect to the current node")
                else:
                    condition = True
                # joints, cost = self.robot.getIK_withoutAngles(new_node.p) 
                # if cost < best_cost:
                #     new_node.add_joints(joints)
                #     new_node.add_cost(cost)
                #     best_cost = cost
                #     best_node = new_node
                if condition:
                    if (best_cost is None) or (new_node.cost < best_cost):                        
                        best_cost = new_node.cost
                        best_node = new_node
                        if best_cost <= 100: # if found a good node, return
                            print("best_cost ",best_cost)
                            return best_node
                print("best_cost ",best_cost)
        if best_node is None:
            best_node = self.enhance_node(node, radius-0.02, sub_sample+5, exclude_center)
        
        # if node.cost != best_node.cost:
        #     self.enhanced_node_count +=1
        # else:
        #     print("best_cost ",best_cost)
        #     print("best_cost ",best_node)
        return best_node
    
    

