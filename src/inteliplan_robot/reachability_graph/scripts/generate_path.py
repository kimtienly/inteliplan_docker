#!/usr/bin/env python3

import numpy as np
import mpl_toolkits.mplot3d.axes3d as axes3D
import time
import rospy
from reachability_graph.path_planning import reachability_graph, collision_object
from reachability_graph.path_planning.maputils import *
from reachability_graph.robot import exotica_interface
from reachability_graph import configuration
import trimesh
import pyexotica as exo
from test_service.srv import graph_cost
from  reachability_graph.path_planning.rrtutils import Node
from geometry_msgs.msg import Pose, PoseArray
import matplotlib

DEBUG = False
if not DEBUG:
    matplotlib.use('agg')
    save_planned_path = False

plt = matplotlib.pyplot
class graph_service():
    def __init__(self):
        self.prm = None
        self.ax = None
        self.cache = {}
        self.cnt = 0

    def graph_callback(self, coordinates):

        # node = Node(np.array([coordinates.end_x, coordinates.end_y, coordinates.end_z]))
        # nearest_node = self.prm.tree.nearest(node)      
        # return nearest_node.cost
        start = [round(coordinates.start_x, 3), round(coordinates.start_y, 3), round(coordinates.start_z, 3) + 0.15]
        end = [round(coordinates.end_x, 3), round(coordinates.end_y, 3), round(coordinates.end_z, 3)+0.15]
        rospy.loginfo('Received request to find cost from {} to {}'.format(start, end))
        key = tuple(sorted([tuple(start), tuple(end)]))

        # If the distance is already calculated, return the cached value
        if not (key in self.cache):            
            path, cost, base_path = self.find_path(start, end)
            self.cache[key] = {}
            self.cache[key]['cost'] = cost
            self.cache[key]['path'] = PoseArray()
            max_cost = max(self.cache.values(), key=lambda x: x['cost'])['cost']
            print("MAX COST IS:", max_cost) 
            # print(path)
            poses = []
            # for pos in path:
            for pos in base_path:
                pose = Pose()
                pose.position.x = pos[0]
                pose.position.y = pos[1]
                pose.position.z = pos[2]
                poses.append(pose)
            self.cache[key]['path'].poses = poses
        print("SOLUTION IS:",self.cache[key]['cost'], len(self.cache[key]['path'].poses))
        return self.cache[key]['cost'], self.cache[key]['path'] 
        
    def find_path(self, start, goal):
        start_time = time.time()
        waypoints, min_cost, base_waypoints = self.prm.plan(start, goal, node_enhancement = True)
        if len(waypoints)>0:
            if DEBUG:
                self.prm.draw_path(self.ax, waypoints, color_ = 'r') # plot the waypoints
                plt.show()
            elif save_planned_path:
                self.prm.draw_path(self.ax, waypoints, color_ = 'r') # plot the waypoints
                self.ax.view_init(elev=33, azim=0)
                plt.savefig('path_from_graph/path_{}.pdf'.format(self.cnt))
                self.cnt+=1
            min_cost /= 10000000000000000000
            time_for_waypoints = np.round(time.time() - start_time, decimals=2, out=None)
            # print('Finish planning in {}s with {} steps'.format(time_for_waypoints, waypoints.shape))
            # print('Path cost: ',min_cost)
            rospy.loginfo('Found solution with path length = {}'.format(len(waypoints)))
        else:    
            if DEBUG:
                self.prm.draw_path(self.ax, [start,goal], color = 'r') # plot the waypoints
                plt.show()
            elif save_planned_path:
                self.ax.view_init(elev=33, azim=-0)
                plt.savefig('path_from_graph/failure_{}.pdf'.format(self.cnt))
                self.cnt+=1
            rospy.logerr('Failed to find path!')
            min_cost = 50
        return waypoints, min_cost, base_waypoints

    def run_graph_service(self):
        exoticaInt = None
        #region ROBOT DEFINITION
        
        # currentTransform = transform.KinovaTransformManager()
        # jointStateWriter = robot_interface.RobotInterface()
        rospy.loginfo("Registering Exotica interface.")
        exoticaInt = exotica_interface.ExoticaInterface()
        
        #endregion

        # add collision to exotica scene
        print('Adding collision scene..')
        collision_objects = collision_object.CollisionObjectWrapper()
        collision_objects.load_collision_objects(configuration.fixed_object_list_exotica)
        exoticaInt.addCollisionObject(collision_objects.objects_list)


        # create a figure
        fig = plt.figure()
        self.ax = fig.add_subplot(projection='3d')
        self.ax.set_xlim((configuration.map_bounds[0][0], configuration.map_bounds[0][1]))
        self.ax.set_ylim((configuration.map_bounds[1][0], configuration.map_bounds[1][1]))
        self.ax.set_zlim((configuration.map_bounds[2][0], configuration.map_bounds[2][1]))

        print('Building reachability graph..')
        mapobs = Map(collision_objects.objects_list, configuration.map_bounds, dim=3)
        mapobs.plotobs(self.ax) # plot obstacles

        rospy.init_node("reachability_graph")
        
        self.prm = reachability_graph.ReachabilityGraph(Map=mapobs, robot=exoticaInt, num_sample=200)
        s = time.perf_counter()
        self.prm.build_graph(node_enhancement=True)
        print(f"Reachability graph built in {(time.perf_counter()-s):.2f}s")
        if DEBUG:
            # self.prm.draw_graph(self.ax) # plot ramdom road map
            # plt.show()
            self.find_path([-0.39302945137023926, 2.9037177562713623, 0.7899999976158142], [0.0, 3.0, 0.7800000047683716])
            # [0.0, 3.0, 0.7200000047683716] to [-0.7424448728561401, 3.0351943969726562, 0.7899999976158142] 1/8
        else:
            # Create rosservice
            s = rospy.Service('reachability_graph_service', graph_cost, self.graph_callback)
            rospy.loginfo('Reachability graph and its service created!')
            
            rospy.spin()

if __name__ == "__main__":
    serv = graph_service()
    serv.run_graph_service()
