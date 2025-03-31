#!/usr/bin/env python3
'''
This is the feasibility module to verify the robot ability to reach an end-effector.
'''
import rospy
import time
import rospy
from reachability_graph.path_planning import reachability_graph, collision_object
from reachability_graph.path_planning.maputils import *
from reachability_graph.robot import exotica_interface
import matplotlib
import tf2_ros

class FeasibilitySubscriber:
    def __init__(self, scene_file, map_bounds, DEBUG = False):
        rospy.loginfo("FeasibilitySubscriber -- initialised")
        self.prm = None
        self.ax = None
        self.cache = {}
        self.cnt = 0
        self.DEBUG=DEBUG
        if not self.DEBUG:
            matplotlib.use('agg')

        self.plt = matplotlib.pyplot
        self.build_reachability_graph(scene_file,map_bounds)
        self.tfBuffer = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.tfBuffer)

    def graph_call(self, start,end):
        rospy.loginfo('Received request to find cost from {} to {}'.format(start, end))
        key = tuple(sorted([tuple(start), tuple(end)]))

        # If the distance is already calculated, return the cached value
        if not (key in self.cache):            
            result = self.find_path(start, end)
            self.cache[key] = result
        return self.cache[key]
        
    def find_path(self, start, goal):
        waypoints, min_cost, base_waypoints = self.prm.plan(start, goal, node_enhancement = True)
        if len(waypoints)>0:
            if self.DEBUG:
                self.prm.draw_path(self.ax, waypoints, color_ = 'r') # plot the waypoints
                self.plt.show()
            rospy.loginfo('Found solution with path length = {}'.format(len(waypoints)))
            return True
        else:    
            rospy.logerr('Failed to find path!')
            return False

    def build_reachability_graph(self, scene, map_bounds):
        exoticaInt = None
        #region ROBOT DEFINITION
        
        rospy.loginfo("Registering Exotica interface.")
        exoticaInt = exotica_interface.ExoticaInterface()
        
        #endregion

        # add collision to exotica scene
        print('Adding collision scene..')
        collision_objects = collision_object.CollisionObjectWrapper()
        collision_objects.load_collision_objects(scene)
        exoticaInt.addCollisionObject(collision_objects.objects_list)

        # create a figure
        fig = self.plt.figure()
        self.ax = fig.add_subplot(projection='3d')
        self.ax.set_xlim((map_bounds[0][0], map_bounds[0][1]))
        self.ax.set_ylim((map_bounds[1][0], map_bounds[1][1]))
        self.ax.set_zlim((map_bounds[2][0], map_bounds[2][1]))

        print('Building reachability graph..')
        mapobs = Map(collision_objects.objects_list, map_bounds, dim=3)
        mapobs.plotobs(self.ax) # plot obstacles
        
        self.prm = reachability_graph.ReachabilityGraph(Map=mapobs, robot=exoticaInt, num_sample=50)
        s = time.perf_counter()
        self.prm.build_graph(node_enhancement=False)
        print(f"Reachability graph built in {(time.perf_counter()-s):.2f}s")
        if self.DEBUG:
            self.prm.draw_graph(self.ax) # plot ramdom road map
        # else:
        #     # Create rosservice
        #     # s = rospy.Service('reachability_graph_service', graph_cost, self.graph_callback)
        #     rospy.loginfo('Reachability graph and its service created!')
            
        #     rospy.spin()

    def get_score(self, goal):
        transf = self.tfBuffer.lookup_transform('map','hand_palm_link', rospy.Time(0),rospy.Duration(4.0))
        trans = [transf.transform.translation.x,transf.transform.translation.y,transf.transform.translation.z]
        if goal is None:
            return str(0)
        else:
            result=self.graph_call(trans,goal)
            return str(int(result))
        
if __name__=='__main__':
    rospy.init_node('feasibility_subscriber')
    scene = {'scene':{'file':'../worlds/room.g', 'mesh_type': False}}
    map_bounds = [[-3, 3],[-2, 2],[0.5, 1.3]] # limits on map dimensions
    detection_sub = FeasibilitySubscriber(scene, map_bounds, DEBUG=False)
    print(detection_sub.get_score([1,1,1])) # this returns 1 as reachable
    print(detection_sub.get_score([3,1,1])) # this returns 0 as unreachable