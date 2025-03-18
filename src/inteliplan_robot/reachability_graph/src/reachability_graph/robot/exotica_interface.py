#!/usr/bin/env python3

import rospy
import numpy as np
import math
from reachability_graph import configuration
import open3d as o3d # removing this causes munmap_chunk error 
import pyexotica as exo
from time import perf_counter
from reachability_graph.path_planning import collision_object 
from pyexotica.tools import *
np.random.seed(1)

class ExoticaInterface:
    def __init__(self):

        rospy.loginfo("Setting up the Exotica instance")
        exo.Setup.init_ros()

        #region ik solver
        self.ik_solver = exo.Setup.load_solver(configuration.ikPlanner)
        self.ik_problem = self.ik_solver.get_problem()
        self.ik_scene = self.ik_problem.get_scene()
        self.ik_taskMaps = self.ik_problem.get_task_maps()
        #endregion

        self.joint_limits = self.ik_scene.get_kinematic_tree().get_joint_limits()
        self.joint_limits = (np.asarray(self.joint_limits))/5 # avoid contorted poses
        self.joint_limits[0]=[-10,10]
        self.joint_limits[1]=[-10,10]

        self.Debug = False
        # print(self.ik_scene.get_controlled_joint_names())

    def __del__(self):
        print("Deleting scene, problem and solver.")
        del self.ik_scene
        del self.ik_problem
        del self.ik_solver


    def addCollisionObject(self, object_list):
        def displayCollisionScene(scene):
            input('To view collision scene, please run "meshcat-server" in a separate terminal. Enter when finished!')
            vis = exo.VisualizationMeshcat(scene, 'tcp://127.0.0.1:6000')
            vis.delete()  # Clear existing scene
            vis.display_scene()  # Display this scene
            print('Scene visualization is available on http://127.0.0.1:7000/static/')

        for object_ in object_list: # list of [Object] type
            mesh = object_.exo_mesh
            color = object_.color
            self.ik_scene.add_object_to_environment(name="scene", color=(color[0], color[1], color[2], 1.0),
                    shape=mesh,transform=exo.KDLFrame(object_.loc.tolist()+object_.ori.tolist()))
         
        # displayCollisionScene(self.ik_scene)
    
    def check_collision_free(self, q1, q2, num_subsamples=200):
        # print(check_trajectory_continuous_time(self.ik_scene, np.asarray([q1,q2])))
        # print(check_whether_trajectory_is_collision_free_by_subsampling(self.ik_scene,np.asarray([q1,q2]),num_subsamples=num_subsamples))
        # return check_whether_trajectory_is_collision_free_by_subsampling(self.ik_scene,np.asarray([q1,q2]),num_subsamples=num_subsamples)
        return (check_trajectory_continuous_time(self.ik_scene, np.asarray([q1,q2])) or check_whether_trajectory_is_collision_free_by_subsampling(self.ik_scene,np.asarray([q1,q2]),num_subsamples=num_subsamples))


    def getIK_withoutAngles(self, ee_goal):
        """Returns an inverse kinematics solution (joint configuration) given the 6D end-effector goal (in world frame) and a start configuration."""
        def randomize_joint_values(limits):
            n = limits.shape[0]
            joint_values = []
            for i in range(n):
                lower_limit, upper_limit = limits[i]
                joint_value = np.random.uniform(lower_limit, upper_limit)
                joint_values.append(joint_value)
            return joint_values
        self.ik_problem.cost.set_goal('Position', ee_goal[:3])
        self.ik_problem.set_rho('Angle', 0)

        cnt_trial = 0
        best_cost = 0
        max_trials = 6 # working on 1/Aug/2023
        # max_trials = 8
        s = perf_counter()
        while cnt_trial<max_trials:
            random_start = randomize_joint_values(self.joint_limits)
            self.ik_problem.start_state = random_start + [0]*21
            # problem.start_state = random_start[:3]+[0]*45
            q = self.ik_solver.solve()[0]

            if best_cost==0 or best_cost>self.ik_problem.get_scalar_cost():
                best_cost = self.ik_problem.get_scalar_cost()
                q_best = q
                if best_cost < 1:
                    break 
            cnt_trial+=1
        e = perf_counter()
        solution = q_best
        current_cost = self.ik_problem.get_scalar_cost()


        return solution,current_cost

    def getIK_withAngles(self, q_start:np.array, ee_goal:np.array):
        """Having implemented EffAxisAlignment, we now need something to convert angles into a vector to load them into the EffAxisAlignment Angle"""
        direction = [
            -math.sin(ee_goal[5]) * math.cos(ee_goal[3]),
            math.cos(ee_goal[5]) * math.cos(ee_goal[3]),
            math.sin(ee_goal[3])]
        self.ik_taskMaps['Angle'].set_direction('j1n6s300_end_effector', direction)
        rospy.loginfo("\tDirection " + str(direction))

        return self.getIK(q_start, ee_goal)


if __name__ == '__main__':
    rospy.init_node('exotica_interface')
    exoticaInt = ExoticaInterface()
    
    for elem in exoticaInt.ik_scene.get_controlled_joint_names():
        print(elem)

    desired_ee = [-1, 1, 3]
    print(exoticaInt.getIK_withoutAngles(desired_ee))