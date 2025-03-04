#! /usr/bin/env python3
""" Common manipulation collision avoidance functionality.
"""

import os.path
import rospy

from std_srvs.srv import Empty
from manipulation.srv import GetReconstruction

from hsrb_interface import robot as _robot

_robot.enable_interactive()


class CollisionWorld:
    """
    Context manager wrapper around global_collision_world.
    """

    def __init__(self, collision_world, whole_body):
        self.collision_world = collision_world
        self.whole_body = whole_body

    def __enter__(self):
        # Enable robot collision checking
        self.whole_body.collision_world = self.collision_world

    def __exit__(self, type, value, traceback):
        # Disable robot collision checking
        self.whole_body.collision_world = None

    @staticmethod
    def empty(whole_body):
        """
        Return an empty CollisionWorld, for bypassing collision checking.
        """
        return CollisionWorld(None, whole_body)


class CollisionMapper:
    """
    This class is an interface to collision mapping on the robot.
    This is not intended to be used asynchronously, but avoids being broken by allowing
    asynchronous access by nodes to the robot's collision map.
    """

    ROBOT_STL_STORAGE_DIR = "/etc/opt/tmc/robot/stl"
    BACKUP_STL_STORAGE_DIR = "/tmp"

    def __init__(self, robot, stl_storage_dir=None):
        self.robot = robot
        self.global_collision_world = self.robot.try_get("global_collision_world")
        self.whole_body = self.robot.try_get("whole_body")

        rospy.loginfo(
            "%s: Waiting for octomap reset service..." % self.__class__.__name__
        )
        rospy.wait_for_service("/octomap_server/reset")
        self.reset_service = rospy.ServiceProxy("/octomap_server/reset", Empty)

        rospy.loginfo(
            "%s: Waiting for reconstruction service..." % self.__class__.__name__
        )
        rospy.wait_for_service("/GetReconstruction")
        self.reconstruction_service = rospy.ServiceProxy(
            "/GetReconstruction", GetReconstruction
        )

        if stl_storage_dir is not None:
            self.stl_storage_dir = stl_storage_dir
        else:
            print("self.ROBOT_STL_STORAGE_DIR = {0}".format(self.ROBOT_STL_STORAGE_DIR));
            if os.path.isdir(self.ROBOT_STL_STORAGE_DIR):
                print("Saving the collision map file into {0}".format(self.ROBOT_STL_STORAGE_DIR));
                self.stl_storage_dir = self.ROBOT_STL_STORAGE_DIR
            else:
                print("Saving the collision map file into {0}".format(self.BACKUP_STL_STORAGE_DIR));
                self.stl_storage_dir = self.BACKUP_STL_STORAGE_DIR

    def reset_collision_map(self):
        """
        Clears all objects from the HSR's collision map and the octomap server node's map.
        """
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        # Clear everything in global collision map
        self.global_collision_world.remove_all()

    def get_converted_octomap(self, external_bb, crop_bbs, stl_path):
        """
        Requests the octomap_to_reconstruction node to build an STL mesh from the octomap
        produced by octomap_server.
        Args:
            external_bb: external BoundingBox that bounds the STL mesh to be created
            crop_bbs: list of BoundingBox to crop out of the map (e.g. covering an object)
            stl_path: path to save the STL mesh file to
        Returns: success flag returned from octomap_to_reconstruction node
        """
        try:
            resp = self.reconstruction_service(external_bb, crop_bbs, stl_path)
            return resp.flag

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def build_collision_world(self, external_bounding_box, crop_bounding_boxes=None):
        """
        Add the robot's 3D occupancy map of a specified area to the HSR collision world.
        Args:
            external_bb: external BoundingBox that bounds the STL mesh to be created
            crop_bbs: list of BoundingBox to crop out of the map (e.g. covering an object)
        """

        # STL path to save to
        filename = "collision_mesh_" + str(rospy.get_time()).replace(".", "-") + ".stl"
        stl_path = os.path.join(self.stl_storage_dir, filename)

        crop_bbs = [] if crop_bounding_boxes is None else crop_bounding_boxes

        # Reset reconstruction
        self.reset_collision_map()

        # Wait for map to populate
        # TODO confirm that Octomap server is actually building the map during this sleep
        rospy.sleep(3)

        # Get and return collision map generated over last 3s
        flag = self.get_converted_octomap(external_bounding_box, crop_bbs, stl_path)

        # Add the collision map (from octomap) to the global collision world
        if flag:
            self.add_map_to_global_collision_world(stl_path)
        else:
            rospy.logwarn(
                "%s: Octomap reconstruction node returned an empty collision mesh."
                % (self.__class__.__name__)
            )

        return CollisionWorld(self.global_collision_world, self.whole_body)

    def add_map_to_global_collision_world(self, stl_path):
        """
        Add an STL mesh to the HSR's global collision world.
        """
        self.global_collision_world.add_mesh(stl_path, frame_id="map", timeout=0.0)
        return
