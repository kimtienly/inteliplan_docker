#!/usr/bin/env python3
'''Note: Currently this collision system is used to add to Exotica scene only'''

import numpy as np
import open3d as o3d
import rospy
import trimesh
import pyexotica as exo
import re
from reachability_graph import configuration

class Object:
    ''' Store info of each object '''

    def __init__(self, object_info, movable = False, mesh_type = True):
        self.loc = np.array(object_info['loc'])
        self.ori = np.array(object_info['ori']) # Quaternion
        self.size = np.array(object_info['size'][:3])
        self.type = object_info['type']
        self.movable = movable
        self.exo_mesh = None 
        self.tri_mesh = None
        self.mesh_type = mesh_type
        self.color= [1, 1, 1]
        self.generate_exo_mesh(object_info)
    
    def __repr__(self):
        return 'Object(location = {}, movable = {})'.format(self.loc, self.movable)

    def generate_exo_mesh(self, object_info):
        
        
        if "/" in object_info['type']: 
            mesh_file = object_info['type']
            # Reload file to get exotica mesh
            self.tri_mesh = trimesh.load(exo.Tools.parse_path(mesh_file))

            # Visualize loaded mesh
            # import matplotlib.pyplot as plt
            # trim = m
            # fig = plt.figure()
            # ax = fig.add_subplot(projection='3d')
            # ax.plot_trisurf(trim.vertices[:, 0], trim.vertices[:,1], trim.vertices[:,2], triangles=trim.faces);
            # plt.show()

            vertices = [self.tri_mesh.vertices[i] for i in self.tri_mesh.faces.flatten()]
            self.exo_mesh = exo.Mesh.createMeshFromVertices(vertices)
        
        elif object_info['type'] in configuration.avai_objects:
            if object_info['type'] == 'ssBox':
                size = object_info['size'][:3]
                self.exo_mesh = exo.Box(size[0], size[1], size[2])
 
        else:
            rospy.logerr('Object {} is not defined'.format(object_info['type']))
            return
        

class CollisionObjectWrapper:
    '''Store info of the map and implement fixed/movable objects'''

    def __init__(self):
        self.objects_list = []        

    def load_collision_objects(self, object_list, movable = False):
        for object_ in object_list:
            mesh_type = object_list[object_]['mesh_type']
            if mesh_type:
                self.objects_list.append(Object(object_list[object_], movable, mesh_type))    
            else:
                frames = self.read_graph_files(object_list[object_]['file'])
                for frame in frames:
                    self.objects_list.append(Object(frame, movable, mesh_type))

    def read_graph_files(self, filename):

        # Open the graph file
        with open(filename, 'r') as f:
            # Initialize a list to store the frame information
            frames = []           
        
            # Iterate over all lines in the file
            for line in f:

                if not line.strip():
                    continue
                             
                # Extract frame ID
                frame_id_match = re.search(r'Frame\s+(\w+)', line)
                if frame_id_match:
                    frame_id = frame_id_match.group(1)
                else:
                    frame_id = None
                if frame_id:
                    # Extract shape
                    shape_match = re.search(r'shape:(\w+)', line)
                    if shape_match:
                        shape = shape_match.group(1)
                    else:
                        shape = None

                    # Extract position
                    position_match = re.search(r'X=<T\s*t\(\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*\)', line)
                    if position_match:
                        position = [float(x) for x in position_match.groups()]
                    else:
                        relative_position_match = re.search(r'Q=<T\s*t\(\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*\)>', line)
                        if relative_position_match:
                            relative_position = [float(x) for x in relative_position_match.groups()]
                        else:
                            relative_position = [0, 0, 0]
                        ref_frame = re.search(r'Frame\s+\w+\s*\((\w+)\)\s*\{', line).group(1)
                        for tmp in frames:
                            if tmp['id'] == ref_frame:
                                ref_frame_position = tmp['loc']
                        position = (np.asarray(relative_position)+np.asarray(ref_frame_position)).tolist()


                    # Extract orientation
                    orientation = [0, 0, 0, 1]
                    orientation_match = re.search(r'd\(\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*(-?[\d\.]+)\s*(-?[\d\.]+)\)>', line)
                    if orientation_match:
                        orientation = [float(x) for x in orientation_match.groups()]

                    # Extract size
                    size_match = re.search(r'size=\[(.*?)\]', line)
                    if size_match:
                        size = [float(x) for x in size_match.group(1).split()]
                    else:
                        size = None

                    # Extract color
                    color_match = re.search(r'color=\[(.*?)\]', line)
                    if color_match:
                        color = [float(x) for x in color_match.group(1).split()]
                    else:
                        color = None


                    # Create a dictionary to store the frame information
                    
                    frame = {
                        'id': frame_id,
                        'type': shape,
                        'loc': position,
                        'ori': orientation,
                        'size': size,
                        'color': color,
                    }

                    # Add the frame to the list
                    frames.append(frame)
            return frames

if __name__ == '__main__':
    collision_scene = CollisionObjectWrapper()
    collision_scene.read_graph_files('/home/ktien/catkin_ws_2/src/reachability-aware-LGP/rhh-lgp/robotModels/tables_with_walls.g')