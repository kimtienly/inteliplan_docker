from rtree import index
import numpy as np
from matplotlib.pyplot import Rectangle

DEBUG = False

#obstacle map
class Map:
  def __init__(self, obstacle_list, bounds, path_resolution = 0.05, dim = 3):
    self.dim = dim
    self.path_res = path_resolution
    self.obstacles = obstacle_list
    self.bounds = bounds
    self.debug_points_collision = []

  def add_obstacle(self, obstacle):
    '''add new obstacle'''
    self.obstacles.append(obstacle)

  def collision(self,start, end):
    '''check if the sample point is inside an obstacle'''
    for p in [start, end]:
      for obs in self.obstacles:
        if obs.mesh_type:
          if obs.contains(np.asarray([p])):
            return True
        else:
          if self.is_point_inside_box(p, obs):
            if DEBUG:
              self.debug_points_collision.append(p)
            return True
          
    '''find if the ray between start and end collides with obstacles'''
    dist = np.linalg.norm(start-end)
    n = int(dist/self.path_res)
    points = np.linspace(start,end,n)
    for p in points:
      for obs in self.obstacles:
        if obs.mesh_type:
          if obs.contains(np.asarray([p])):
            return True
        else:
          if self.is_point_inside_box(p, obs):
            if DEBUG:
              self.debug_points_collision.append(p)
            return True
    return False

  def is_point_inside_box(self, point, obs):
    xmin, ymin, zmin = obs.loc - obs.size / 2
    xmax, ymax, zmax = obs.loc + obs.size / 2
    x, y, z = point
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax

  def plotobs(self, ax):
    '''plot all obstacles'''
    def plot_cube(position, orientation, width, depth, height):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        from scipy.spatial.transform import Rotation

        # Define cube vertices
        vertices = np.array([
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5]
        ])

        # Scale cube
        vertices[:, 0] *= width
        vertices[:, 1] *= depth
        vertices[:, 2] *= height

        # Rotate and translate cube
        r = Rotation.from_quat(orientation)
        rotated_vertices = np.dot(vertices, r.as_matrix().T)
        translated_vertices = rotated_vertices + position
        # Define cube faces
        faces = np.array([
            [0, 1, 2, 3],
            [1, 5, 6, 2],
            [5, 4, 7, 6],
            [4, 0, 3, 7],
            [0, 4, 5, 1],
            [3, 2, 6, 7]
        ])

        # Create cube patches
        cube_patches = []
        for face in faces:
            cube_patches.append(translated_vertices[face])

        # Create Poly3DCollection object
        cube_collection = Poly3DCollection(cube_patches, alpha=0.8)

        # Set face colors
        cube_collection.set_facecolor('blue')

        # Add collection to plot
        ax.add_collection3d(cube_collection)

    for obs in self.obstacles:
      if obs.mesh_type:
        ax.plot_trisurf(obs.tri_mesh.vertices[:, 0], obs.tri_mesh.vertices[:,1], obs.tri_mesh.vertices[:,2], triangles=obs.tri_mesh.faces)
      else:    
        if obs.type == 'ssBox':
          plot_cube(obs.loc, obs.ori, obs.size[0], obs.size[1], obs.size[2])
        else:
          print('Cannot plot obstacle due to undefined object type!')

    if DEBUG:
      xs = []
      ys = []
      zs = []
      for node in self.debug_points_collision:
          xs.append(node[0])
          ys.append(node[1])
          zs.append(node[2])
      ax.scatter(xs, ys, zs, marker='o', s=5, color = 'r')