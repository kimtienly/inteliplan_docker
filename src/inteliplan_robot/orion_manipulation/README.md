# orion\_manipulation
Repo for packages related to manipulating objects and opening doors etc.
Currently maintained by Kim Tien Ly.

<img src="/doc/img/header.png" alt="Header: object placement, object segmentation, and things going wrong" width="1000"/>

## Build / Installation
* See the Wiki for TMC package [installation](https://github.com/ori-orion/orion-documentation/wiki/Installing-HSR-Packages-and-Simulator-Locally) and ORIon package [initial environment setup, prerequisite installation, and building](https://github.com/ori-orion/orion-documentation/wiki/Installing-ORIon-Packages-Locally).
* Find manipulation installation guide [here](https://github.com/ori-orion/orion-documentation/wiki/Manipulation-Setup)
* Clone required ORIon packages:
``` 
git clone git@github.com:ori-orion/orion_actions.git
git clone git@github.com:ori-orion/orion_speech_recognition.git
```
* GPG and GPD are used for (optional) object grasp pose synthesis. Without grasp pose synthesis enabled, the HSR will choose a grasp which is hard coded relative to the target tf.

## Running a manipulation demo in simulation:
This simulation demo works with only manipulation components running. If you want to run it on the real robot, you should run perception's tf publisher node or use the AR tags and [`tmc_stereo_marker_recognizer`](https://docs.hsr.io/hsr_develop_manual_en/reference/packages/tmc_stereo_marker_recognizer/PKGDOC.html?highlight=ar%20marker#ros-interface), to ensure tfs are published for manipulation to use. You can use Rviz to display the TF tree to confirm the name of the TF and that it is located correctly.

* `roslaunch orion_simulation manipulation_test_sim.launch`
  * Starts the Gazebo simulation. This manipulation test environment includes several static tf publishers. These are useful for testing: for example, the `target` tf is placed in the same way on the glass jar as if perception image recognition had published that tf. Note that all HSR Gazebo worlds vibrate at high frequency (?) and objects will randomly wander over time. If you leave the simulator running for ~5 minutes, the `target` tf will no longer be on the glass jar.
  * This launchfile also starts a manipulation-specific rviz setup, with visualisation of manipulation systems.
  
* `roslaunch manipulation manipulation_servers.launch`
  * Starts up manipulation system basic components, including pick_up_object and put_object_on_surface actions.

* Watch the Gazebo and Rviz windows for the next two commands, which make the robot do things.

* `rosrun manipulation pick_up_object_client.py target`
  * When run, the robot should successfully pick up the glass jar that it spawns facing.
  
* Use Rviz goal pose tool to turn the robot 180 degrees so it faces the worktop.

* `rosrun manipulation put_object_on_surface.py surface_placement_l`
  * When run, the robot should successfully place the glass jar at the specified tf on the worktop.

## manipulation Package
* This repo enables the HSR to perform manipulation tasks such as picking up an object, opening a door, or giving the object to a human operator via a set of action servers.
* Nodes/scripts in this package are written in Python

### Pick up object (example nodes/data flow for a single action):

<img src="/doc/img/manipulation_data_flow.png" alt="Data Flow Diagram" width="500"/>
This is a data process flow illustration of `pick_up_object_server.py`. Similar to all updated manipulation action servers, it makes use of point cloud input from the Xtion RGBD camera for online collision mapping. This feeds into an OctoMap server which provides occupancy estimation collision maps.

### Collision mapping

<img src="/doc/img/collision_mesh.png" alt="Collision mesh illustration" width="400"/>

3D occupancy maps are maintained by `octomap_server`, using input from the Xtion RGBD camera. When carrying out manipulation actions, we:
* Get an occupancy map from `octomap_server`,
* Crop it (to make the map smaller and remove areas we want to ignore - e.g. crop out the object we want to pick up, lest the robot try to avoid touching the object),
* Convert it to a binary .STL mesh (this is the fastest method to get a complex object collision world representation),
* Insert the STL mesh in the correct location into the HSR's internal collision mapping system.

### Grasp pose synthesis
* Grasp pose components are launched by `manipulation grasp_synthesis.launch` (included in `manipulation_servers.launch`).
* See the flow diagram above. The `object_segmentation_node` node is responsible for extracting a point cloud of the object and passing it to GPD/GPG ROS nodes which produce the grasp candidates.
* The pick_up_object action server loops over the generated grasp candidates, highest score first, and chooses the first that HSRB motion planning can plan a collision-free path to.

### Coordinate frames
* The most important coordinate frame is the wrist-hand joint coordinate frame, which we use to specify where to move the end effector to:

<img src="/doc/img/coord_frame.png" alt="Hand-palm link coordinate system" width="400"/>

Red = x, green = y, blue = z.

## point\_cloud\_filtering Package
* This package is used internally by manipulation for isolating, segmenting and clustering point clouds in order to determine grasp poses.
* It processes point cloud data so is written in C++ for speed.

### Surface segmenter
* `surface_segmentation_node` provides the `/detect_surface` node.
* Surface detection is used to detect surfaces that objects are placed on / should be placed on, detecting the floor level, and detecting plane door surfaces etc.

### Object segmenter
* `object_segmentation_node` provides the `/segment_object` node.
* This is a specialised version of the surface segmenter, which takes the points above the detected surface and clusters them to find the target object.

## Action server status and TODOs:
- ReceiveObjectFromOperator.action
    - [x] `receive_object_from_operator_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- GiveObjectToOperator.action
    - [x] `receive_object_from_operator_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- PickUpObject.action
    - [x] `pick_up_object_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [x] Tested on real robot
- PourInto.action
    - [x] `pour_into_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- Follow.action
    - [x] `follow_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PointToObject.action
    - [x] `point_to_object.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PutObjectOnSurface.action
    - [x] `put_object_on_surface_server.py` updated to new inheritance structure
    - [x] Tested in simulation
    - [ ] Tested on real robot
- PlaceObjectRelative.action
    - [ ] `place_object_relative_server.py` updated to new inheritance structure. Should inherit from PutObjectOnSurface.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenDrawer.action
    - [ ] `open_drawer_server.py` updated to new inheritance structure. Should inherit from OpenDoor.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- CloseDrawer.action
    - [ ] `close_drawer_server.py` updated to new inheritance structure. Should inherit from OpenDrawer.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenDoor.action
    - [ ] `open_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenFurnitureDoor.action
    - [ ] Creation of action server `open_furniture_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- CloseFurnitureDoor.action
    - [ ] Creation of action server `close_furniture_door_server.py` updated to new inheritance structure
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- OpenBinLid.action
    - [ ] `open_bin_lid_server.py` updated to new inheritance structure
    - [ ] Rewritten to be less hard-coded and reduce duplication with PickUpBinBag which also attempts to do lid things.
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- PickUpBinBag.action
    - [ ] `pick_up_bin_bag_server.py` updated to new inheritance structure. Likely should inherit from PickUpObject.
    - [ ] Rewritten to be less hard-coded
    - [ ] Tested in simulation
    - [ ] Tested on real robot
- [ ] Implement a check that the handle has been grasped (or object!)!!!
- [ ] Manage all parameters (defaults and specific to specific action servers) through rosparam.
    - [ ] Add YAML files to hold default parameters.
- [ ] Currently, when the robot puts objects down it assumes they are a certain height. We should track the height and grip point of objects after picking them up, and use that information when we put the object down again so that it is gently dropped from ~1cm height above the surface.
- [ ] Additional visualisation:
    - [ ] Add a flag to octomap_to_reconstruction_service to publish octomap boxes as rviz markers if stl files not available over network
## Notes

* The HSR struggles to generate handles in a point cloud if their material is too specular like that in the lab. Similarly it can't detect the glass doors.
* Grasp pose synthesis is a bit weird when applied to the door handle point cloud. It doesn't give the classic perpendicular pose as I would like. Think this should be hard coded instead.
* The omni_base commands fail if the pose integrator is killed (as was required for using AMCL move\_base)
