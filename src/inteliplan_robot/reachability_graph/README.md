To generate the reachability graph, you need to have required robot description package depending on which ROBOT is used (which is defined in [configuration](./src/reachability_graph/configuration.py)):
- Kinova-anymal:
    - Anymal: anymal_c (default), [anymal_coyote_drs](https://github.com/ori-drs/anymal_coyote_drs)
    - Kinova: [kinova-ros](https://github.com/ori-drs/kinova-ros)
    - Robotiq: [robotiq](https://github.com/ori-drs/robotiq)
- PR2: [pr2_common](https://github.com/PR2/pr2_common)
    - Dependencies: ros-noetic-convex-decomposition, ros-noetic-ivcon