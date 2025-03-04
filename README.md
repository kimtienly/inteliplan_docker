# InteLiPlan workspace
Author: Kim Tien Ly

This workspace is intended for paper "InteLiPlan: Interactive Lightweight LLM-Based Planner for Domestic Robot Autonomy".

More details of the paper is available at [our project page](https://kimtienly.github.io/InteLiPlan/).

# Docker usage
Use the following command to:
- build

```bash
make build
```

- Build without cache
```bash
docker compose build --no-cache
```

- create the container
```bash
make up
```

Once the container is running, open container terminal:
```bash
docker exec -it inteliplan_docker bash
```

# Running

## Fine-tune the model
Code for fine-tuning is in `/inteliplan/`. Run the python files in the following order:
- `create_fetchme_data.py` provide a quick example of generate csv data files. The data will be text-only as described in the paper.
- `data_csv_to_jsonl` converts the generated csv file to a jsonl format dedicated for fine-tuning.
- `train.py` finetunes the LLM model (default: Mistral).
- `inference.py` tests the trained model.


## Run the full system with simulation
The system was tested on the Toyota Human Support Robot (HSR). The HSR simulation package is installed after the docker was built. The simulation is operating with ROS 1.

`inteliplan_robot` is the ROS package for the robot experiment. Proceed to `inteliplan_ws` to build and source the workspace with:
```bash
catkin build
source devel/setup.bash
```

In order to be able to run **graphical user interfaces** from inside the Docker, on the **host system**, run:

```bash
xhost +local:root
```

Intall octomap-server from inside docker:
```bash
sudo apt-get install ros-noetic-octomap-server
```


Launch the tmux workspace:
```bash
rosrun inteliplan_interface run.tmux
```
- The first window **robot** has two terminals: 
    - `simulation_vision.launch` launches the robot simulation together with vision recognition
    - `manipulation_servers_no_grasp_synthesis.launch` launches the manipulation servers, which includes a list of motion APIs.
- The second window **inteliplan** is the main inteliplan pipeline, where you input commands and instructions for the robot.
- Tips for tmux:
    - Ctrl+B then 0 to open **robot** window
    - Ctrl+B then 1 to open **inteliplan** window
    - Ctrl+B then left arrow/right arrow to navigate between terminals on the same window.
    - Ctrl+B then [ to scroll in each terminal





