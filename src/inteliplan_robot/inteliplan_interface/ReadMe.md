# To run on Alienware
In different terminals:
- Launch simulation
```
roslaunch transformers_robot simulation_vision.launch
```
- Launch manipulation stack. Doing this separately makes it easier to read the terminal.
```
roslaunch manipulation manipulation_servers_no_synthesis.launch
```
- Run the main pipeline in conda environment:
```
conda activate transformers && python3 main.py
```

# World design
Due to not being able to spawn human anywhere apart from [0,0], the world model need to be shifted an offset. To do that:
```
cd src/transformers_robot/worlds/
python3 shift_world.py
```