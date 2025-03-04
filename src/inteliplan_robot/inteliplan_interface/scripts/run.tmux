#!/bin/bash

# Tmux session name
SESSION=inteliplan_experiment
WS_NAME=inteliplan_ws
# Set this variable in order to source ORIon ROS workspace
WS=/$WS_NAME/devel/setup.bash

CLEAR_PANE='tmux -u send-keys "clear" Enter'
_SRC_ENV="tmux send-keys source Space $WS C-m "

PREFIX="source $WS;"

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'robot'
tmux new-window -t $SESSION:1 -n 'inteliplan'


# ######################################################################

tmux select-window -t $SESSION:robot

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch inteliplan_interface simulation_world.launch"

tmux split-window -h
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch manipulation manipulation_servers_no_grasp_synthesis.launch"

# ######################################################################

tmux select-window -t $SESSION:inteliplan

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX cd src/inteliplan_robot/inteliplan_interface/scripts/; python3 main.py"



# ######################################################################

# Set default window
tmux select-window -t $SESSION:robot

# Attach to session
tmux -2 attach-session -t $SESSION