from vision_subscriber import DetectionSubscriber
from feasibility_graph import FeasibilitySubscriber
from action_execution import HSR_control
from transformers_inference import Inference
from transformers import AutoModelForCausalLM, AutoTokenizer
import rospy
import torch
import os
seed=0
torch.manual_seed(seed)

robot_origin=None
from huggingface_hub import login
hf_token = os.getenv("HF_TOKEN")
if hf_token:
    login(hf_token)
else:
    print("Hugging Face token is not set. Please set HF_TOKEN environment variable.")

def robot_function(inp):
    action_list = inp.split(',')
    # print('action_list:', action_list)
    # for action in action_list:
    #     if 'back' in action:
    #         global robot_origin
    #         robot_origin = robot.omni_base.pose
            # print('robot_origin is set to ',str(robot_origin))
    for action in action_list:
        temp = [item for item in action.split(' ') if item] 
        if temp[0]=='failed':
            return
        print('--------------------------------------')
        print('Executing action: '+action)
        # print('temp:', temp)

        if temp[0]=='pick':
            if vision.is_seen(temp[1]):
                re = robot.pick_up_object_client(vision.get_pose(temp[1]))
            else:
                print('Pick up action is called but no {} is found'.format(temp[1]))
                return False, 'vision'
        elif temp[0]=='turn':
            re = robot.turn(temp[1])
        elif temp[0]=='search':
            re = robot.search(temp[-1],vision_func=vision.is_seen)
        elif temp[0]=='go' or temp[0]=='move':
            if temp[1]=='back':
                re = robot.move(robot_origin)
            elif temp[1]=='closer':
                re = robot.move([0.5,0,0])
        elif temp[0]=='place':
            re = robot.put_object_on_surface_client([1.7, 0.85, 0.5]) 
        if re == False:
            return False
        
    return True


rospy.init_node('inteliplan_robot')

# initialize robot poses
robot = HSR_control()



# Intialize the model and tokenizer
model_path = "/inteliplan_ws/src/inteliplan_robot/inteliplan_interface/models/fetchme"
model = AutoModelForCausalLM.from_pretrained(model_path,load_in_4bit=True, device_map="auto" )
tokenizer = AutoTokenizer.from_pretrained(model_path)


test = Inference(model, tokenizer)
vision = DetectionSubscriber()
scene = {'scene':{'file':'../worlds/room.g', 'mesh_type': False}}
map_bounds = [[-3, 3],[-2, 2],[0.5, 1.3]] # limits on map dimensions
feasibility = FeasibilitySubscriber(scene, map_bounds, DEBUG=False)
test.interactive_session(vision_func=vision.is_seen, vision_loc=vision.get_pose, feasibility_func=feasibility.get_score, robot_func=robot_function)
rospy.spin()