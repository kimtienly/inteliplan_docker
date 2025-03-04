'''
This is the main inteliplan interface.
'''
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch
import time
import sys
import re
seed=0
torch.manual_seed(seed)
from transformers.utils import logging
logging.set_verbosity_error()


# np.random.seed(seed)
class Inference: 
    def __init__(self, model, tokenizer):
        self.model = model
        self.tokenizer = tokenizer

    def infer(self, msg, print_output = False):
        # print('infered text: ',msg)
        model_inputs = self.tokenizer.apply_chat_template(msg, return_tensors="pt").to("cuda")

        generated_ids = self.model.generate(model_inputs, max_new_tokens=100, do_sample=True)
        result= self.tokenizer.batch_decode(generated_ids)[0]
        start_index = result.rfind('[/INST]')
        output = result[start_index+7:]
        if not('failed' in output):
            match = re.search(r'<robot>: .*?\.', output)
            output=match.group()  
        if print_output:
            self.print_conversation(output)
        return output


    def print_conversation(self, out, remove='<history>:'):
        print('\n\n------------------------')
        # Remove the specified substring until the next '<'
        start_index = out.find(remove)
        if start_index != -1:
            end_index = out.find('<', start_index + len(remove))
            if end_index != -1:
                out = out[:start_index] + out[end_index:]    

        start_inds = [i for i, letter in enumerate(out) if letter == '<']
        for i, char in enumerate(out):
            if i in start_inds:
                print('')
            print(char, end='')
    
    def interactive_session(self, vision_func = None, vision_loc=None, feasibility_func=None, robot_func = None):          
        inp_prompt = "<history>: "
        out = ''
        while True:
            inp_user = input("<user>: ")
            while True:
                if vision_func is not None:
                    vision_result = vision_func(inp_user)
                    if vision_result:
                        inp_vision = 'found'
                    else:
                        inp_vision = 'none'
                    print("<vision>: ", inp_vision)
                else:
                    inp_vision = input("<vision>: ")

                if feasibility_func is not None:
                    object_location = vision_loc()
                    inp_score = feasibility_func(object_location)
                else:
                    inp_score = input("<feasibility>: ")

                if len(out)>0:
                    inp_prompt = '<history>: <user>: '+ inp_user +' '+ out[start_index:]
                else:
                    inp_prompt = "<history>: "
                # print(inp_prompt)
                input_string = inp_prompt + " <user>: "+ inp_user + " <vision>: "+inp_vision +" <feasibility>: " +inp_score
                messages = [{"role": "user", "content": input_string}]
                out = self.infer(messages)
                start_index = out.rfind('<robot>:')
                print(out[start_index:])

                if 'guidance' in out[start_index:]:

                    inp_prompt = "<history>: <user>: '"+ inp_user +" "+ out[start_index:]+"'"
                    inp_user_temp = input("<user>: ")
                    input_string = inp_prompt + " <user>: "+ inp_user_temp + " <vision>: "+inp_vision +" <feasibility>: " +inp_score
                    messages = [{"role": "user", "content": input_string}]
                    out = self.infer(messages)
                    start_index = out.rfind('<robot>:')
                    print(out[start_index:])

                if robot_func is not None:
                    succeeded = robot_func(out[(start_index+8):])
                    print('====================================')
                    print('ROBOT EXCECUTION RESULT:', succeeded)
                    print('====================================')
                else:
                    succeeded = input('Succeeded? (y/n)')

                if succeeded == 'y' or succeeded == True:
                    break
                out = ''

            next = input('enter "q" to exit or "n" to start a new chat or any key to continue: ')
            
            
            # Clear the last line
            sys.stdout.write('\033[F')  # Move cursor up one line
            sys.stdout.write('\033[K')  # Clear the line
            if next == 'q':
                break
            elif next == 'n':
                print('\n--------------------\n')
                out = ''

if __name__=="__main__":
    
    model_path = "./../checkpoints/fetchme"
    model = AutoModelForCausalLM.from_pretrained(model_path,load_in_4bit=True, device_map="auto" )
    tokenizer = AutoTokenizer.from_pretrained(model_path)
    torch.cuda.empty_cache()

    test = Inference(model, tokenizer)
    test.interactive_session()
    start_time = time.time()

    # test.print_conversation("<user>: fetch me a ball <vision>: found <feasibility>: 1")
    # # test.infer("<user>: fetch me a ball <vision>: found <feasibility>: 1")
    # # test.infer("<user>: bring me a cake <vision>: none <feasibility>: 1")
    # # test.infer("<user>: can you get me an apple <vision>: found <feasibility>: 0")


    # end_time = time.time()

    # processing_time = end_time - start_time
    # # Print the processing time
    # print(f"\nInference time: {processing_time:.4f} seconds")