from transformers import AutoModelForCausalLM, AutoTokenizer
import torch
from params import *
seed=0
torch.manual_seed(seed)

def inference(msg):
    model_inputs = tokenizer.apply_chat_template(msg, return_tensors="pt").to("cuda")

    generated_ids = model.generate(model_inputs, max_new_tokens=100, do_sample=True)
    result= tokenizer.batch_decode(generated_ids)[0]
    start_index = result.rfind('[/INST]')
    print(result)
    # print(result[start_index+7:])
    return result[start_index+7:]

def inference2(msg, log_time = False):
    import time
    start_time = time.time()
    # print('====== Infered message: ',msg)
    messages = msg
    # model_input = tokenizer(msg, return_tensors="pt").to("cuda")
    # model.eval()
    # Tokenize the user input with the chat template
    inputs = tokenizer.apply_chat_template(
    messages,
    tokenize=True,  
    add_generation_prompt=True,  
    return_tensors="pt", 
    padding=True,  # Add padding to match sequence lengths
    ).to("cuda") 

    attention_mask = inputs != tokenizer.pad_token_id

    outputs = model.generate(
    input_ids=inputs,
    attention_mask=attention_mask, 
    max_new_tokens=256,  
    use_cache=True,  # Use cache for faster token generation
    temperature=0.6,  # Controls randomness in responses
    min_p=0.1,  # Set minimum probability threshold for token selection
    pad_token_id=tokenizer.eos_token_id
    )

    # Decode the generated tokens into human-readable text
    text = tokenizer.decode(outputs[0], skip_special_tokens=True)

    if log_time:
        print('Interence time: ',time.time()-start_time)
    print(text) 
    return text


def test_interactive():
    import sys
    inp_prompt = "<history>: ''"
    cnt = 0
    while True:
        inp_user = input("<user>: ")
        if cnt == 0:
            inp_vision = input("<vision>: ")
            inp_score = input("<feasibility>: ")
            
        input_string = inp_prompt + ". <user>: "+ inp_user + ". <vision>: "+inp_vision +". <feasibility>: " +inp_score+'.'
        print(input_string)
        messages = [{"role": "user", "content": input_string}]
            
        out = inference(messages)
        start_index = out.rfind('<robot>:')
        print(out[start_index:])
        inp_prompt = '"<history>: '+inp_user +" "+out[start_index:]+'"'
        next = input('enter "q" to exit or "n" to start a new chat or any key to continue: ')
        # Clear the last line
        sys.stdout.write('\033[F')  # Move cursor up one line
        sys.stdout.write('\033[K')  # Clear the line
        cnt += 1
        if next == 'q':
            break
        elif next == 'n':
            cnt = 0 
            print('\n--------------------\n')
            inp_prompt = "<history>: "



model_path = "./trained_model_backup"
# Load the model and tokenizer from the saved directory
model = AutoModelForCausalLM.from_pretrained(model_path,load_in_4bit=True, device_map="auto" )
tokenizer = AutoTokenizer.from_pretrained(model_path)
print("Model and tokenizer loaded successfully.")

test_interactive()