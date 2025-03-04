# from transformers import GPT2LMHeadModel, GPT2Tokenizer
from transformers import AutoModelForCausalLM, AutoTokenizer,  BitsAndBytesConfig,TrainingArguments
from peft import LoraConfig,  prepare_model_for_kbit_training, get_peft_model

import torch
from huggingface_hub import login
from trl import SFTTrainer
torch.cuda.empty_cache()

# login(token="") # uncomment this and add your token

device = "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"



# LOAD DATASET====================
from datasets import load_dataset

# Load the dataset from JSONL file
dataset_file = 'data/data_conversation_fetchme.jsonl'

def format_prompts(examples):
    """
    Define the format for your dataset
    This function should return a dictionary with a 'text' key containing the formatted prompts
    """
    conversations = []
    # Iterate over the batch of examples
    for messages in examples['messages']:
        # Join all messages in the conversation into a single string
        conversation = "\n".join([f"{msg['role']}: {msg['content']}" for msg in messages])
        conversations.append(conversation)
    return {'text':conversations}

dataset = load_dataset('json',data_files=dataset_file, split="train")
train_dataset = dataset.map(format_prompts, batched=True)


bnb_config = BitsAndBytesConfig(  
    load_in_4bit= True,
    bnb_4bit_quant_type= "nf4",
    bnb_4bit_compute_dtype= torch.bfloat16,
    bnb_4bit_use_double_quant= False,
)
model = AutoModelForCausalLM.from_pretrained(
        "mistralai/Mistral-7B-Instruct-v0.2",
        quantization_config=bnb_config,
        torch_dtype=torch.bfloat16,
        device_map="auto",
        trust_remote_code=True,
        # offload_folder="offload",  # Offload parts of the model to disk
)
model.config.use_cache = False # silence the warnings
model.config.pretraining_tp = 1
model.gradient_checkpointing_enable()

tokenizer = AutoTokenizer.from_pretrained("mistralai/Mistral-7B-Instruct-v0.2", trust_remote_code=True)
tokenizer.padding_side = 'right'
tokenizer.pad_token = tokenizer.eos_token
tokenizer.add_eos_token = True
tokenizer.add_bos_token, tokenizer.add_eos_token

model = prepare_model_for_kbit_training(model)
peft_config = LoraConfig(
    lora_alpha=16,
    lora_dropout=0.1,
    r=64,
    bias="none",
    task_type="CAUSAL_LM",
    target_modules=["q_proj", "k_proj", "v_proj", "o_proj","gate_proj"]
)
model = get_peft_model(model, peft_config)


training_arguments = TrainingArguments(
    output_dir="./trained_results",
    num_train_epochs=5,
    # per_device_train_batch_size=2,
    # gradient_accumulation_steps=4,
    per_device_train_batch_size=1,  # Reduce batch size
    gradient_accumulation_steps=8,  # Increase gradient accumulation to maintain effective batch size
    optim="paged_adamw_32bit",
    # optim="adamw_bnb_8bit",  # Use 8-bit AdamW optimizer
    save_steps=25,
    logging_steps=25,
    learning_rate=2e-4,
    weight_decay=0.001,
    fp16=False,
    # fp16=True,  # Enable mixed precision training
    bf16=False,
    max_grad_norm=0.3,
    max_steps=-1,
    warmup_ratio=0.03,
    group_by_length=True,
    lr_scheduler_type="constant",
    # report_to="wandb"
)


trainer = SFTTrainer(
    model=model,
    train_dataset=train_dataset,
    # train_dataset=tokenized_train_data,  # Use the tokenized training data
    # eval_dataset=tokenized_test_data,  # Use the tokenized test data
    peft_config=peft_config,
    max_seq_length= None,
    dataset_text_field="text",
    tokenizer=tokenizer,
    args=training_arguments,
    packing= False,
)

trainer.train()
# Specify the directory to save the model
output_dir = "./trained_model"

# Save the model
model.save_pretrained(output_dir)

# Save the tokenizer
tokenizer.save_pretrained(output_dir)

print(f"Model and tokenizer saved to {output_dir}")
