import json

from torch.utils.data import Dataset
import json
import pandas as pd
class ChatData(Dataset):
    def __init__(self, path:str, tokenizer, enable_prompt=False, data_num=None):
        # self.data = json.load(open(path, "r"))
        self.data = pd.read_csv(path, na_filter=False)
        if data_num:
            self.data = pd.read_csv(path, nrows=data_num, na_filter=False)
        self.X = []
        for index, row in self.data.iterrows():
            # msg= [{"role": "input", "content": f"<prompt>: '{row['Prompt']}'. <user>: {row['User']}. <vision>: {row['Vision']}. <feasibility>: {row['Score']}."},
            #     {"role": "output", "content": f"<robot>: {row['Robot']}" }
            #     ]
            msg= [{"role": "user", "content": f"<history>: ''. <user>: {row['User']}. <vision>: {row['Vision']}. <feasibility>: 1."},
                {"role": "assistant", "content": f"<robot>: {row['Robot']}" }
                ]
            self.X.append({"messages":msg})
        
        # print('length of data: ',len(self.X))
        # print(self.X[0])
        # print(self.X[3])


    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return (self.input_ids[idx], self.attention_mask[idx])
    

if __name__=="__main__":
    csv_filename='data/data_conversation_fetchme'
    my_dataset = ChatData(f'{csv_filename}.csv',None, enable_prompt=True)
    data = my_dataset.X
    # import random
    # data_ = random.sample(data, 1000) 
    data_ = data[:]
    print(len(data_))
    # Save the data to a JSONL file
    with open(f'{csv_filename}.jsonl', 'w') as f:
        for entry in data_:
            f.write(json.dumps(entry) + "\n")
