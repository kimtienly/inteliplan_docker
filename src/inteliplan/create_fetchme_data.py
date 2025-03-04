import csv
from params import *
data = []
import numpy as np   
for ob in seen_objects[:20]:
    # # Feasible action
    # # score=np.random.uniform(0.5,1)
    score =1 
    data.append(["","get me a {}".format(ob),"found",str(score),"pick {}, go back, place {}.".format(ob,ob)])
    
    score =0
    # cannot find
    precond = ["","get me a {}".format(ob),"none",str(score),"failed to find {} in the scene. Any guidance?".format(ob)]
    data.append(precond)
    prompt = " <user>: "+precond[1]+ " <vision>: "+precond[2]+" <score>: "+precond[3]+" <robot>: "+precond[4]
    # qqq
    prompt = " <user>: "+precond[1]+ " <robot>: "+precond[4]
    
    # guidance to turn
    case1 = ["Turn left","none",1,"turn left, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["Turn right","none",1,"turn right, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["It is on the left","none",1,"turn left, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["It is on the table to the right","none",1,"turn right, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["It is behind you","none",1,"turn around, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["Turn around","none",1,"turn around, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    
    # cannot do
    score =0
    precond = ["","get me a {}".format(ob),"found",str(score),"found the {} but I cannot perform the task. Any guidance?".format(ob,ob)]
    data.append(precond)

    prompt = " <user>: "+precond[1]+ " <vision>: "+precond[2]+" <score>: "+precond[3]+" <robot>: "+precond[4]
    prompt = " <user>: "+precond[1]+ " <robot>: "+precond[4]

    case1 = ["Get the {} on the table to the left".format(ob),"none",1,"turn left, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)

    case1 = ["Get the {} on the table to the right".format(ob),"none",1,"turn right, search for {}, pick {}, go back, place {}.".format(ob,ob,ob)]
    data.append([prompt]+case1)
     

file_name = 'data/data_conversation_fetchme.csv'
print(len(data))

with open(file_name, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Prompt","User","Vision","Score","Robot"])
    for item in data:
        writer.writerow(item)