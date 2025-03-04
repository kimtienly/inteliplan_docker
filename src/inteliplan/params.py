seen_objects = [
    "apple", "banana", "orange", "lemon", "lime",
    "mug", "bottle", "can", "glass", "cup",]


unseen_objects = [
"plum",
"strawberry",
"pear",
"pineapple",
"lemon",
"cherry",
"melon",
"jam",
"rubik",
"wine bottle",
"tea cup",
"box",          
"bag",         
"tool",         
"phone",        
"remote",       
"vegetable",    
"flower",       
"jar",
"cracker",
]
 
seen_commands = ['Get me a bottle']
unseen_commands = [
    "Bring me a bottle.",
    "Can you grab a bottle for me?",
    "Could you bring me a bottle?",
    "Please bring me a bottle.",
    "I need a bottle, please.",
    "Would you mind getting a bottle for me?",
    "Can I have a bottle, please?",
    "Could you pick up a bottle for me?",
    "Please get a bottle for me.",
    "Hand me a bottle, please.",
    "Grab me a bottle.",
    "Please fetch me a bottle.",
    "Can you please get a bottle for me?",
    "Could you go get a bottle for me?",
    "Please pass me a bottle."
]

seen_guidance_dict = {"left":["Turn left","It is on the left"],
'right':["Turn right","It is on the table to the right"],
'around':["It is behind you","Turn around"]}

unseen_guidance_dict = {"left":["The item is located to the left","It is on the table to the left","Check the table on your left","Rotate to your left side", "Head left for the object"],
                        "right":["The item is located to the right","It is to your right","Check the shelf on your right","Rotate to your right side", "Head right for the object"],
                        "around":["Spin around", "Turn backwards","It is on the table behind you","Get it from your back","Make a 180-degree rotation"]}

