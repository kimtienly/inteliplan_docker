import xml.etree.ElementTree as ET

tree = ET.parse('room_small.world')
root = tree.getroot()

offset = [-2.23, -3.66, 0, 0, 0, 0]

def add_offset(pose):
    values = [float(v) for v in pose.split()]
    new_values = [values[i] + offset[i] for i in range(6)]
    return ' '.join(map(str,new_values))

for entity in root.findall('.//pose'):
    original = entity.text
    entity.text = add_offset(original)

file_to_save = 'shifted_world.world'
tree.write(file_to_save)
print('World shifted. Saved to '+file_to_save)