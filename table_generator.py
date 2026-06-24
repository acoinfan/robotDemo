import json
from object import Object, Table

def test_build():
    origin = Table.from_json("./data/0001/origin.json")
    end = Table.from_json("./data/0001/end.json")
    print(origin)
    print(end)
    for object1 in origin.objects:
        for object2 in end.objects:
            if object1 == object2 and object1.position_changed(object2):
                print(f"object {object1.name} moved: ")
                print(object1.get_movement())
    
test_build()