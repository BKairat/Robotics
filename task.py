import math

class Object():
    def __init__(self, id, obj):
        self.id = id
        self.obj = obj
        
        
class Sort():
    def __init__(self, robot, cubes, cube_ids, box, box_ids):
        self.robot = robot
        self.cube_ids = cube_ids
        self.box_ids = box_ids
        self.cubes = self.set_obj(cubes)
        self.boxes = self.set_obj(box)
        
    def set_obj(self, obj):
        ret = []
        for o in obj:
            ret.append(Object(int(o[0]), o[1:]))
        return ret
            
    def get_box_by_id(self, id):
        return min(self.boxes, key=lambda x: abs(x.id-id))
            
    def run(self):
        order = {i: 0 for i in self.cube_ids}
        for i, c in enumerate(order.keys()):
            order[c] += self.box_ids[i%len(self.box_ids)]
            
        for cube in self.cubes:
            self.robot.grab_cube(cube)
            box = self.get_box_by_id(order[cube.id])
            self.robot.init_pos()
            self.robot.move_to_box(box)
            self.robot.release()
            self.robot.init_pos()
            
class Sort_hard(Sort):
    pass
    
            