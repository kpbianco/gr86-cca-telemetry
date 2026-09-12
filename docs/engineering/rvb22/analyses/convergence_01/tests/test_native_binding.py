import copy, sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_binding import content_hash
class BindingTests(unittest.TestCase):
    def setUp(self):
        self.board = ["kicad_pcb", ["footprint","TEST",["property","MPN","A",["uuid","p"]],
            ["fp_rect",["start",0,0],["end",1,1],["layer","B.CrtYd"],["uuid","f"]],
            ["pad","1","smd","rect",["at",0,0],["net",30,"GND"],["uuid","pad"]]],
            ["segment",["start",1,2],["end",3,4],["layer","F.Cu"],["uuid","s"]],
            ["zone",["net",30],["filled_polygon",["layer","In1.Cu"],["pts",["xy",1,1]]]],
            ["via",["at",2,2],["drill",0.2],["uuid","v"]]]
    def change(self,path,value,same=False):
        other=copy.deepcopy(self.board);node=other
        for k in path[:-1]:node=node[k]
        node[path[-1]]=value
        self.assertEqual(content_hash(self.board)==content_hash(other),same)
    def test_property_id(self):self.change([1,2,3,1],"new",True)
    def test_courtyard_id(self):self.change([1,3,-1,1],"new",True)
    def test_mpn(self):self.change([1,2,2],"B")
    def test_courtyard_geometry(self):self.change([1,3,1,1],.1)
    def test_pad_uuid(self):self.change([1,4,-1,1],"new")
    def test_pad_net(self):self.change([1,4,-2,1],31)
    def test_segment_geometry(self):self.change([2,1,1],1.1)
    def test_filled_copper(self):self.change([3,2,2,1,1],1.1)
    def test_drill(self):self.change([4,2,1],.3)
if __name__=="__main__":unittest.main()
