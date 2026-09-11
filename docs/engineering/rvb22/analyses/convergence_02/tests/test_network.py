import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.sparse import coo_matrix
from shapely.geometry import box
from shared_network import solve_network
class NetworkTests(unittest.TestCase):
    def matrix(self,heat=1):
        return {"base":coo_matrix(np.array([[1.,-1.,0,0],[-1.,2.,-1.,0],[0,-1.,2.,-1.],[0,0,-1.,1.]])).tocsr(),"q":np.array([0.,0.,0.,heat]),
            "N":1,"areas":np.array([0.]),"carrier":box(0,0,1,1),
            "overlap":lambda g:np.array([1.]),"height_m":.05,"x":np.array([.5]),"y":np.array([.5]),
            "heat":[("Q",heat,box(0,0,1,1))],"source_layers":{"Q":3},"step":1,
            "disconnected_copper_cells":0}
    def contact(self,name="c",ri=2,rb=3):
        return {"name":name,"interface_R":ri,"bridge_R":rb,"wkt":box(0,0,1,1).wkt}
    def test_bad_sink(self):
        with self.assertRaises(ValueError):solve_network(self.matrix(),[self.contact()],sink_R=0)
    def test_bad_interface(self):
        with self.assertRaises(ValueError):solve_network(self.matrix(),[self.contact(ri=0)])
    def test_series_fixed_sink(self):
        r=solve_network(self.matrix(),[self.contact()])
        self.assertAlmostEqual(r["source_region_mean_C"]["Q"],75,7)
        self.assertAlmostEqual(r["contact_shoe_C"]["c"],73,7)
    def test_series_finite_sink(self):
        r=solve_network(self.matrix(),[self.contact()],sink_R=4)
        self.assertAlmostEqual(r["sink_C"],69,7)
        self.assertAlmostEqual(r["source_region_mean_C"]["Q"],74,7)
    def test_parallel_paths(self):
        r=solve_network(self.matrix(),[self.contact("a"),self.contact("b")])
        self.assertAlmostEqual(r["source_region_mean_C"]["Q"],72.5,7)
        self.assertAlmostEqual(r["contact_heat_W"]["a"],.5,7)
    def test_heat_doubles(self):
        r=solve_network(self.matrix(2),[self.contact()],sink_R=4)
        self.assertAlmostEqual(r["source_region_mean_C"]["Q"],83,7)
        self.assertLess(abs(r["energy_balance_residual_W"]),1e-9)
if __name__=="__main__":unittest.main()
