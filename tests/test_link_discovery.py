import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
spec=importlib.util.spec_from_file_location('link_discovery',Path(__file__).resolve().parents[1]/'Add-IN/ACDC4Robot/core/link_discovery.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def occ(path,children=0,bodies=1,visible=True):
    return NS(fullPathName=path,isLightBulbOn=visible,childOccurrences=NS(count=children),component=NS(bRepBodies=NS(count=bodies)))
class Joint:
    def __init__(self,name,a,b):self.name=name;self.occurrenceOne=a;self.occurrenceTwo=b
class DiscoveryTests(unittest.TestCase):
    def root(self,occs,joints,asbuilt=()):return NS(allOccurrences=occs,allJoints=joints,allAsBuiltJoints=asbuilt)
    def test_rigid_nested_subparts(self):
        a,b,sub=occ('base:1'),occ('arm:1',1,0),occ('arm:1+bracket:1')
        links,joints=m.discover_links(self.root([a,b,sub],[Joint('J',a,b)]))
        self.assertEqual([x.fullPathName for x in links],['base:1','arm:1'])
    def test_same_name_distinct_joints_and_repeated_entity(self):
        a,b,c=occ('a'),occ('b'),occ('c');j1,j2=Joint('J',a,b),Joint('J',b,c)
        joints=m.unique_joints(self.root([a,b,c],[j1,j2],[j1]))
        self.assertEqual(joints,[j1,j2])
    def test_unconnected_geometry_rejected(self):
        a,b,c=occ('a'),occ('b'),occ('c')
        with self.assertRaisesRegex(ValueError,'Visible geometry c'):m.discover_links(self.root([a,b,c],[Joint('J',a,b)]))
    def test_hidden_endpoint_rejected(self):
        a,b=occ('a'),occ('b',visible=False)
        with self.assertRaisesRegex(ValueError,'hidden'):m.discover_links(self.root([a,b],[Joint('J',a,b)]))
    def test_overlapping_subtrees_rejected(self):
        a,b=occ('a',1),occ('a+b')
        with self.assertRaisesRegex(ValueError,'Overlapping'):m.discover_links(self.root([a,b],[Joint('J',a,b)]))
    def test_ground_rejected(self):
        a=occ('a')
        with self.assertRaisesRegex(ValueError,'root/ground'):m.discover_links(self.root([a],[Joint('J',None,a)]))
    def test_jointless_leaf_compatibility(self):
        a,b,c=occ('a',1),occ('a+b'),occ('c',visible=False)
        links,joints=m.discover_links(self.root([a,b,c],[]));self.assertEqual(links,[b]);self.assertEqual(joints,[])
