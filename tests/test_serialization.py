import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from xml.etree.ElementTree import fromstring
spec=importlib.util.spec_from_file_location('serialization',Path(__file__).resolve().parents[1]/'Add-IN/ACDC4Robot/core/serialization.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class SerializationTests(unittest.TestCase):
    def test_tiny_inertia_preserved_without_exponent(self):
        for value in ['1e-61','-1.234e-12','123456789.123456789','1e+20']:
            result=m.decimal_number(value)
            self.assertNotIn('e',result.lower())
            self.assertEqual(m.Decimal(value),m.Decimal(result))
    def test_zero_and_trailing_zeros(self):
        self.assertEqual(m.decimal_number('-0.000'),'0')
        self.assertEqual(m.decimal_number('100'),'100')
        self.assertEqual(m.decimal_number('1.200'),'1.2')
    def test_nonfinite_rejected(self):
        for value in ['nan','inf','-inf','invalid']:
            with self.assertRaises(ValueError):m.decimal_number(value)
    def test_numeric_fields_only(self):
        element=fromstring('<link name="some1e-3"><inertial><mass value="1e-8"/><inertia ixx="1e-61"/></inertial><visual><geometry><mesh filename="some1e-3.stl" scale="1e-3 1e-3 1e-3"/></geometry></visual></link>')
        m.normalize_urdf_numbers(element)
        self.assertEqual(element.attrib['name'],'some1e-3')
        mesh=element.find('visual/geometry/mesh');self.assertEqual(mesh.attrib['filename'],'some1e-3.stl')
        self.assertEqual(mesh.attrib['scale'],'0.001 0.001 0.001')
        self.assertGreater(float(element.find('inertial/inertia').attrib['ixx']),0)
    def test_instance_and_path_identity(self):
        self.assertEqual(m.export_name('Robot arm:2+Finger:1'),'Robot_arm_2__Finger_1')
        self.assertNotEqual(m.export_name('Payload:1'),m.export_name('Payload:2'))
    def test_collision_rejected(self):
        with self.assertRaisesRegex(ValueError,'duplicate'):
            m.validate_export_names([NS(fullPathName='Robot arm:1'),NS(fullPathName='Robot_arm:1')])
