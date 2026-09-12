"""Negative-path tests for configuration-sensitive engineering evidence."""
import copy,json,sys,tempfile,unittest
from pathlib import Path
D=Path(__file__).resolve().parents[1];sys.path.insert(0,str(D))
import run_review as r
import thermal_native as t

class ProductGuards(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.b=r.sx.loads((r.W/'candidate/cad/GR86_CCA_RevB.kicad_pcb').read_text())
  cls.code=(r.W/'candidate/firmware/cca_telemetry/cca_telemetry.ino').read_text()
  cls.header=(r.W/'candidate/firmware/cca_telemetry/src/led_status.h').read_text()
 def change_property(self,b,ref,key,value):
  f=r.fp_map(b)[ref]
  for p in r.c.child(f,'property'):
   if p[1]==key:p[2]=value;return
  raise AssertionError('Test fixture property missing')
 def test_receive_only_current(self):self.assertEqual(r.receive_only(self.b,self.code)['transmit_calls'],0)
 def test_population_of_isolation_link_rejected(self):
  b=copy.deepcopy(self.b);self.change_property(b,'R301','Assembly','FACTORY')
  with self.assertRaisesRegex(ValueError,'unpopulated'):r.receive_only(b,self.code)
 def test_removing_pullup_rejected(self):
  b=copy.deepcopy(self.b);self.change_property(b,'R303','Assembly','DNP')
  with self.assertRaisesRegex(ValueError,'pull-up'):r.receive_only(b,self.code)
 def test_termination_population_rejected(self):
  b=copy.deepcopy(self.b);self.change_property(b,'R306','Assembly','FACTORY')
  with self.assertRaisesRegex(ValueError,'termination'):r.receive_only(b,self.code)
 def test_transmit_call_rejected(self):
  with self.assertRaises(ValueError):r.receive_only(self.b,self.code+'\nvoid bad(){twai_transmit(0,0);}\n')
 def test_normal_mode_rejected(self):
  with self.assertRaises(ValueError):r.receive_only(self.b,self.code.replace('TWAI_MODE_LISTEN_ONLY','TWAI_MODE_NORMAL'))
 def test_tx_queue_rejected(self):
  with self.assertRaises(ValueError):r.receive_only(self.b,self.code.replace('general.tx_queue_len = 0','general.tx_queue_len = 1'))
 def test_can_gpio_rejected(self):
  with self.assertRaises(ValueError):r.receive_only(self.b,self.code.replace('CAN_RX_GPIO = 4','CAN_RX_GPIO = 8'))
 def test_comment_does_not_fake_transmit_call(self):r.receive_only(self.b,self.code+'\n// twai_transmit(0,0);\n')
 def test_LED_chains_current(self):
  x=r.led_review(self.b,self.header);self.assertEqual(len(x['rows']),6);self.assertLess(x['all_six_current_upper_mA'],23)
 def test_LED_polarity_rejected(self):
  with self.assertRaises(ValueError):r.led_review(self.b,self.header.replace('LED_PWR_ACTIVE_LOW 0','LED_PWR_ACTIVE_LOW 1'))
 def test_LED_resistor_rejected(self):
  b=copy.deepcopy(self.b);self.change_property(b,'R601','MPN','WRONG')
  with self.assertRaises(ValueError):r.led_review(b,self.header)
 def test_optical_guarantee_not_invented(self):
  x=r.led_review(self.b,self.header);self.assertTrue(all(v['brightness_lower_bound_mcd']==0 for v in x['rows']));self.assertFalse(x['original_WCA_07_closed'])
 def test_handling_updates_exact_two_parts(self):
  x=r.current_handling(self.b,r.read(D/'support/HANDLING_I22.json'));self.assertEqual({v['reference']for v in x['corrected_references']},{'R155','R156'});self.assertEqual(x['fitted_references'],153)
 def test_unknown_new_MPN_fails_closed(self):
  b=copy.deepcopy(self.b);self.change_property(b,'R155','MPN','UNREVIEWED')
  with self.assertRaisesRegex(ValueError,'handling review'):r.current_handling(b,r.read(D/'support/HANDLING_I22.json'))
 def test_TNPU_unknown_MSL_not_invented(self):
  x=r.current_handling(self.b,r.read(D/'support/HANDLING_I22.json'));self.assertTrue(all(v['msl']is None for v in x['rows']if v['mpn'].startswith('TNPU')))
 def test_thermal_empty_native_not_accepted(self):
  with tempfile.TemporaryDirectory() as x:
   p=Path(x)/'wrong.kicad_pcb';p.write_text('(kicad_pcb)')
   with self.assertRaisesRegex(ValueError,'differs'):t.extract_native(p)
 def test_contact_is_not_a_bare_land(self):
  g,x=t.contact_screen(self.b);self.assertGreater(g.area,0);self.assertIn('insulating TIM',x['electrical']);self.assertEqual(x['status'],'INSULATED_CONTACT_FEASIBILITY_NOT_ADOPTED')
 def test_contact_contains_complete_path_terms(self):
  _,x=t.contact_screen(self.b);self.assertGreater(x['path_terms_K_W']['axial_bridge_25x12x3_mm'],0);self.assertAlmostEqual(sum(x['path_terms_K_W'].values()),x['total_to_landing_K_W'])
 def test_real_component_notch_is_present(self):
  _,x=t.contact_screen(self.b);self.assertIn('R158',{v['ref']for v in x['courtyard_exclusions']})
 def test_regression_binds_current_source(self):
  x=r.verify_regressions(r.W/'candidate/cad/GR86_CCA_RevB.kicad_pcb',r.W/'candidate/firmware');self.assertEqual(x['firmware_suites'],9)
 def test_altered_PCB_invalidates_evidence(self):
  with tempfile.TemporaryDirectory() as x:
   p=Path(x)/'changed.kicad_pcb';p.write_text('(kicad_pcb)')
   with self.assertRaisesRegex(ValueError,'PCB changed'):r.verify_regressions(p,r.W/'candidate/firmware')
if __name__=='__main__':unittest.main()
