"""Guard evidence effectivity and prevent count-driven or physically fabricated closure."""
import copy, hashlib, json, sys, tempfile, unittest
from pathlib import Path
D=Path(__file__).resolve().parents[1];sys.path.insert(0,str(D))
import led_bound as led
import reconcile as rec
import return_review as ret

class I26Guards(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.l=led.analyze(led.W/'candidate/cad/GR86_CCA_RevB.kicad_pcb')
  cls.g=rec.load(D/'HISTORICAL_54_RETURN_FINDINGS.json')
  cls.t=rec.load(rec.W/'I25_THERMAL_MILESTONE.json')
  cls.r=rec.load(rec.W/'FINAL_REVIEW_REGISTER.json')
 def test_seven_LEDs_present(self): self.assertEqual({r['LED'] for r in self.l['rows']},{'D401','D601','D602','D603','D604','D605','D606'})
 def test_GPIO_buffer_population(self): self.assertEqual((self.l['GPIO_driven_LED_count'],self.l['buffer_driven_LED_count']),(6,1))
 def test_drift_current_corner(self): self.assertAlmostEqual(led.current_upper_mA(1000,True),3.7863011623944566)
 def test_total_GPIO_current(self): self.assertAlmostEqual(self.l['six_GPIO_total_current_upper_with_drift_mA'],22.71780697436674)
 def test_zero_resistance_rejected(self):
  with self.assertRaises(ValueError): led.current_upper_mA(0)
 def test_optical_guarantee_not_invented(self): self.assertEqual(self.l['optical_prediction']['guaranteed_low_current_hot_optical_min_mcd'],0)
 def test_typical_inference_identified(self): self.assertTrue(self.l['optical_prediction']['typical_curve_only'])
 def test_optical_case_count(self): self.assertEqual(self.l['optical_prediction']['case_count'],48)
 def test_temperature_extrapolation_rejected(self):
  with self.assertRaises(ValueError): led.predicted_current(3.3,1000,10,-40)
 def test_curve_extrapolation_rejected(self):
  with self.assertRaises(ValueError): led.interpolate(10,[0,1],[0,1])
 def test_smaller_resistor_not_silently_adopted(self): self.assertEqual([r['resistance_ohm'] for r in self.l['resistor_comparison'] if r['is_adopted']],[1000])
 def test_LED_self_heating_not_total_supply_heat(self):
  for r in self.l['rows']: self.assertLess(r['LED_power_upper_mW'],r['limiter_power_upper_mW'])
 def test_changed_pcb_invalidates_LED(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'b.kicad_pcb';p.write_text('(kicad_pcb)')
   with self.assertRaisesRegex(ValueError,'effectivity'): led.analyze(p)
 def test_changed_filled_pcb_invalidates_return(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'b.kicad_pcb';p.write_text('(kicad_pcb)')
   with self.assertRaisesRegex(ValueError,'effectivity'): ret.load_native(p)
 def test_all54_unique(self): self.assertEqual((len(self.g['rows']),len({r['uuid'] for r in self.g['rows']})),(54,54))
 def test_48_removed_6_retained(self): self.assertEqual(self.g['counts'],{'REMOVED_BY_PRIOR_COORDINATED_COPPER_REVISION':48,'ISOLATED_TX_STUB_NO_RECEIVE_PATH':6})
 def test_zero_added_detour_not_zero_inductance(self):
  for r in self.g['rows']:
   if r['historical_gap_excess_loop_inductance_nH']==0: self.assertIn('NOT zero',r['zero_excess_interpretation'])
 def test_ground_not_falsely_closed(self): self.assertFalse(self.g['gnd02_desktop_complete'])
 def test_expanded60_transfers(self): self.assertEqual(rec.load(D/'EXPANDED_RETURN_SCOPE.json')['signal_via_count'],60)
 def test_raw_CAN_RXD_not_omitted(self): self.assertEqual(rec.load(D/'EXPANDED_RETURN_SCOPE.json')['finding_counts']['CAN_RXD'],2)
 def test_GPIO18_receiver_scope(self): self.assertIn('GPS_TX_MODULE_BUFFERED',rec.load(D/'EXPANDED_RETURN_SCOPE.json')['critical_net_set'])
 def test_reconcile_preserves_original_fields(self):
  a=copy.deepcopy(self.r);b=rec.reconcile_register(a,self.l,self.g,self.t,D/'QUALIFICATION_GATES.json')
  for old,new in zip(self.r['rows'],b['rows']):
   for k in ['id','row','A','B','C','D','E','F','closure']: self.assertEqual(old[k],new[k])
   if old['id'] not in rec.IDS: self.assertEqual(old,new)
 def test_missing_criterion_fails(self):
  a=copy.deepcopy(self.r);a['rows'].pop()
  with self.assertRaises(ValueError): rec.reconcile_register(a,self.l,self.g,self.t,D/'QUALIFICATION_GATES.json')
 def test_duplicate_criterion_fails(self):
  a=copy.deepcopy(self.r);a['rows'][0]['id']=a['rows'][1]['id']
  with self.assertRaises(ValueError): rec.reconcile_register(a,self.l,self.g,self.t,D/'QUALIFICATION_GATES.json')
 def test_false_ground_close_fails(self):
  g=copy.deepcopy(self.g);g['gnd02_desktop_complete']=True
  with self.assertRaises(ValueError): rec.reconcile_register(copy.deepcopy(self.r),self.l,g,self.t,D/'QUALIFICATION_GATES.json')
 def test_C02_adoption_invalidates_reconciliation(self):
  t=copy.deepcopy(self.t);t['decisions']['adopt_c02_cooler']=True
  with self.assertRaises(ValueError): rec.reconcile_register(copy.deepcopy(self.r),self.l,self.g,t,D/'QUALIFICATION_GATES.json')
 def test_guaranteed_optical_min_cannot_be_invented(self):
  l=copy.deepcopy(self.l);l['optical_prediction']['guaranteed_low_current_hot_optical_min_mcd']=.1
  with self.assertRaises(ValueError): rec.reconcile_register(copy.deepcopy(self.r),l,self.g,self.t,D/'QUALIFICATION_GATES.json')
 def test_gates_unperformed(self):
  g=rec.qualification_gates();self.assertFalse(g['physical_tests_performed']);self.assertTrue(all(r['status']=='NOT_EXECUTED' for r in g['gates']))
 def test_actual_count_is_one_not_zero(self): self.assertEqual(rec.load(D/'RECONCILIATION_AUDIT.json')['actionable_prehardware_ids'],['GND-02'])
 def test_thermal_board_not_junction(self):
  text=json.dumps(rec.qualification_gates());self.assertIn('not a junction-temperature',text);self.assertIn('immediate_air_max_C',text)
 def test_self_heat_Rtheta_not_misapplied(self): self.assertIn('16mm2',json.dumps(rec.qualification_gates()))
 def test_aggregate_counts_290(self): self.assertEqual(sum(rec.load(D/'RECONCILIATION_AUDIT.json')['prehardware_counts'].values()),290)
if __name__=='__main__': unittest.main()
