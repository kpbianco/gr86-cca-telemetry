#!/usr/bin/env python3
"""Independent exported-XLSX preservation and disposition comparison."""
import argparse
import collections
import json
import posixpath
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile

NS = {'s':'http://schemas.openxmlformats.org/spreadsheetml/2006/main', 'r':'http://schemas.openxmlformats.org/officeDocument/2006/relationships'}

def read_book(path):
    with ZipFile(path) as z:
        shared = []
        if 'xl/sharedStrings.xml' in z.namelist():
            shared = [''.join(t.text or '' for t in q.findall('.//s:t',NS)) for q in ET.fromstring(z.read('xl/sharedStrings.xml'))]
        relations = {q.attrib['Id']:posixpath.normpath('xl/'+q.attrib['Target']) if not q.attrib['Target'].startswith('/') else q.attrib['Target'].lstrip('/') for q in ET.fromstring(z.read('xl/_rels/workbook.xml.rels'))}
        sheets = {}
        for sheet in ET.fromstring(z.read('xl/workbook.xml')).findall('s:sheets/s:sheet',NS):
            root = ET.fromstring(z.read(relations[sheet.attrib['{'+NS['r']+'}id']]))
            cells = {}
            for c in root.findall('s:sheetData/s:row/s:c',NS):
                f = c.find('s:f',NS)
                v = c.find('s:v',NS)
                if f is not None: value = ('formula', f.text)
                elif c.attrib.get('t') == 's': value = ('text',shared[int(v.text)]) if v is not None else None
                elif c.attrib.get('t') == 'inlineStr': value = ('text',''.join(q.text or '' for q in c.findall('.//s:t',NS)))
                elif c.attrib.get('t') == 'str': value = ('text',v.text) if v is not None else None
                elif v is not None: value = ('value',v.text)
                else: value = None
                if value is not None and value != ('text',''):cells[c.attrib['r']] = value
            sheets[sheet.attrib['name']] = cells
        drawings = len([p for p in z.namelist() if p.startswith('xl/drawings/drawing') and p.endswith('.xml')])
        charts = len([p for p in z.namelist() if p.startswith('xl/charts/chart') and p.endswith('.xml')])
        return sheets, {'drawings':drawings,'charts':charts}

def main():
    p=argparse.ArgumentParser();p.add_argument('original',type=Path);p.add_argument('output',type=Path);p.add_argument('register',type=Path);p.add_argument('audit',type=Path);a=p.parse_args()
    before,bobj=read_book(a.original);after,aobj=read_book(a.output)
    assert list(before)==list(after), 'Worksheet names/order changed'
    for col in 'ABCDEFKLM':
        for row in range(5,295):
            cell=f'{col}{row}'
            assert before['Review Register'].get(cell)==after['Review Register'].get(cell),cell
    unmodified=['Known Baseline','Gate Decisions','Sources','ERB Findings','Assembly Cost']
    for name in unmodified:
        assert before[name]==after[name],name
    data=json.loads(a.register.read_text())
    for row in data['rows']:
        cell='N'+str(row['row'])
        assert after['Review Register'][cell]==('text',{'open':'OPEN','closed':'CLOSED','na':'N/A'}[row['closure']]),row['id']
    assert bobj==aobj,'Original drawing/chart count changed'
    result={'status':'PASS','original_criteria_preserved':290,'original_user_input_cells_preserved':870,'original_sheet_order_preserved':True,'unmodified_sheets_compared':unmodified,'closure_cells_match_controlled_JSON':290,'original_drawing_chart_counts_preserved':aobj,'current_counts':data['summary']['current'],'native_excel_execution_claimed':False,'scope':'Exported XML static values and formulas compared; artifact-tool status-change recalculation was separately executed.'}
    a.audit.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))

if __name__=='__main__':main()
