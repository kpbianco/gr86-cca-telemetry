import fs from 'node:fs/promises';
import crypto from 'node:crypto';
import { FileBlob, SpreadsheetFile } from '@oai/artifact-tool';
const W=process.env.RVB22_RECOVERY || '/workspace/scratch/6686d1f90cd8/recovered/GR86_CCA_RevB_RVB22';
const input=process.env.RVB22_WORKBOOK_INPUT || '/workspace/scratch/6686d1f90cd8/outputs/rvb22_i21/GR86_CCA_RevB_ERB_CCB_Review_Register.xlsx';
const outputDir=process.env.RVB22_OUTPUT_DIR || '/workspace/scratch/6686d1f90cd8/outputs/rvb22_i22';
const audit=W+'/control/workbook_verification_i22';
const originalSheets=['Start Here','Review Register','Known Baseline','Gate Decisions','Sources','ERB Findings','Assembly Cost','Board Vetting','Current Deltas'];
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(input));
await fs.mkdir(audit,{recursive:true});
if(process.argv.includes('--help-font')){ console.log(wb.help('range.format.font',{include:'index,examples,notes',maxChars:3000}).ndjson);process.exit(0);}
if(process.argv.includes('--inspect')){
 const original=Object.fromEntries(originalSheets.map(name=>[name,{values:wb.worksheets.getItem(name).getRange('A1:N20').values,formulas:wb.worksheets.getItem(name).getRange('A1:N20').formulas}]));
 await fs.writeFile(audit+'/baseline_inspection.json',JSON.stringify(original,null,2));
 console.log((await wb.inspect({kind:'computedStyle',sheetId:'Start Here',range:'A1:H12',maxChars:2200})).ndjson);
 for(let i=0;i<originalSheets.length;i++){
  const name=originalSheets[i]; const blob=await wb.render({sheetName:name,range:name==='Review Register'?'A1:F6':'A1:H12',scale:1,format:'png'});
  await fs.writeFile(audit+'/baseline_'+String(i+1).padStart(2,'0')+'.png',new Uint8Array(await blob.arrayBuffer()));
 }
 console.log('Read-only baseline render complete: '+originalSheets.length+' sheets.');process.exit(0);
}
const contractPath=W+'/control/FINAL_REVIEW_REGISTER.json';
const contractBytes=await fs.readFile(contractPath); // Required before the first worksheet edit.
const data=JSON.parse(contractBytes);
const register=wb.worksheets.getItem('Review Register');
const preserved=JSON.stringify(register.getRange('A5:F294').values);
const preservedFormulas=JSON.stringify(register.getRange('A5:F294').formulas);
const preservedUserInputs=JSON.stringify(register.getRange('K5:M294').values);
const oldClosures=register.getRange('N5:N294').values.map(r=>r[0]);
const ids=register.getRange('A5:A294').values.map(r=>r[0]);
const rowMap=new Map(data.rows.map(r=>[r.id,r]));
if(rowMap.size!==290||data.rows.length!==290||ids.some(id=>!rowMap.has(id)))throw new Error('The controlled contract must cover all290 unique original IDs.');
const closureMap={closed:'CLOSED',open:'OPEN',na:'N/A'};
if(data.rows.some(r=>!closureMap[r.closure]))throw new Error('Unknown closure enum');
const body=ids.map(id=>rowMap.get(id));
const text=v=>v===null||v===undefined?'':typeof v==='object'?JSON.stringify(v):String(v);
const status=v=>text(v).replaceAll('_',' ');
const evidence=v=>Array.isArray(v)?v.map(text).join('\n'):text(v);
const counts={closed:body.filter(r=>r.closure==='closed').length,open:body.filter(r=>r.closure==='open').length,na:body.filter(r=>r.closure==='na').length};
const originalClosed=JSON.parse(await fs.readFile(W+'/control/review_input/USER_BASELINE.json','utf8')).counts.CLOSED;
// The exactly-once marker is run externally immediately before this authoring invocation.
register.getRange('G5:J294').values=body.map(r=>[status(r.evidence_status),'RVB22 coordinated engineering review',text(r.observation)+'\n\nCRITERION: '+closureMap[r.closure]+'\nDESKTOP: '+status(r.desktop_status)+'\nREMAINING: '+text(r.remaining),evidence(r.evidence)]);
register.getRange('N5:N294').values=body.map(r=>[closureMap[r.closure]]);
register.getRange('N5:N294').dataValidation={rule:{type:'list',values:['CLOSED','OPEN','N/A']}};
register.getRange('G5:G294').dataValidation={rule:{type:'list',values:[...new Set(body.map(r=>status(r.evidence_status)))]}};
register.getRange('I5:J294').format.wrapText=true;
register.getRange('A1').values=[['RVB22 current criterion disposition | original questions and required evidence preserved']];

const colors={navy:'#18394A',teal:'#147D8D',light:'#E8F2F5',green:'#DFF1E6',amber:'#FFF2CC',red:'#FDE3DF',ink:'#18394A',muted:'#516670'};
const newNames=['RVB22 Current','RVB22 Criteria','RVB22 Redlines','RVB22 Inputs','RVB22 Iterations','RVB22 Gates','RVB22 Evidence'];
const sheets=Object.fromEntries(newNames.map(n=>[n,wb.worksheets.getItem(n)]));
for(const sh of Object.values(sheets)){
 for(const t of [...sh.tables.items])t.delete();
 sh.getRange('A1:K2500').unmerge();
 sh.getRange('A1:K2500').clear({applyTo:'contents'});
 sh.getRange('A1:K2500').conditionalFormats.deleteAll();
}
function title(sh,last,titleText,note){
 sh.showGridLines=false;sh.getRange('A1:'+last+'1').merge();sh.getRange('A1').values=[[titleText]];
 sh.getRange('A1:'+last+'1').format={fill:colors.navy,font:{name:'Aptos',size:15,bold:true,color:'#FFFFFF'},rowHeight:34,verticalAlignment:'center'};
 sh.getRange('A2:'+last+'2').merge();sh.getRange('A2').values=[[note]];
 sh.getRange('A2:'+last+'2').format={font:{name:'Aptos',size:10,color:colors.muted},rowHeight:46,wrapText:true,verticalAlignment:'center'};
 sh.freezePanes.freezeRows(4);
}
function table(name,headers,rows,widths){
 const sh=sheets[name],last=String.fromCharCode(64+headers.length),end=4+rows.length;
 sh.getRange('A4:'+last+end).values=[headers,...rows];
 sh.getRange('A4:'+last+end).format={font:{name:'Aptos',size:10,color:colors.ink},wrapText:true,verticalAlignment:'top'};
 sh.getRange('A4:'+last+'4').format={fill:colors.teal,font:{bold:true,color:'#FFFFFF'},rowHeight:30};
 widths.forEach((w,i)=>sh.getRange(String.fromCharCode(65+i)+'1:'+String.fromCharCode(65+i)+end).format.columnWidth=w);
 rows.forEach((r,i)=>{const lines=Math.max(...r.map((v,j)=>text(v).split('\n').reduce((a,s)=>a+Math.max(1,Math.ceil(s.length/(widths[j]*.92))),0)));sh.getRange('A'+(i+5)+':'+last+(i+5)).format.rowHeight=Math.min(390,Math.max(30,lines*13+10));if(i%2===0)sh.getRange('A'+(i+5)+':'+last+(i+5)).format.fill=colors.light;});
 sh.tables.add('A4:'+last+end,true,name.replace(/ /g,'')+'Table');
 return sh;
}
function statusCF(sh,range){
 const r=sh.getRange(range);
 r.conditionalFormats.add('containsText',{text:'OPEN',format:{fill:colors.red,font:{bold:true,color:'#8B2720'}}});
 r.conditionalFormats.add('containsText',{text:'BLOCKED',format:{fill:colors.amber,font:{bold:true,color:'#7A5200'}}});
 r.conditionalFormats.add('containsText',{text:'CONDITIONAL',format:{fill:colors.amber}});
 r.conditionalFormats.add('containsText',{text:'CORRECTED',format:{fill:colors.green}});
 r.conditionalFormats.add('containsText',{text:'CLOSED',format:{fill:colors.green,font:{bold:true}}});
}
title(sheets['RVB22 Criteria'],'G','RVB22 I22 current criteria','Each original criterion retains its own closure. Desktop/model work and remaining native, physical or supplier conditions are explicit.');
const currentRows=body.map(r=>[r.id,closureMap[r.closure],status(r.desktop_status),status(r.evidence_status),text(r.observation),text(r.remaining),evidence(r.evidence)]);
table('RVB22 Criteria',['Criterion','Closure','Desktop status','Evidence status','Current observation','Remaining condition','Evidence paths'],currentRows,[15,14,30,30,82,72,88]);
statusCF(sheets['RVB22 Criteria'],'B5:D294');
title(sheets['RVB22 Redlines'],'F','RVB22 I22 redline dispositions','Findings and their actual corrections remain visible; a bounded model is identified separately from a native or supplier acceptance gate.');
const redRows=(data.redlines??[]).map(r=>[text(r.id),text(r.lane),status(r.status),text(r.finding),text(r.correction)+(r.remaining?'\nRemaining: '+text(r.remaining):''),evidence(r.evidence)]);
table('RVB22 Redlines',['Redline','Lane','Status','Finding','Correction / disposition','Evidence paths'],redRows,[23,20,32,78,100,90]);
statusCF(sheets['RVB22 Redlines'],'C5:C'+(4+redRows.length));
title(sheets['RVB22 Inputs'],'C','RVB22 I22 inputs and operating bounds','Values and model conditions are source inputs. Edit with traceability; changing an assumption can invalidate its dependent closure.');
const inputRows=[['User baseline criteria closed',originalClosed,'Original user baseline (73 closed), not the superseded I06 consolidation'],['Current checkpoint',text(data.checkpoint),'control/FINAL_REVIEW_REGISTER.json / checkpoint'],['Review date',text(data.date),'control/FINAL_REVIEW_REGISTER.json / date'],['Current PCB SHA256',text(data.PCB_sha256),'control/FINAL_REVIEW_REGISTER.json / PCB_sha256'],['Firmware manifest SHA256',text(data.firmware_manifest_sha256),'control/FINAL_REVIEW_REGISTER.json / firmware_manifest_sha256'],['Meaning of zero',text(data.meaning_of_zero),'control/FINAL_REVIEW_REGISTER.json / meaning_of_zero'],...(data.inputs??[]).map(r=>[text(r.item),typeof r.value==='number'?r.value:text(r.value),text(r.basis)])];
for(const [k,v]of Object.entries(data.summary??{})){if(k!=='iterations')inputRows.push(['Summary / '+k,typeof v==='number'?v:text(v),'control/FINAL_REVIEW_REGISTER.json / summary.'+k]);}
table('RVB22 Inputs',['Input / bound','Value','Basis / source'],inputRows,[39,90,100]);
inputRows.forEach((row,i)=>{if(typeof row[1]==='number')sheets['RVB22 Inputs'].getRange('B'+(i+5)).format.numberFormat=Number.isInteger(row[1])?'#,##0':/temperature/i.test(row[0])?'0.0':/\(V\)/.test(row[0])?'0.000000':'0.000';});
sheets['RVB22 Inputs'].getRange('B7').values=[[Date.parse(data.date)/86400000+25569]];sheets['RVB22 Inputs'].getRange('B7').format.numberFormat='yyyy-mm-dd';
title(sheets['RVB22 Gates'],'D','RVB22 I22 remaining gates','Gate actions identify the remaining qualification or native evidence. They do not erase completed source corrections or model results.');
const gateRows=(data.gates??[]).map(r=>[text(r.id),status(r.status),text(r.description),text(r.closure_action)]);
table('RVB22 Gates',['Gate','Status','Description','Closure action'],gateRows,[25,30,92,105]);statusCF(sheets['RVB22 Gates'],'B5:B'+(4+gateRows.length));
title(sheets['RVB22 Iterations'],'D','RVB22 revision history','Recorded source revisions and verification checkpoints; failed trials are retained in the redline and evidence files.');
const iterationRows=[];
for(const folder of []){
 const manifestPath=W+'/iterations/'+folder+'/CANDIDATE_MANIFEST.json';
 try{const m=JSON.parse(await fs.readFile(manifestPath,'utf8'));iterationRows.push([folder,folder.startsWith('I06')?'CURRENT HANDOFF':folder.startsWith('I04')?'ELECTRICAL BASIS':folder.startsWith('I05')?'SUPERSEDED DOCUMENT HANDOFF':'SUPERSEDED SOURCE','PCB SHA256: '+text(m.pcb_sha256??m.PCB_sha256)+'\nNative validation: '+text(m.native_validation??(m.native_refill_erc_drc_required?'NOT RUN — REQUIRED':'not specified'))+(m.integration_note?'\n'+text(m.integration_note):'')+(m.status?'\nDisposition: '+text(m.status):'')+(m.changed_schematic_text_objects!==undefined?'\nChanged schematic notes: '+m.changed_schematic_text_objects+'; changed electrical objects: '+m.changed_electrical_objects:''), 'iterations/'+folder+'/CANDIDATE_MANIFEST.json']);}catch{}
}
if(Array.isArray(data.summary?.iterations))for(const r of data.summary.iterations)iterationRows.push([text(r.id??r.iteration),text(r.status),text(r.description??r),evidence(r.evidence)]);
if(iterationRows.length===0)iterationRows.push(['RVB22','CURRENT REVIEW',text(data.summary),'control/FINAL_REVIEW_REGISTER.json']);
table('RVB22 Iterations',['Iteration','Effectivity','Source / verification record','Evidence'],iterationRows,[27,28,110,98]);

title(sheets['RVB22 Evidence'],'C','RVB22 I22 evidence hashes','One source reference per criterion. SHA256 values are supplied by the controlled consolidation; an absent hash is shown explicitly.');
const evidenceRows=body.flatMap(r=>(r.evidence??[]).map(path=>[r.id,text(path),text(r.evidence_sha256?.[path]??'HASH NOT PROVIDED')]));
table('RVB22 Evidence',['Criterion','Evidence path','SHA256'],evidenceRows,[17,102,75]);

const dashboard=sheets['RVB22 Current'];title(dashboard,'H','GR86 CCA I22 engineering review','Six criteria close in I22. Open criteria decrease from 152 to 146. Counts below calculate from the original review register.');
dashboard.getRange('A1:H42').format.columnWidth=17;
for(const [labelRange,valRange,label,formula]of [
 ['A4:B4','A5:B6','CRITERIA',"=COUNTA('Review Register'!$A$5:$A$294)"],
 ['C4:D4','C5:D6','CLOSED',"=COUNTIF('Review Register'!$N$5:$N$294,\"CLOSED\")"],
 ['E4:F4','E5:F6','OPEN',"=COUNTIF('Review Register'!$N$5:$N$294,\"OPEN\")"],
 ['G4:H4','G5:H6','N/A',"=COUNTIF('Review Register'!$N$5:$N$294,\"N/A\")"]]){
 dashboard.getRange(labelRange).merge();dashboard.getRange(labelRange.split(':')[0]).values=[[label]];dashboard.getRange(labelRange).format={fill:colors.teal,font:{bold:true,color:'#FFFFFF',size:10},rowHeight:25};
 dashboard.getRange(valRange).merge();dashboard.getRange(valRange.split(':')[0]).formulas=[[formula]];dashboard.getRange(valRange).format={fill:colors.light,font:{bold:true,color:colors.navy,size:25},numberFormat:'#,##0',horizontalAlignment:'center',verticalAlignment:'center',rowHeight:27};
}
const statusText=text(data.summary?.release_status??data.summary?.status??data.status??'ENGINEERING CANDIDATE — see explicit remaining gates').replaceAll('_',' ');
dashboard.getRange('A8:H9').merge();dashboard.getRange('A8').values=[[statusText]];dashboard.getRange('A8:H9').format={fill:colors.amber,font:{bold:true,color:'#775000',size:12},wrapText:true,rowHeight:25,verticalAlignment:'center'};
dashboard.getRange('A11:D11').merge();dashboard.getRange('A11').values=[['Change since user baseline']];dashboard.getRange('E11:H11').merge();dashboard.getRange('E11').formulas=[["=C5-'RVB22 Inputs'!B5"]];dashboard.getRange('E11:H11').format={numberFormat:'+0;-0;0',font:{bold:true,size:16},fill:colors.light,horizontalAlignment:'center'};
dashboard.getRange('E11:H11').conditionalFormats.add('cellIs',{operator:'lessThan',formula:0,format:{fill:colors.amber,font:{color:'#775000'}}});
dashboard.getRange('A13:H13').merge();dashboard.getRange('A13').values=[['HOW TO READ THIS PACKAGE']];dashboard.getRange('A13:H13').format={fill:colors.teal,font:{bold:true,color:'#FFFFFF'},rowHeight:26};
const guide=[['RVB22 Criteria','All 290 current observations, desktop dispositions and remaining conditions.'],['RVB22 Redlines','Every aggregated finding, correction, evidence reference and residual status.'],['RVB22 Inputs','Exact selected interfaces, numerical assumptions and source bounds.'],['RVB22 Gates','Concrete closure actions for native execution, physical correlation and supplier acceptance.'],['RVB22 Iterations','Source effectivity and preserved failed / superseded checkpoints.'],['RVB22 Evidence','Criterion-by-criterion source paths and supplied SHA256 identities.'],['Review Register','Original criterion wording in A–F; current evidence and closure in G–J/N.']];
guide.forEach((r,i)=>{const y=15+i;dashboard.getRange('A'+y+':C'+y).merge();dashboard.getRange('D'+y+':H'+y).merge();dashboard.getRange('A'+y).values=[[r[0]]];dashboard.getRange('D'+y).values=[[r[1]]];dashboard.getRange('A'+y+':H'+y).format={fill:i%2===0?colors.light:'#FFFFFF',wrapText:true,rowHeight:38,font:{size:10},verticalAlignment:'center'};});
dashboard.getRange('A22:H23').merge();dashboard.getRange('A22').values=[['A continuous CAD trace is evidence about intended copper connectivity. It does not prove the manufactured board, connector contact resistance, installed RF performance or a supplier process. Model conditions remain visible so a conditional result is not mistaken for an unrestricted release.']];dashboard.getRange('A22:H23').format={wrapText:true,font:{size:10,color:colors.muted},rowHeight:28};
let yr=25;
const boundInputs=(data.inputs??[]).filter(r=>/rail|reset|rf|thermal|temperature|sensor|power|startup|antenna/i.test(text(r.item))).slice(0,8);
if(boundInputs.length){dashboard.getRange('A25:H25').merge();dashboard.getRange('A25').values=[['SELECTED CURRENT INPUTS — full definitions in RVB22 Inputs']];dashboard.getRange('A25:H25').format={fill:colors.teal,font:{bold:true,color:'#FFFFFF'},rowHeight:27};yr=26;for(const r of boundInputs){dashboard.getRange('A'+yr+':C'+yr).merge();dashboard.getRange('D'+yr+':H'+yr).merge();dashboard.getRange('A'+yr).values=[[text(r.item)]];dashboard.getRange('D'+yr).values=[[typeof r.value==='number'?r.value:text(r.value)]];
if(typeof r.value==='number')dashboard.getRange('D'+yr).format.numberFormat=Number.isInteger(r.value)?'#,##0':/temperature/i.test(r.item)?'0.0':/\(V\)/.test(r.item)?'0.000000':'0.000';dashboard.getRange('A'+yr+':H'+yr).format={wrapText:true,rowHeight:Math.min(90,Math.max(35,Math.ceil(text(r.value).length/75)*14+12)),font:{size:10}};yr++;}}
const start=wb.worksheets.getItem('Start Here');start.getRange('A1').values=[['GR86 CCA I22 corrective engineering']];
const noteUpdates={A3:'RVB22 applies source corrections, bounded equivalent models and criterion-by-criterion review to the recovered engineering checkpoint. See RVB22 Current for the controlling review and RVB22 Gates for remaining actions.',A4:statusText,A9:'NET CRITERIA CLOSED SINCE USER BASELINE',A11:'I22: confirmed Adafruit 851/960; all 12 package orientation overlays; complete 1.71 mm height stack; 1,154 populated and 776 mated checks; via heating and board RF review. I20 CAD and firmware remain current.',A13:'POWER / RF: source-bound numerical results and exact component/route revisions are recorded in RVB22 Inputs and the evidence paths. Native tool execution and physical correlation are identified separately; passing finite cases are not unlimited guarantees.',A15:'FIRMWARE: current source tests, configuration and build disposition are recorded per criterion. The existing historical tabs do not establish the current firmware image or build acceptance.',A17:'OIL / INSTALLATION: exact sensor and harness bounds are explicit. Private-return, vehicle-ground and accessory conditions are retained in current evidence and gate actions.',A19:'ASSEMBLY / RELEASE: supplier process, filled/capped vias, component handling, mechanical fit and thermal installation requirements are controlled in current source and gates. Historical cost and qualification estimates are not current supplier acceptance.'};
for(const [cell,val]of Object.entries(noteUpdates))start.getRange(cell).values=[[val]];
start.getRange('G9').formulas=[["=C7-'RVB22 Inputs'!B5"]];
for(const name of ['Board Vetting','Current Deltas']){const sh=wb.worksheets.getItem(name);sh.getRange('A1').values=[['HISTORICAL / SUPERSEDED — '+name+' | Current review: RVB22 Current']];sh.getRange('A1:H1').format={fill:colors.amber,font:{bold:true,color:'#775000'},rowHeight:32};}
if(JSON.stringify(register.getRange('A5:F294').values)!==preserved||JSON.stringify(register.getRange('A5:F294').formulas)!==preservedFormulas)throw new Error('Original A–F criterion cells changed');
if(JSON.stringify(register.getRange('K5:M294').values)!==preservedUserInputs)throw new Error('Original user inputs changed');
wb.recalculate();
const beforeClosed=Number(dashboard.getRange('C5').values[0][0]);
const testRow=5+body.findIndex(r=>r.closure==='closed');
register.getRange('N'+testRow).values=[['OPEN']];
wb.recalculate();
if(Number(dashboard.getRange('C5').values[0][0])!==beforeClosed-1)throw new Error('Dashboard does not recalculate from source status');
register.getRange('N'+testRow).values=[['CLOSED']];
wb.recalculate();
if(Number(dashboard.getRange('C5').values[0][0])!==counts.closed || Number(dashboard.getRange('E5').values[0][0])!==counts.open || Number(dashboard.getRange('G5').values[0][0])!==counts.na)throw new Error('Dashboard count mismatch');
await fs.mkdir(outputDir,{recursive:true});
const verify={input_sha256:crypto.createHash('sha256').update(await fs.readFile(input)).digest('hex'),contract_sha256:crypto.createHash('sha256').update(contractBytes).digest('hex'),criterion_count:ids.length,expected_counts:counts,original_A_F_values_sha256:crypto.createHash('sha256').update(preserved).digest('hex'),preserved_A_F:true,preserved_K_M:true,recalculation_status_change_verified:true,updated_sheets:newNames,updated_columns:['G','H','I','J','N'],summary_values:dashboard.getRange('A4:H6').values,start_values:start.getRange('A6:H9').values};
await fs.writeFile(audit+'/AUTHORING_VERIFICATION.json',JSON.stringify(verify,null,2));
console.log((await wb.inspect({kind:'table',range:'RVB22 Current!A4:H11',maxChars:2400,tableMaxRows:8,tableMaxCols:8})).ndjson);
const errors=await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:100},summary:'Final formula error scan'});await fs.writeFile(audit+'/FORMULA_ERROR_SCAN.ndjson',errors.ndjson);console.log(errors.ndjson.slice(0,2000));
for(let i=0;i<originalSheets.length+newNames.length;i++){const name=[...originalSheets,...newNames][i];if(process.env.RVB22_RENDER_ONLY&&!process.env.RVB22_RENDER_ONLY.split(',').includes(name))continue;const range=name==='RVB22 Current'?'A1:H'+Math.max(23,yr-1):name==='Review Register'?'G4:N6':name==='RVB22 Criteria'?'A1:G8':name==='RVB22 Redlines'?'A1:F7':name==='RVB22 Inputs'?'A1:C8':name==='RVB22 Gates'?'A1:D8':name==='RVB22 Iterations'?'A1:D8':name==='RVB22 Evidence'?'A1:C8':'A1:H12';const blob=await wb.render({sheetName:name,range,scale:1,format:'png'});await fs.writeFile(audit+'/final_'+String(i+1).padStart(2,'0')+'.png',new Uint8Array(await blob.arrayBuffer()));}
await fs.writeFile(audit+'/FINAL_VALUES.json',JSON.stringify({dashboard:dashboard.getRange('A4:H11').values,counts,register_ids:ids,closures:register.getRange('N5:N294').values},null,2));
const exported=await SpreadsheetFile.exportXlsx(wb);const output=outputDir+'/GR86_CCA_RevB_ERB_CCB_Review_Register.xlsx';await exported.save(output);console.log(JSON.stringify({output,counts,criteria_preserved:true,sheets:originalSheets.length+newNames.length}));
