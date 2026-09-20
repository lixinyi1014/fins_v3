"""Compile and link the real Keil source list with Arm Compiler 6; never flash.

Uses an explicit STM32F407IG memory layout: Flash 1 MiB, DMA-visible SRAM 128 KiB.
Generated objects, map and firmware go only into --output.
"""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse
import json
import shlex
import subprocess
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument('--toolchain', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--project', default='MDK-ARM/RM_Frame_C.uvprojx')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
project = root / args.project
tree = ET.parse(project)
# Inherit the production project's miscellaneous flags; refuse a project that removes finite checks.
misc = tree.find('.//Cads/VariousControls/MiscControls')
project_flags = shlex.split(misc.text or '') if misc is not None else []
for required in ('-fno-fast-math', '-fno-finite-math-only'):
    if required not in project_flags:
        raise SystemExit('Keil project must retain ' + required)
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=True)
includes = [(project.parent / p.replace('\\', '/')).resolve()
            for p in tree.find('.//Cads/VariousControls/IncludePath').text.split(';')]
sources = sorted(set((project.parent / e.text.replace('\\', '/')).resolve()
                     for e in tree.findall('.//Groups/Group/Files/File/FilePath')))
sources = [p for p in sources if p.suffix.lower() in ('.c', '.cpp', '.s')]

def compile_source(item):
    index, source = item
    obj = args.output / ('%03d_%s.o' % (index, source.stem))
    if source.suffix.lower() == '.s':
        command = [str(args.toolchain / 'armasm.exe'), '--cpu=Cortex-M4.fp',
                   '--apcs=interwork', '-g', str(source), '-o', str(obj)]
    else:
        command = [str(args.toolchain / 'armclang.exe'), '--target=arm-arm-none-eabi',
                   '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
                   '-fshort-enums', '-fshort-wchar', '-ffunction-sections', '-fdata-sections',
                   '-O2', '-fno-fast-math', '-fno-finite-math-only',
                   '-g', '-DUSE_HAL_DRIVER', '-DSTM32F407xx',
                   '-std=c++11' if source.suffix == '.cpp' else '-std=c99']
        command += project_flags
        command += ['-I' + str(p) for p in includes]
        command += ['-c', str(source), '-o', str(obj)]
    result = subprocess.run(command, capture_output=True)
    return {'file': str(source.relative_to(root)), 'object': str(obj),
            'exit_code': result.returncode,
            'output': (result.stdout + result.stderr).decode('utf-8', errors='replace')}

with ThreadPoolExecutor(max_workers=4) as pool:
    results = list(pool.map(compile_source, enumerate(sources)))
report = {'project_flags': project_flags, 'purpose': 'offline verification; release uses actual Keil build',
          'compiled': len(results), 'failed': sum(r['exit_code'] != 0 for r in results),
          'results': results}
for item in results:
    if item['exit_code']:
        print(item['file'], item['output'][-5000:])
if not report['failed']:
    scatter = args.output / 'firmware.sct'
    scatter.write_text('''LR_IROM1 0x08000000 0x00100000 {
  ER_IROM1 0x08000000 0x00100000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000 {
    .ANY (+RW +ZI)
  }
}
''', encoding='ascii')
    command = [str(args.toolchain / 'armlink.exe'), '--cpu=Cortex-M4.fp',
               '--scatter=' + str(scatter), '--map', '--symbols', '--info=sizes,totals,unused',
               '--list=' + str(args.output / 'firmware.map'),
               '--output=' + str(args.output / 'firmware.axf')]
    command += [item['object'] for item in results]
    linked = subprocess.run(command, capture_output=True)
    report['link_exit_code'] = linked.returncode
    report['link_output'] = (linked.stdout + linked.stderr).decode('utf-8', errors='replace')
    print(report['link_output'])
(args.output / 'build-report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
print(json.dumps({k: v for k, v in report.items() if k != 'results'}))
raise SystemExit(1 if report['failed'] or report.get('link_exit_code', 1) else 0)
