"""Portable host checks. No firmware flashing, robot commands or network access."""
from pathlib import Path
import os
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
os.chdir(ROOT)
OUT = ROOT/'testing/host/out/binary'
OUT.mkdir(parents=True, exist_ok=True)
CC = shutil.which('gcc') or 'gcc'
FLAGS = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-Wpedantic', '-UNDEBUG']
EXT = '.exe' if os.name == 'nt' else ''

def run(args):
    print('+', ' '.join(map(str,args)), flush=True)
    subprocess.run(list(map(str,args)), cwd=ROOT, check=True)

def build(name, sources, extra=()):
    exe=OUT/(name+EXT)
    run([CC,*FLAGS,*extra,*sources,'-o',exe])
    return exe

def main():
    binary='app_program/subsystem1/buddy1_binary.c'
    bridge='app_program/subsystem1/buddy1_bridge.c'
    diagnostics='app_program/subsystem1/buddy1_diagnostics.c'
    protocol='app_program/subsystem1/buddy1_protocol.c'
    run([sys.executable,ROOT/'tools/generate_buddy1_binary.py'])
    run([build('diagnostics',['host_tests/diagnostics_test.c',binary,diagnostics])])
    exe=build('binary',['host_tests/binary_test.c',binary])
    fixture=OUT/'golden.bin';run([exe,fixture])
    assert fixture.read_bytes().hex() == (ROOT/'host_tests/binary-golden-v3.hex').read_text().strip(), 'C/JS golden vector mismatch'
    for name,sources,extra in [
        ('protocol',['host_tests/protocol_test.c',protocol],['-Iapp_program/subsystem1']),
        ('retry',['host_tests/retry_test.c'],['-Ilib/libwifi/sysdepend/pico_rp2040']),
        ('controller',['testing/host/test_vehicle_controller.c','app_program/vehicle_controller/safety.c','app_program/vehicle_controller/vehicle_controller.c'],[]),
        ('motion',['testing/host/test_motion_math.c','app_program/subsystem2/motion_math.c'],[]),
        ('perception',['testing/host/test_perception_math.c','app_program/subsystem3/barcode_decoder.c','app_program/subsystem4/lsm303_decode.c'],[]),
    ]:
        run([build(name,sources,extra)])
    for mode in (0,1):
        run([build('lifecycle'+str(mode),['testing/host/test_buddy1_lifecycle.c','app_program/subsystem1/subsystem1.c','app_program/app_main.c'],['-Itesting/host/buddy1_stubs','-DB1_STANDALONE_DEMO='+str(mode)])])
    sources=['testing/host/test_buddy1_heartbeat.c',bridge,protocol,binary,diagnostics]
    exe=build('heartbeat',sources,['-Itesting/host/buddy1_stubs'])
    for until in (0,5020,5100):run([exe,until])
    run([build('real_snapshot',sources,['-Itesting/host/buddy1_stubs','-DB1_DEMO=0']),0])
    if '--c-only' not in sys.argv:
        run([shutil.which('node') or 'node','--test','dashboard/web/test.mjs','dashboard/web/binary.test.mjs','dashboard/web/telemetry-view.test.mjs','dashboard/web/calibration.test.mjs','dashboard/web/diagnostics.test.mjs','dashboard/web/buddy2.test.mjs'])
    run([sys.executable,'-B','-m','unittest','discover','-s','dashboard','-p','test*.py'])
    print('PASS: requested portable suites; hardware integration not tested.')

if __name__=='__main__':main()
