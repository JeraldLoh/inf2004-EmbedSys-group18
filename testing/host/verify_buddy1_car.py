"""Buddy 1 PC regression checks, independent of unfinished team/B4 tests."""
from pathlib import Path
import subprocess
import os
ROOT=Path(__file__).resolve().parents[2]
os.chdir(ROOT)
OUT=ROOT/'build/buddy1-car-host'
OUT.mkdir(parents=True,exist_ok=True)
base='app_program/subsystem1/'
bridge=[base+'buddy1_bridge.c',base+'buddy1_protocol.c',base+'buddy1_binary.c',base+'buddy1_diagnostics.c']
def check(name,sources,flags=(),args=()):
 exe=OUT/name
 subprocess.run(['gcc','-std=c11','-Wall','-Wextra','-Werror','-Wpedantic','-UNDEBUG',*flags,*sources,'-o',str(exe)],check=True)
 subprocess.run([str(exe),*map(str,args)],check=True)
check('car',['testing/host/test_buddy1_car.c',*bridge],['-Itesting/host/buddy1_stubs','-DAPP_CAR_MQTT=1'],[OUT/'car.hex'])
check('protocol',['host_tests/protocol_test.c',base+'buddy1_protocol.c'],['-Iapp_program/subsystem1'])
check('binary',['host_tests/binary_test.c',base+'buddy1_binary.c'],args=[OUT/'golden.bin'])
assert (OUT/'golden.bin').read_bytes().hex()==(ROOT/'host_tests/binary-golden-v3.hex').read_text().strip()
check('diagnostics',['host_tests/diagnostics_test.c',base+'buddy1_binary.c',base+'buddy1_diagnostics.c'])
check('retry',['host_tests/retry_test.c'],['-Ilib/libwifi/sysdepend/pico_rp2040'])
for mode in (0,1):
 check('lifecycle'+str(mode),['testing/host/test_buddy1_lifecycle.c',base+'subsystem1.c','app_program/app_main.c'],['-Itesting/host/buddy1_stubs','-DB1_STANDALONE_DEMO='+str(mode)])
for delay in (0,5020,5100):
 check('heartbeat'+str(delay),['testing/host/test_buddy1_heartbeat.c',*bridge],['-Itesting/host/buddy1_stubs'],[delay])
check('real_snapshot',['testing/host/test_buddy1_heartbeat.c',*bridge],['-Itesting/host/buddy1_stubs','-DB1_DEMO=0'],[0])
controller=['app_program/contracts/hardware_faults.c','app_program/vehicle_controller/safety.c','app_program/vehicle_controller/vehicle_controller.c','app_program/vehicle_controller/controller_service.c']
check('remote_start',['testing/host/test_remote_start.c',*controller])
check('controller_service',['testing/host/test_controller_service.c',*controller])
print('PASS Buddy 1 portable suites; hardware not tested.')
