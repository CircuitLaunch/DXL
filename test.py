#!/usr/bin/env python3

import time

from DXL import *

try:
    port = DXLPort("/dev/ttyACM0")
except DXLException as x:
    print(x)
    exit()
except Exception as x:
    print(x)
    exit()

port.scan()

dxl = port.actuators[1]

if dxl == None:
    print("No actuators found")
    exit()

print(port.modelName(dxl.model))
print("\nEEPROM Table")
print("============")
print(f'Firmware version: {dxl.firmwareVersion}')
print(f'Return delay time: {dxl.returnDelayTime}')
print(f'cwAngleLimit: {dxl.cwAngleLimit}')
print(f'ccwAngleLimit: {dxl.ccwAngleLimit}')
print(f'temperatureLimit: {dxl.temperatureLimit}')
print(f'minVoltageLimit: {dxl.minVoltageLimit}')
print(f'maxVoltageLimit: {dxl.maxVoltageLimit}')
print(f'maxTorque: {dxl.maxTorque}')
print(f'statusReturnLevel: {dxl.statusReturnLevel}')
print(f'shutdown: {dxl.shutdown}')

if isinstance(dxl, DXL_MX):
    print(f'multiTurnOffset: {dxl.multiTurnOffset}')
    print(f'resolutionDivider: {dxl.resolutionDivider}')

print("\nRam")
print("===")
print(f'torqueEnable: {dxl.torqueEnable}')
print(f'led: {dxl.led}')

if isinstance(dxl, DXL_AX):
    print(f'cwComplianceMargin: {dxl.cwComplianceMargin}')
    print(f'ccwComplianceMargin: {dxl.ccwComplianceMargin}')
    print(f'cwComplianceSlope: {dxl.cwComplianceSlope}')
    print(f'ccwComplianceSlope: {dxl.ccwComplianceSlope}')

if isinstance(dxl, DXL_MX):
    print(f'dGain: {dxl.dGain}')
    print(f'iGain: {dxl.iGain}')
    print(f'pGain: {dxl.pGain}')

print(f'goalPosition: {dxl.goalPosition}')
print(f'movingSpeed: {dxl.movingSpeed}')
print(f'torqueLimit: {dxl.torqueLimit}')
print(f'presentPosition: {dxl.presentPosition}')
print(f'presentSpeed: {dxl.presentSpeed}')
print(f'presentVoltage: {dxl.presentVoltage}')
print(f'registered: {dxl.registered}')
print(f'moving: {dxl.moving}')
print(f'lock: {dxl.lock}')
print(f'punch: {dxl.punch}')

if isinstance(dxl, DXL_EX):
    print(f'sensedCurrent: {dxl.sensedCurrent}')

if isinstance(dxl, DXL_MX):
    print(f'realtimeTick: {dxl.realtimeTick}')
    if isinstance(dxl, DXL_MX64):
        print(f'current: {dxl.current}')
        print(f'torqueCtrlModeEnable: {dxl.torqueCtrlModeEnable}')
        print(f'goalTorque: {dxl.goalTorque}')
    print(f'goalAcceleration:{dxl.goalAcceleration}')


dxl.torqueEnable = 1
time.sleep(3)
dxl.goalPosition = dxl.cwAngleLimit+256
time.sleep(3)
dxl.goalPosition = dxl.ccwAngleLimit-256
time.sleep(3)
dxl.goalPosition = 2048
time.sleep(3)
dxl.torqueEnable = 0
