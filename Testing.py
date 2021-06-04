TEST_DXLPort_scan = False
TEST_DXLPort_modelName = False
TEST_DXLPort_readUInt8 = False
TEST_DXLPort_readUInt16 = False
TEST_DXLPort_writeUInt8 = False
TEST_DXLPort_writeUInt16 = False
TEST_DXLPort_syncWrite = False
TEST_DXL_EEPROM_getters = False
TEST_DXL_BASE_RAM_getters = True
TEST_DXL_BASE_RAM_setters = False
TEST_DXL_20 = False

import sys
from select import select

def kbhit():
    dr, dw, de = select([sys.stdin],[],[],0)
    return dr != []

import time
from DXL import *

port = DXLPort("/dev/serial/by-id/usb-Xevelabs_USB2AX_74031303437351011190-if00", 1000000)

def callback(dxl, result, error):
    if result != 0 or error != 0:
        print(f"dxl_{dxl.id} comm {port.resultString(result)}, error [{port.errorString(error)}]")

if TEST_DXLPort_scan:
    port.scan()


rightArmIds = [10, 11, 12, 13, 14, 15, 16, 17]
leftArmIds = [20, 21, 22, 23, 24, 25, 26, 27]

dxls = { i:port.getDXL(i, callback) for i in rightArmIds + leftArmIds }

if TEST_DXLPort_modelName:
    for id in rightArmIds:
        print(f'Right arm: dxl_{id}.model: {port.modelName(dxls[id].model)}')

    for id in leftArmIds:
        print(f'Right arm: dxl_{id}.model: {port.modelName(dxls[id].model)}')

if TEST_DXLPort_readUInt8:
    for id in rightArmIds:
        print(f'Right arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)}')

    for id in leftArmIds:
        print(f'Right arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)}')

if TEST_DXLPort_readUInt16:
    for id in rightArmIds:
        print(f'Right arm: dxl_{id}.presentPosition: {port.readUInt16(id, RAM_PRESENT_POSITION)[0]}')

    for id in leftArmIds:
        print(f'Right arm: dxl_{id}.presentPosition: {port.readUInt16(id, RAM_PRESENT_POSITION)[0]}')

if TEST_DXLPort_writeUInt8:
    for id in rightArmIds:
        print(f'Right arm writing 1 to dxl_{id}.portEnable')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 1)
        print(f'Right arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)[0]}')
        print(f'Right arm writing 0 to dxl_{id}.portEnable')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 0)
        print(f'Right arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)[0]}')

    for id in leftArmIds:
        print(f'Left arm writing 1 to dxl_{id}.portEnable')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 1)
        print(f'Left arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)[0]}')
        print(f'Left arm writing 0 to dxl_{id}.portEnable')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 0)
        print(f'Left arm: dxl_{id}.torqueEnable: {port.readUInt8(id, RAM_TORQUE_ENABLE)[0]}')

if TEST_DXLPort_writeUInt16:
    for id in rightArmIds:
        presentPosition, _, _ = port.readUInt16(id, RAM_PRESENT_POSITION)
        print(f'Right arm writing {presentPosition} to dxl_{id}.goalPosition')
        port.writeUInt16(id, RAM_GOAL_POSITION, presentPosition)
        print(f'Right arm: dxl_{id}.goalPosition: {port.readUInt16(id, RAM_GOAL_POSITION)[0]}')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 0)

    for id in leftArmIds:
        presentPosition, _, _ = port.readUInt16(id, RAM_PRESENT_POSITION)
        print(f'Left arm writing {presentPosition} to dxl_{id}.goalPosition')
        port.writeUInt16(id, RAM_GOAL_POSITION, presentPosition)
        print(f'Left arm: dxl_{id}.goalPosition: {port.readUInt16(id, RAM_GOAL_POSITION)[0]}')
        port.writeUInt8(id, RAM_TORQUE_ENABLE, 0)

if TEST_DXLPort_syncWrite:
    print('Writing 1 to torqueEnable for all actuators')
    result = port.syncWrite(RAM_TORQUE_ENABLE, 1, dict(zip(rightArmIds, [1] * len(rightArmIds))))
    result = port.syncWrite(RAM_TORQUE_ENABLE, 1, dict(zip(leftArmIds, [1] * len(leftArmIds))))
    print('Sleeping for 5 seconds')
    time.sleep(5.0)
    print('Writing 0 to torqueEnable for all actuators')
    result = port.syncWrite(RAM_TORQUE_ENABLE, 1, dict(zip(rightArmIds, [0] * len(rightArmIds))))
    result = port.syncWrite(RAM_TORQUE_ENABLE, 1, dict(zip(leftArmIds, [0] * len(leftArmIds))))

if TEST_DXL_EEPROM_getters:
    for id in rightArmIds + leftArmIds:
        dxl = dxls[id]
        print(f'dxl_{id} ----------------------------------------')
        print(f' .stepResolution = {dxl.stepResolution}')
        print(f' .centerOffset = {dxl.centerOffset}')
        print(f' .firmwareVersion = {dxl.firmwareVersion}')
        print(f' .returnDelayTime = {dxl.returnDelayTime}')
        print(f' .cwAngleLimit = {dxl.cwAngleLimit}')
        print(f' .ccwAngleLimit = {dxl.ccwAngleLimit}')
        print(f' .temperatureLimit = {dxl.temperatureLimit}')
        print(f' .minVoltageLimit = {dxl.minVoltageLimit}')
        print(f' .maxVoltageLimit = {dxl.maxVoltageLimit}')
        print(f' .maxTorque = {dxl.maxTorque}')
        print(f' .statusReturnLevel = {dxl.statusReturnLevel}')
        print(f' .shutdown = {port.errorString(dxl.shutdown)}')

if TEST_DXL_BASE_RAM_getters:
    for id in rightArmIds + leftArmIds:
        dxl = dxls[id]
        print(f'dxl_{id} ----------------------------------------')
        print(f' .alarmLED = {dxl.alarmLED}')
        print(f' .torqueEnable = {dxl.torqueEnable}')
        print(f' .led = {dxl.led}')
        print(f' .goalPosition = {dxl.goalPosition}')
        print(f' .movingSpeed = {dxl.movingSpeed}')
        print(f' .torqueLimit = {dxl.torqueLimit}')
        print(f' .presentPosition = {dxl.presentPosition}')
        print(f' .presentLoad = {dxl.presentLoad}')
        print(f' .presentTemperature = {dxl.presentTemperature}')
        print(f' .registered = {dxl.registered}')
        print(f' .moving = {dxl.moving}')
        print(f' .lock = {dxl.lock}')
        print(f' .punch = {dxl.punch}')

if TEST_DXL_BASE_RAM_setters:
    for id in rightArmIds + leftArmIds:
        dxl = dxls[id]
        print(f'dxl_{id} ----------------------------------------')
        alarmLED = dxl.alarmLED
        print(f' .alarmLED (original) = {alarmLED}')
        print(' setting alarmLED to 1')
        dxl.alarmLED = 1
        print(f' .alarmLED (current) = {dxl.alarmLED}')
        print(f' setting alarmLED back to {alarmLED}')
        dxl.alarmLED = alarmLED
        print(f' .alarmLED (current) = {dxl.alarmLED}')
        print('---')
        torqueEnable = dxl.torqueEnable
        print(f' .torqueEnable (original) = {torqueEnable}')
        print(' setting torqueEnable to 1')
        dxl.torqueEnable = 1
        print(f' .torqueEnable (current) = {dxl.torqueEnable}')
        print(f' setting torqueEnable back to {torqueEnable}')
        dxl.torqueEnable = torqueEnable
        print(f' .torqueEnable (current) = {dxl.torqueEnable}')
        print('---')
        led = dxl.led
        print(f' .led (original) = {led}')
        print(' setting led to 1')
        dxl.led = 1
        print(f' .led (current) = {dxl.led}')
        print(f' setting led back to {led}')
        dxl.led = led
        print(f' .led (current) = {dxl.led}')
        print('---')
        dxl.goalPosition = dxl.presentPosition
        goalPosition = dxl.goalPosition
        print(f'.goalPosition (original) = {goalPosition}')
        print(f' setting goalPosition to {goalPosition + 5}')
        dxl.goalPosition = goalPosition + 5
        time.sleep(0.5)
        print(f' .goalPosition (current) = {dxl.goalPosition}')
        print(f' setting goalPosition back to {goalPosition}')
        dxl.goalPosition = goalPosition
        time.sleep(0.5)
        print(f' .goalPosition (current) = {dxl.goalPosition}')
        dxl.torqueEnable = 0
        print('---')
        torqueLimit = dxl.torqueLimit
        print(f' .torqueLimit (original) = {torqueLimit}')
        print(' setting torqueLimit to 511')
        dxl.torqueLimit = 511
        print(f' .torqueLimit (current) = {dxl.torqueLimit}')
        print(f' setting torqueLimit back to {torqueLimit}')
        dxl.torqueLimit = torqueLimit
        print(f' .torqueLimit (current) = {dxl.torqueLimit}')
        '''
        print(f' .movingSpeed = {dxl.movingSpeed}')
        '''

if TEST_DXL_20:
    dxl20 = dxls[20]
    presentPosition = dxl20.presentPosition
    goalPosition = dxl20.goalPosition
    print('Enable torque on dxl_20')
    dxl20.enableTorque = 1
    print(f'dxl_20.presentPosition: {presentPosition}')
    print(f'dxl_20.goalPosition: {goalPosition}')
    print(f'dxl_20.offset: {dxl20.offset}')
    print(f'dxl_20.centerOffset: {dxl20.centerOffset}')
    print(f'dxl_20.cwAngleLimit: {dxl20.cwAngleLimit}')
    print(f'dxl_20.ccwAngleLimit: {dxl20.ccwAngleLimit}')
    print(f'Press return to set dxl_20.goalPosition to {presentPosition}')
    while(1):
        if kbhit():
            ch = sys.stdin.read(1)
            break
    print(f'Setting dxl_20.goalPosition to {presentPosition}')
    dxl20.goalPosition = presentPosition
    print('Waiting 1 second')
    time.sleep(1.0)
    presentPosition = dxl20.presentPosition
    goalPosition = dxl20.goalPosition
    print(f'dxl_20.presentPosition: {presentPosition}')
    print(f'dxl_20.goalPosition: {goalPosition}')
    print('Press return to disable torque on dxl_20')
    while(1):
        if kbhit():
            ch = sys.stdin.read(1)
            break
    dxl20.torqueEnable = 0
