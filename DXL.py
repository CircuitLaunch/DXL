# Copyright 2021 Edward Janne
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

################################################################################
#
# Object oriented Python Wrapper for Dynamixel Protocol 1.0 API.
#
# This module is thread safe within the same Python Kernel.
# This module is NOT multi-process safe.
#
################################################################################

################################################################################
# Import dependencies
import dynamixel_sdk as sdk
import time
from typing import List
from threading import Lock
import inspect

################################################################################
# These codes are available in the EEPROM & RAM Control Table listings for each
# device at https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#supported-dynamixel

################################################################################
# Model codes returned by the ping command
MODEL_NUMBER_AX12A = 12
MODEL_NUMBER_AX12W = 300
MODEL_NUMBER_AX18A = 18
MODEL_NUMBER_EX106 = 107
MODEL_NUMBER_RX10 = 10
MODEL_NUMBER_RX24F = 24
MODEL_NUMBER_RX28 = 28
MODEL_NUMBER_RX64 = 64
MODEL_NUMBER_MX12W = 360
MODEL_NUMBER_MX28 = 29
MODEL_NUMBER_MX64 = 310
MODEL_NUMBER_MX106 = 320
MODEL_NUMBER_MX28_2 = 30
MODEL_NUMBER_MX64_2 = 311
MODEL_NUMBER_MX106_2 = 321

################################################################################
# EEPROM register addresses
EEPROM_FIRMWARE_VERSION = 2
EEPROM_RETURN_DELAY_TIME = 5
EEPROM_CW_ANGLE_LIMIT = 6
EEPROM_CCW_ANGLE_LIMIT = 8
EEPROM_TEMPERATURE_LIMIT = 11
EEPROM_MIN_VOLTAGE_LIMIT = 12
EEPROM_MAX_VOLTAGE_LIMIT = 13
EEPROM_MAX_TORQUE = 14
EEPROM_STATUS_RETURN_LEVEL = 16
EEPROM_ALARM_LED = 17
EEPROM_SHUTDOWN = 18

MX_EEPROM_MULTI_TURN_OFFSET = 20
MX_EEPROM_RESOLUTION_DIVIDER = 22

################################################################################
# RAM register addresses
RAM_TORQUE_ENABLE = 24
RAM_LED = 25
RAM_GOAL_POSITION = 30
RAM_MOVING_SPEED = 32
RAM_TORQUE_LIMIT = 34
RAM_PRESENT_POSITION = 36
RAM_PRESENT_SPEED = 38
RAM_PRESENT_LOAD = 40
RAM_PRESENT_VOLTAGE = 42
RAM_PRESENT_TEMPERATURE = 43
RAM_REGISTERED = 44
RAM_MOVING = 46
RAM_LOCK = 47
RAM_PUNCH = 48

AX_RAM_CW_COMPLIANCE_MARGIN = 26
AX_RAM_CCW_COMPLIANCE_MARGIN = 27
AX_RAM_CW_COMPLIANCE_SLOPE = 28
AX_RAM_CCW_COMPLIANCE_SLOPE = 29

EX_RAM_SENSED_CURRENT = 56

MX_RAM_D_GAIN = 26
MX_RAM_I_GAIN = 27
MX_RAM_P_GAIN = 28
MX_RAM_REALTIME_TICK = 50
MX_RAM_GOAL_ACCELERATION = 73

MX64_RAM_CURRENT = 68
MX64_RAM_TORQUE_CTRL_MODE_ENABLE = 70
MX64_RAM_GOAL_TORQUE = 71

################################################################################
# Flags for shutdown and alarmLED registers
ERROR_BIT_VOLTAGE = 1
ERROR_BIT_ANGLE_LIMIT = 2
ERROR_BIT_OVERHEATING = 4
ERROR_BIT_RANGE = 8
ERROR_BIT_CHECKSUM = 16
ERROR_BIT_OVERLOAD = 32
ERROR_BIT_INSTRUCTION = 64

################################################################################
# Dictionaries to translate model codes to and from human readable form
gModelStrToCodeDict = {
    'AX12A': MODEL_NUMBER_AX12A,
    'AX12W': MODEL_NUMBER_AX12W,
    'AX18A': MODEL_NUMBER_AX18A,
    'EX106': MODEL_NUMBER_EX106,
    'RX10': MODEL_NUMBER_RX10,
    'RX24F': MODEL_NUMBER_RX24F,
    'RX28': MODEL_NUMBER_RX28,
    'RX64': MODEL_NUMBER_RX64,
    'MX12W': MODEL_NUMBER_MX12W,
    'MX28': MODEL_NUMBER_MX28,
    'MX64': MODEL_NUMBER_MX64,
    'MX106': MODEL_NUMBER_MX106,
    'MX28_2': MODEL_NUMBER_MX28_2,
    'MX64_2': MODEL_NUMBER_MX64_2,
    'MX106_2': MODEL_NUMBER_MX106_2,
}

gModelCodeToStrDict = {
    MODEL_NUMBER_AX12A: 'AX12A',
    MODEL_NUMBER_AX12W: 'AX12W',
    MODEL_NUMBER_AX18A: 'AX18A',
    MODEL_NUMBER_RX10: 'RX10',
    MODEL_NUMBER_EX106: 'EX106',
    MODEL_NUMBER_RX24F: 'RX24F',
    MODEL_NUMBER_RX28: 'RX28',
    MODEL_NUMBER_RX64: 'RX64',
    MODEL_NUMBER_MX12W: 'MX12W',
    MODEL_NUMBER_MX28: 'MX28',
    MODEL_NUMBER_MX64: 'MX64',
    MODEL_NUMBER_MX106: 'MX106',
    MODEL_NUMBER_MX28_2: 'MX28_2',
    MODEL_NUMBER_MX64_2: 'MX64_2',
    MODEL_NUMBER_MX106_2: 'MX106_2',
}

############################################################################
# Result code and error bit dictionaries
gResultCodeDescriptors = {
    sdk.COMM_SUCCESS: 'OK',
    sdk.COMM_PORT_BUSY: 'PORT_BUSY',
    sdk.COMM_TX_FAIL: 'TX_FAIL',
    sdk.COMM_RX_FAIL: 'RX_FAIL',
    sdk.COMM_TX_ERROR: 'TX_ERROR',
    sdk.COMM_RX_WAITING: 'RX_WAITING',
    sdk.COMM_RX_TIMEOUT: 'RX_TIMEOUT',
    sdk.COMM_RX_CORRUPT: 'RX_CORRUPT',
    sdk.COMM_NOT_AVAILABLE: 'NOT_AVAILABLE'
}

gErrorBitDescriptors = ['INSTRUCTION', 'OVERLOAD', 'CHECKSUM', 'RANGE', 'OVERHEAT', 'ANGLE', 'VOLTAGE']

################################################################################
# Exception subclass
class DXLException(Exception):
    def __init__(self, msg):
        self.msg = msg
        frame = inspect.stack()[1][0]
        info = inspect.getframeinfo(frame)
        self.file = info.filename
        self.function = info.function
        self.line = info.lineno

    def __str__(self):
        return f"DXL Exception caught on line {self.line} of {self.file}: {self.msg}"

################################################################################
# Class to manage the connection to the Dynamixel USB controller
class DXLPort:
    ############################################################################
    # Class method to return string model name given model code
    # Returns None if code does not correspond to a known model
    @staticmethod
    def modelName(code: int):
        if code in gModelCodeToStrDict:
            return gModelCodeToStrDict[code]
        return None

    ############################################################################
    # Class method to return model code given model string name
    # Returns None if name does not correspond to a known model
    @staticmethod
    def modelCode(name: str):
        if name in gModelStrToCodeDict:
            return gModelStrToCodeDict[name]
        return None

    ############################################################################
    # Constructor
    # Pass in the port name, and the baud rate
    def __init__(self, address: str, baud: int = 57600):
        self.lock = Lock()
        self.baud = baud
        self.port = None
        port = sdk.PortHandler(address)
        self.actuators = {}
        self.actuatorLock = Lock()
        self.syncRegister = None
        self.syncDataLen = 0
        self.syncLock = Lock()
        self.syncEncoder = None
        self.syncReadDXLs = None
        self.syncWriteCount = 0

        try:
            port.openPort()
        except Exception as x:
            raise(DXLException(str(x)))

        self.port = port

        try:
            self.port.setBaudRate(self.baud)
        except Exception as x:
            raise(DXLException(str(x)))

        self.packetHandler = sdk.PacketHandler(1.0)

    ############################################################################
    # Destructor
    def __del__(self):
        if self.port != None:
            with self.lock:
                self.port.closePort()

    ############################################################################
    # Scans for connected actuators and lists their model numbers and ids
    def scan(self):
        for i in range(254):
            print(f'Pinging {i}')
            actuator = self.getDXL(i)
            if actuator != None:
                print(f'Found: {i}')
                self.actuators[i] = actuator

    ############################################################################
    # Returns a DXL representing the actuator with the given id
    # If one was previously instantiated, a reference to that one is returned
    # If no actuator with that id exists, returns None
    def getDXL(self, id: int, callback = None):
        with self.actuatorLock:
            if id in self.actuators.keys():
                dxl = self.actuators[id]
                with dxl.lock:
                    if callback == None: dxl.callback = callback
                return dxl

        model, result, error = self.ping(id)
        if model in [MODEL_NUMBER_AX12A, MODEL_NUMBER_AX12W, MODEL_NUMBER_AX18A, MODEL_NUMBER_RX10, MODEL_NUMBER_RX24F, MODEL_NUMBER_RX28, MODEL_NUMBER_RX64]:
            dxl = DXL_AX(self, id, model, callback)
        elif model == MODEL_NUMBER_EX106:
            dxl = DXL_EX(self, id, model, callback)
        elif model in [MODEL_NUMBER_MX12W, MODEL_NUMBER_MX28]:
            dxl = DXL_MX(self, id, model, callback)
        elif model in [MODEL_NUMBER_MX64, MODEL_NUMBER_MX106]:
            dxl = DXL_MX64(self, id, model, callback)
        else:
            return None

        with self.actuatorLock:
            self.actuators[id] = dxl

        return dxl

    ############################################################################
    # Disables torque, sets the goal position to the present position, sets
    # torque_limit to maximum, and then re-enables torque; this is the
    # recommended way to recover from an overload error; may not work reliably
    # on AX models
    def recover(self, id: int):
        with self.syncLock:
            self.writeUInt8(id, RAM_TORQUE_ENABLE, 0)
            time.sleep(1.0)
            presentPosition, _, _ = self.readUInt16(id, RAM_PRESENT_POSITION)
            self.writeUInt16(id, RAM_GOAL_POSITION, presentPosition)
            self.writeUInt16(id, RAM_TORQUE_LIMIT, 1023)
            self.writeUInt8(id, RAM_TORQUE_ENABLE, 1)

    ############################################################################
    # Returns a result string given a result code
    def resultString(self, result: int):
        if result in gResultCodeDescriptors:
            return gResultCodeDescriptors[result]
        return "UNKNOWN_ERROR"

    ############################################################################
    # Returns a space delimited list of error strings based on the error flags
    # set in the error code
    def errorString(self, error: int):
        bit = 1
        errorStr = ""
        for errorBitDescriptor in gErrorBitDescriptors:
            if error & bit:
                errorStr += ('' if errorStr == '' else ' ') + errorBitDescriptor
            bit <<= 1
        return errorStr

    ############################################################################
    # Sends a ping instruction.
    # If successful, returns the model code, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def ping(self, id: int):
        with self.lock:
            model, result, error = self.packetHandler.ping(self.port, id)
        return model, result, error

    ############################################################################
    # Reads from a byte register
    # If successful, returns the value, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def readUInt8(self, id: int, register: int):
        with self.lock:
            value, result, error = self.packetHandler.read1ByteTxRx(self.port, id, register)
        return value, result, error

    ############################################################################
    # Reads from a two byte register
    # If successful, returns the value, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def readUInt16(self, id: int, register: int):
        with self.lock:
            value, result, error = self.packetHandler.read2ByteTxRx(self.port, id, register)
        return value, result, error

    ############################################################################
    # Writes to a byte register
    # If successful, returns COMM_SUCCESS, 0
    # If unsuccessful, returns the result code, and an error code
    def writeUInt8(self, id: int, register: int, value: int):
        with self.lock:
            result, error = self.packetHandler.write1ByteTxRx(self.port, id, register, value)
        return result, error

    ############################################################################
    # Writes tp a two byte register
    # If successful, returns COMM_SUCCESS, 0
    # If unsuccessful, returns the result code, and an error code
    def writeUInt16(self, id: int, register: int, value: int):
        with self.lock:
            result, error = self.packetHandler.write2ByteTxRx(self.port, id, register, value)
        return result, error

    '''
    ############################################################################
    # ARGGGGGGHHHHH, syncRead is not available in Protocol 1.0
    ############################################################################

    ############################################################################
    # Performs a sync read for the servos specified in the idList
    # If successful, returns COMM_SUCCESS, data
    # If unsuccessful, returns the result code, and an empty dictionary
    # Data is a { servoId: value } dictionary.
    def syncRead(self, register: int, dataLen: int, idList: List[int]):
        self.syncReadInit(register, dataLen)
        for id in idList:
            if id in self.actuators:
                self.syncReadPush(self.actuators[id], register)
        return self.syncReadComplete()

    ############################################################################
    # Opens a sync read session
    # Pass in the starting register address, and the number of bytes (between 1
    # and 4)
    def syncReadInit(self, register: int, dataLen: int):
        self.syncLock.acquire()
        self.syncDataLen = dataLen
        self.syncEncoder = sdk.GroupSyncRead(self.port, self.packetHandler, register, dataLen)
        self.syncRegister = register
        self.syncReadDXLs = []

    ############################################################################
    # Pushes an actuator id into the synch read tx buffer
    def syncReadPush(self, dxl, register: int):
        if self.syncRegister == register:
            if self.syncEncoder.addParam(dxl.id):
                self.syncReadDXLs.append(dxl)
                return True
        return False

    ############################################################################
    # Closes the sync read session and broadcasts the sync read command and
    # collects the returned data in a { key: value } dictionary.
    # If successful, returns COMM_SUCCESS, data
    # If unsuccessful, returns the result code, and an empty dictionary
    def syncReadComplete(self):
        self.syncRegister = None
        result = sdk.COMM_SUCCESS
        data = {}
        if len(self.syncReadDXLs) > 0:
            with self.lock:
                result = self.syncEncoder.txRxPacket()
            if result == sdk.COMM_SUCCESS:
                data = { dxl.id: dxl.convert(self.syncRegister, self.syncEncoder.getData(dxl.id, self.syncRegister, self.syncDataLen)) for dxl in self.syncReadDXLs }
        self.syncEncoder = None
        self.syncReadDXLs = None
        self.syncLock.release()
        return result, data
    '''

    ############################################################################
    # Performs a sync write for the servo, value pairs specified in the dataDict
    # If successful, returns COMM_SUCCESS
    # If unsuccessful, returns the result code
    def syncWrite(self, register: int, dataLen: int, dataDict):
        self.syncWriteInit(register, dataLen)
        for id, value in dataDict.items():
            if id in self.actuators:
                self.syncWritePush(self.actuators[id], register, value)
        return self.syncWriteComplete()

    ############################################################################
    # Opens a sync write session
    # Pass in the starting register address, and the number of bytes (between 1
    # and 4)
    def syncWriteInit(self, register: int, dataLen: int):
        self.syncLock.acquire()
        self.syncDataLen = dataLen
        self.syncEncoder = sdk.GroupSyncWrite(self.port, self.packetHandler, register, dataLen)
        self.syncRegister = register
        self.syncWriteCount = 0

    ############################################################################
    # Pushes an actuator id, and a value into the synch write tx buffer
    def syncWritePush(self, dxl, register: int, value: int):
        if self.syncRegister == register:
            param = [sdk.DXL_LOBYTE(sdk.DXL_LOWORD(value)), sdk.DXL_HIBYTE(sdk.DXL_LOWORD(value)), sdk.DXL_LOBYTE(sdk.DXL_HIWORD(value)), sdk.DXL_HIBYTE(sdk.DXL_HIWORD(value))]
            if not self.syncEncoder.addParam(dxl.id, param[0:self.syncDataLen]):
                return False
            self.syncWriteCount += 1
            return True
        return False

    ############################################################################
    # Closes the sync write session and broadcasts the sync write command
    # If successful, returns COMM_SUCCESS
    # If unsuccessful, returns the result code
    def syncWriteComplete(self):
        self.syncRegister = None
        result = sdk.COMM_SUCCESS
        if self.syncWriteCount > 0:
            with self.lock:
                result = self.syncEncoder.txPacket()
        self.syncEncoder = None
        self.syncLock.release()
        return result

################################################################################
# Classes to wrap register access to Dynamixel actuators
# Do not instantiate these directly. Call DXLPort.getDXL() instead.
class DXL:
    def __init__(self, port: DXLPort, id: int, model: int, callback = None):
        self.port = port
        self.id = id
        self.model = model
        self.result = None
        self.error = None
        self.offset = 0.0
        self.callback = callback
        self.callbackLock = Lock()

    ############################################################################
    # Recovers after an overload error
    def recover(self):
        self.port.recover(self.id)

    ############################################################################
    # Returns the model code of the actuator
    def ping(self):
        model, self.result, self.error = self.port.ping(self.id)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return model

    ############################################################################
    # The following are accessors for all the registers for AX, EX, RX and MX
    # model Dynamixels. Please refer to the Dynamixel e-Manuals for how they
    # should be interpreted and used. https://emanual.robotis.com/docs/en/dxl/
    ############################################################################

    @property
    def stepResolution(self):
        raise DXLException("stepResolution property needs to be overridden in the base classs")
        return 0.0

    @property
    def centerOffset(self):
        raise DXLException("centerOffset property needs to be overridden in the base classs")
        return 0

    @property
    def firmwareVersion(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_FIRMWARE_VERSION)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def returnDelayTime(self): # In microseconds
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_RETURN_DELAY_TIME)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value << 1

    @returnDelayTime.setter
    def returnDelayTime(self, value: int): # In microseconds
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_RETURN_DELAY_TIME, value >> 1)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def cwAngleLimit(self):
        steps, self.result, self.error = self.port.readUInt16(self.id, EEPROM_CW_ANGLE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return self.toDegrees(steps)

    @cwAngleLimit.setter
    def cwAngleLimit(self, value: int):
        steps = self.fromDegrees(value)
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_CW_ANGLE_LIMIT, steps)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def ccwAngleLimit(self):
        steps, self.result, self.error = self.port.readUInt16(self.id, EEPROM_CCW_ANGLE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return self.toDegrees(steps)

    @ccwAngleLimit.setter
    def ccwAngleLimit(self, value: int):
        steps = self.fromDegrees(value)
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_CCW_ANGLE_LIMIT, steps)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def temperatureLimit(self): # In degrees
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_TEMPERATURE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    '''
    # Disabled. It is inadvisable to set the temperature limit to anything other
    # than that set by the factory
    @temperatureLimit.setter
    def temperatureLimit(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_TEMPERATURE_LIMIT, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
    '''

    @property
    def minVoltageLimit(self): # In volts
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_MIN_VOLTAGE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return float(value) * 0.1

    @minVoltageLimit.setter
    def minVoltageLimit(self, volts: float): # In volts
        value = int(volts * 10.0)
        if value < 50:
            value = 50
        elif value > 160:
            value = 160
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_MIN_VOLTAGE_LIMIT, int(value * 10.0))
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def maxVoltageLimit(self): # In volts
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_MAX_VOLTAGE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return float(value) * 0.1

    @maxVoltageLimit.setter
    def maxVoltageLimit(self, volts: int): # In volts
        value = int(volts * 10.0)
        if value < 50:
            value = 50
        elif value > 160:
            value = 160
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_MAX_VOLTAGE_LIMIT, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def maxTorque(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EEPROM_MAX_TORQUE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @maxTorque.setter
    def maxTorque(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_MAX_TORQUE, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    # 0 = return status packet for ping instruction only
    # 1 = return status packet for ping and read instructions only
    # 2 = return status packet for all instructions
    @property
    def statusReturnLevel(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_STATUS_RETURN_LEVEL)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @statusReturnLevel.setter
    def statusReturnLevel(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_STATUS_RETURN_LEVEL, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def shutdown(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_SHUTDOWN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @shutdown.setter
    def shutdown(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_SHUTDOWN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def alarmLED(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_ALARM_LED)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

        return value
    @alarmLED.setter
    def alarmLED(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_ALARM_LED, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def torqueEnable(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_TORQUE_ENABLE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @torqueEnable.setter
    def torqueEnable(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_TORQUE_ENABLE, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def led(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_LED)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @led.setter
    def led(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_LED, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def goalPosition(self):
        steps, self.result, self.error = self.port.readUInt16(self.id, RAM_GOAL_POSITION)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return self.toDegrees(steps)

    @goalPosition.setter
    def goalPosition(self, value: int):
        steps = self.fromDegrees(value)
        self.result, self.error = self.port.writeUInt16(self.id, RAM_GOAL_POSITION, steps)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def movingSpeed(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_MOVING_SPEED)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @movingSpeed.setter
    def movingSpeed(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_MOVING_SPEED, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def torqueLimit(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_TORQUE_LIMIT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @torqueLimit.setter
    def torqueLimit(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_TORQUE_LIMIT, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def presentPosition(self):
        steps, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_POSITION)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return self.toDegrees(steps)

    @property
    def presentSpeed(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_SPEED)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def presentLoad(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_LOAD)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def presentVoltage(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_PRESENT_VOLTAGE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def presentTemperature(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_PRESENT_TEMPERATURE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def registered(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_REGISTERED)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def moving(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_MOVING)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def lock(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_LOCK)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @lock.setter
    def lock(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_LOCK, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def punch(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PUNCH)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @punch.setter
    def punch(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_PUNCH, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    ############################################################################
    # Helper methods
    # Not intended for general use.
    ############################################################################
    # Conversion of goal_position, present_position and angle limit registers
    # from actuator step units into degrees. This angle is calculated as
    #
    #   (<position> - <center offset>) * <actuator step resolution> - <user offset>
    #
    # Where
    # <position> is the actuator position (goal, present, or angle limit)
    # <center offset> is the offset from zero at which the notch on the
    #       dynamixel horn is aligned with the notch on the case
    # <actuator step resolution> is the angle in degrees for each step of the
    #       actuator
    # <user offset> is a user settable offset angle in degrees
    #
    # This function will simply pass the value back for other registers.
    def convert(self, register: int, value: int):
        if register in [RAM_GOAL_POSITION, RAM_PRESENT_POSITION, EEPROM_CW_ANGLE_LIMIT, EEPROM_CCW_ANGLE_LIMIT]:
            return self.toDegrees(value)
        return value

    def fromDegrees(self, degrees: float)->int:
        return int(self.centerOffset + (degrees + self.offset) / self.stepResolution)

    def toDegrees(self, steps: int)->float:
        #print(steps)
        #print(self.port.modelName(self.model))
        #print(self.centerOffset)
        #print(self.stepResolution)
        return (steps - self.centerOffset) * self.stepResolution - self.offset

################################################################################
# Accessors to registers unique to AX, RX and EX models
class DXL_AX(DXL):
    @DXL.stepResolution.getter # Python's oop implementation is so fucking weird
    def stepResolution(self):
        return 0.29

    @DXL.centerOffset.getter
    def centerOffset(self):
        return 512

    @property
    def cwComplianceMargin(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @cwComplianceMargin.setter
    def cwComplianceMargin(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def ccwComplianceMargin(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CCW_COMPLIANCE_MARGIN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @ccwComplianceMargin.setter
    def ccwComplianceMargin(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CCW_COMPLIANCE_MARGIN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def cwComplianceSlope(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CW_COMPLIANCE_SLOPE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @cwComplianceSlope.setter
    def cwComplianceSlope(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN, SLOPE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def ccwComplianceSlope(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CCW_COMPLIANCE_SLOPE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @ccwComplianceSlope.setter
    def ccwComplianceSlope(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CCW_COMPLIANCE_SLOPE, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

################################################################################
# Accessors to registers unique to the EX model
class DXL_EX(DXL_AX):
    @DXL_AX.stepResolution.getter
    def stepResolution(self):
        return 0.06127

    @DXL_AX.centerOffset.getter
    def centerOffset(self):
        return 2048

    @property
    def sensedCurrent(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EX_RAM_SENSED_CURRENT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

################################################################################
# Accessors to registers unique to MX models
class DXL_MX(DXL):
    @DXL.stepResolution.getter
    def stepResolution(self):
        return 0.088

    @DXL.centerOffset.getter
    def centerOffset(self):
        return 2048

    @property
    def multiTurnOffset(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX_EEPROM_MULTI_TURN_OFFSET)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @multiTurnOffset.setter
    def multiTurnOffset(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, MX_EEPROM_MULTI_TURN_OFFSET, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def resolutionDivider(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_EEPROM_RESOLUTION_DIVIDER)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @resolutionDivider.setter
    def resolutionDivider(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX_EEPROM_RESOLUTION_DIVIDER, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def dGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_D_GAIN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @dGain.setter
    def dGain(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_D_GAIN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def iGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_I_GAIN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @iGain.setter
    def iGain(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_I_GAIN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def pGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_P_GAIN)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @pGain.setter
    def pGain(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_P_GAIN, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def realtimeTick(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX_RAM_REALTIME_TICK)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @property
    def goalAcceleration(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_GOAL_ACCELERATION)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @goalAcceleration.setter
    def goalAcceleration(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_GOAL_ACCELERATION, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

################################################################################
# Accessors to registers unique to the MX64 and MX106 models
class DXL_MX64(DXL_MX):
    @property
    def current(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX64_RAM_CURRENT)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @current.setter
    def current(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, MX64_RAM_CURRENT, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def torqueCtlModeEnable(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX64_RAM_TORQUE_CTL_MODE_ENABLE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @torqueCtlModeEnable.setter
    def torqueCtlModeEnable(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, MX64_RAM_TORQUE_CTL_MODE_ENABLE, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)

    @property
    def goalTorque(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX64_RAM_GOAL_TORQUE)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
        return value

    @current.setter
    def goalTorque(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, MX64_RAM_GOAL_TORQUE, value)
        with self.callbackLock:
            if self.callback != None: self.callback(self, self.result, self.error)
-60.104
