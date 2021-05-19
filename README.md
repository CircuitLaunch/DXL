# DXL.py
Python drop-in module that wraps the Dynamixel Protocol 1.0 api in a friendly Python class that is thread safe within the same Kernel.
Protocol 2.0 is not supported.

## Requires the DynamixelSDK
If you haven't already, install the DynamixelSDK by following the instructions provided at
https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/

## Clone the DXL repo
```Bash
$ git clone https://github.com/CircuitLaunch/DXL.git
```

## Copy the DXL.py file into your project folder

## Basic Example
```Python
from DXL import *

controller = DXLPort('/dev/ttyUSB0', 1000000)

servo1 = controller.getDXL(1)
if servo1 == None:
  print("No actuator with id 1")
  exit(-1)
  
servo2 = controller.getDXL(2)
if servo2 == None:
  print("No actuator with id 2")
  exit(-1)

servo1.torqueEnable = 1
servo2.torqueEnable = 1

if servo1.result != 0:
  print(f"Comm error with servo1: {controller.resultString(servo1.result)}")
if servo1.error != 0:
  print(f"Servo1 returned error: {controller.errorString(servo1.error)}")
  
if servo2.result != 0:
  print(f"Comm error with servo2: {controller.resultString(servo2.result)}")
  
if servo2.error != 0:
  print(f"Servo2 returned error: {controller.errorString(servo2.error)}")
  
servo1.goalPosition = 30.0
servo2.goalPosition = 60.0
```
## Example Sync Write
```Python
from DXL import *
import timer

controller = DXLPort('/dev/ttyUSB0', 1000000)

angles = [-50, -40, -30, -20, -10, 10, 20, 30, 40, 50]

# Enable torque
controller.syncWriteInit(RAM_TORQUE_ENABLE, 1) # Register value, size in bytes of register (up to 4), refer to Dynamixel e-Manual for register sizes
for id in range(1, 11):
  servo = controller.getDXL(i)
  if servo == None:
    print(f"Actuator {id} not found")
    exit(-1)
  servo.torqueEnable = 1
result = controller.syncWriteComplete()
if result != 0:
  print(f"Comm error on sync write torqueEnable: {controller.resultString(result)}")

# Command servos to goal positions
controller.syncWriteInit(RAM_GOAL_POSITION, 2) # Register value, size in bytes of register (up to 4), refer to Dynamixel e-Manual for register sizes
for id in range(1, 11):
  controller.getDXL(i).goalPosition = angles[1]
result = controller.syncWriteComplete()

if result != 0:
  print(f"Comm error on sync write goalPosition: {controller.resultString(result)}")
  
# Allow time for servos to attain goal position
timer.sleep(3.0)
  
# Disable torque
controller.syncWriteInit(RAM_TORQUE_ENABLE, 1) # Register value, size in bytes of register (up to 4), refer to Dynamixel e-Manual for register sizes
for id in range(1, 11):
  controller.getDXL(i).torqueEnable = 0
result = controller.syncWriteComplete()

if result != 0:
  print(f"Comm error on sync write torqueEnable: {controller.resultString(result)}")
```
## Example Sync Read
```Python
from DXL import *

controller = DXLPort('/dev/ttyUSB0', 1000000)

# Get present position
controller.syncReadInit(RAM_PRESENT_POSITION, 2) # Register value, size in bytes of register (up to 4), refer to Dynamixel e-Manual for register sizes
for id in range(1, 11):
  servo = controller.getDXL(i)
  if servo == None:
    print(f"Actuator {id} not found")
  servo.presentPosition
result, dictionary = controller.syncReadComplete()

if result != 0:
  print(f"Comm error on sync read: {controller.resultString(result)}")
  
for id in range(1, 11):
  print(f"Present position of actuator {id}: {dictionary[id]}")
```
## Reference

### Constants
```Python
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
gResultCodeDescriptors = [
    dxl.COMM_SUCCESS: 'OK',
    dxl.COMM_PORT_BUSY: 'PORT_BUSY',
    dxl.COMM_TX_FAIL: 'TX_FAIL',
    dxl.COMM_RX_FAIL: 'RX_FAIL',
    dxl.COMM_TX_ERROR: 'TX_ERROR',
    dxl.COMM_RX_WAITING: 'RX_WAITING',
    dxl.COMM_RX_TIMEOUT: 'RX_TIMEOUT',
    dxl.COMM_RX_CORRUPT: 'RX_CORRUPT',
    dxl.COMM_NOT_AVAILABLE: 'NOT_AVAILABLE'
]

gErrorBitDescriptors = ['INSTRUCTION', 'OVERLOAD', 'CHECKSUM', 'RANGE', 'OVERHEAT', 'ANGLE', 'VOLTAGE']
```

### DXLPort class
```Python
    ############################################################################
    # Class method to return string model name given model code
    # Returns None if code does not correspond to a known model
    @staticmethod
    def modelName(code: int)

    ############################################################################
    # Class method to return model code given model string name
    # Returns None if name does not correspond to a known model
    @staticmethod
    def modelCode(name: str)

    ############################################################################
    # Constructor
    # Pass in the port name, and the baud rate
    # Throws DXLException if a connection to the device cannot be made
    # or if the baud rate is not supported
    def __init__(self, address: str, baud: int = 57600)

    ############################################################################
    # Scans for connected actuators and lists their model numbers and ids
    def scan(self)

    ############################################################################
    # Returns a DXL representing the actuator with the given id
    # If one was previously instantiated, a reference to that one is returned
    # If no actuator with that id exists, returns None
    def getDXL(self, id: int)

    ############################################################################
    # Disables torque, sets the goal position to the present position, sets
    # torque_limit to maximum, and then re-enables torque; this is the
    # recommended way to recover from an overload error; may not work reliably
    # on AX models
    def recover(self, id: int)

    ############################################################################
    # Returns a result string given a result code
    def resultString(self, result: int)

    ############################################################################
    # Returns a space delimited list of error strings based on the error flags
    # set in the error code
    def errorString(self, error: int)

    ############################################################################
    # Sends a ping instruction.
    # If successful, returns the model code, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def ping(self, id: int)

    ############################################################################
    # Reads from a byte register
    # If successful, returns the value, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def readUInt8(self, id: int, register: int)

    ############################################################################
    # Reads from a two byte register
    # If successful, returns the value, COMM_SUCCESS, 0
    # If unsuccessful, returns 0, the result code, and an error code
    def readUInt16(self, id: int, register: int)

    ############################################################################
    # Writes to a byte register
    # If successful, returns COMM_SUCCESS, 0
    # If unsuccessful, returns the result code, and an error code
    def writeUInt8(self, id: int, register: int, value: int)

    ############################################################################
    # Writes tp a two byte register
    # If successful, returns COMM_SUCCESS, 0
    # If unsuccessful, returns the result code, and an error code
    def writeUInt16(self, id: int, register: int, value: int)

    ############################################################################
    # Opens a sync read session
    # Pass in the starting register address, and the number of bytes (between 1
    # and 4)
    def syncReadInit(self, register: int, dataLen: int)

    ############################################################################
    # Pushes an actuator id into the synch read tx buffer
    def syncReadPush(self, dxl, register: int)

    ############################################################################
    # Closes the sync read session and broadcasts the sync read command and
    # collects the returned data in a { key: value } dictionary.
    # If successful, returns COMM_SUCCESS, data
    # If unsuccessful, returns the result code, and an empty dictionary
    def syncReadComplete(self)

    ############################################################################
    # Opens a sync write session
    # Pass in the starting register address, and the number of bytes (between 1
    # and 4)
    def syncWriteInit(self, register: int, dataLen: int)

    ############################################################################
    # Pushes an actuator id, and a value into the synch write tx buffer
    def syncWritePush(self, dxl, register: int, value: int)

    ############################################################################
    # Closes the sync write session and broadcasts the sync write command and
    # collects the returned data in a { key: value } dictionary.
    # If successful, returns COMM_SUCCESS
    # If unsuccessful, returns the result code
    def syncWriteComplete(self)
```
### DXLException(Exception) class
```Python
    ############################################################################
    # Member variables
    self.file       # the file name in which the exception was thrown
    self.function   # the name of the function from which the exception was thrown
    self.line       # the line number in the file on which the exception was thrown
    self.msg        # the message passed to the constructor
    
    ############################################################################
    # Constructor
    # Initializes member variables with the filename, function, and line number
    def __init__(self, msg)

    ############################################################################
    # Returns a string reporting the file, function, line, and message
    def __str__(self)

```
### DXL base class
```Python
    ############################################################################
    # Member variables
      self.id     # The actuator id
      self.model  # The model code
      self.result # The result returned after each TxRx
      self.error  # The error code returned after each TxRx
      self.offset # The user settable offset for the actuator in degrees

    ############################################################################
    # Recovers after an overload error
    def recover(self)

    ############################################################################
    # Returns the model code, the com status, and the error status of the
    # actuator
    def ping(self)->(int, int, int)

    ############################################################################
    # The following are accessors for all the registers for AX, EX, RX and MX
    # model Dynamixels. Please refer to the Dynamixel e-Manuals for how they
    # should be interpreted and used. https://emanual.robotis.com/docs/en/dxl/
    ############################################################################

    @property
    def stepResolution(self) # Base class throws unimplemented exception
    
    @property
    def centerOffset(self)  # Base class throws unimplemented exception
    
    @property
    def firmwareVersion(self)

    @property
    def returnDelayTime(self) # In microseconds
    @returnDelayTime.setter
    def returnDelayTime(self, value: int) # In microseconds

    @property
    def cwAngleLimit(self) # In degrees
    @cwAngleLimit.setter
    def cwAngleLimit(self, value: int) # In degrees

    @property
    def ccwAngleLimit(self) # In degrees
    @ccwAngleLimit.setter
    def ccwAngleLimit(self, value: int) # In degrees

    @property
    def temperatureLimit(self) # In degrees

    @property
    def minVoltageLimit(self) # In volts
    @minVoltageLimit.setter
    def minVoltageLimit(self, volts: float) # In volts
    # Clamps to between 50 <= volts <= 160

    @property
    def maxVoltageLimit(self): # In volts
    @maxVoltageLimit.setter
    def maxVoltageLimit(self, volts: int) # In volts
    # Clamps to between 50 <= volts <= 160

    @property
    def maxTorque(self)
    @maxTorque.setter
    def maxTorque(self, value: int)

    @property
    def statusReturnLevel(self)
    @statusReturnLevel.setter
    def statusReturnLevel(self, value: int)
    # 0 = return status packet for ping instruction only
    # 1 = return status packet for ping and read instructions only
    # 2 = return status packet for all instructions

    @property
    def shutdown(self)
    @shutdown.setter
    def shutdown(self, value: int)

    @property
    def alarmLED(self)
    @alarmLED.setter
    def alarmLED(self, value: int)

    @property
    def torqueEnable(self)
    @torqueEnable.setter
    def torqueEnable(self, value: int)

    @property
    def led(self)
    @led.setter
    def led(self, value: int)

    @property
    def goalPosition(self)
    @goalPosition.setter
    def goalPosition(self, value: int)

    @property
    def movingSpeed(self)
    @movingSpeed.setter
    def movingSpeed(self, value: int)

    @property
    def torqueLimit(self)
    @torqueLimit.setter
    def torqueLimit(self, value: int)

    @property
    def presentPosition(self)

    @property
    def presentSpeed(self)

    @property
    def presentLoad(self)

    @property
    def presentVoltage(self)

    @property
    def presentTemperature(self)

    @property
    def registered(self)
    
    @property
    def moving(self)

    @property
    def lock(self)
    @lock.setter
    def lock(self, value: int)

    @property
    def punch(self)
    @punch.setter
    def punch(self, value: int)
```
### DXL_AX(DXL) class
```Python
    ################################################################################
    # Accessors to registers unique to AX, RX and EX models
    
    @DXL.stepResolution.getter
    def stepResolution(self) # Overrides super class property to return 0.29
    
    @DXL.centerOffset.getter
    def centerOffset(self) # Overrides super class property to return 512

    @property
    def cwComplianceMargin(self)
    @cwComplianceMargin.setter
    def cwComplianceMargin(self, value: int)

    @property
    def ccwComplianceMargin(self)
    @ccwComplianceMargin.setter
    def ccwComplianceMargin(self, value: int)

    @property
    def cwComplianceSlope(self)
    @cwComplianceSlope.setter
    def cwComplianceSlope(self, value: int)

    @property
    def ccwComplianceSlope(self)
    @ccwComplianceSlope.setter
    def ccwComplianceSlope(self, value: int)

```
### DXL_EX(DXL_AX) class
```Python
    ################################################################################
    # Accessors to registers unique to the EX model
    
    @DXL_AX.stepResolution.getter
    def stepResolution(self) # Overrides super class property to return 0.06127

    @DXL_AX.centerOffset.getter
    def centerOffset(self) # Overrides super class property to return 2048

    @property
    def sensedCurrent(self)
```
### DXL_MX(DXL) class
```Python
    ################################################################################
    # Accessors to registers unique to MX models
    
    @DXL.stepResolution.getter
    def stepResolution(self) # Overrides super class property to return 0.088

    @DXL.centerOffset.getter
    def centerOffset(self): # Overrides super class property to return 2048

    @property
    def multiTurnOffset(self)
    @multiTurnOffset.setter
    def multiTurnOffset(self, value: int)

    @property
    def resolutionDivider(self)
    @resolutionDivider.setter
    def resolutionDivider(self, value: int)

    @property
    def dGain(self)
    @dGain.setter
    def dGain(self, value: int)

    @property
    def iGain(self)
    @iGain.setter
    def iGain(self, value: int)

    @property
    def pGain(self)
    @pGain.setter
    def pGain(self, value: int)

    @property
    def realtimeTick(self)

    @property
    def goalAcceleration(self)
    @goalAcceleration.setter
    def goalAcceleration(self, value: int):
```
### DXL_MX64(DXL_MX) class
```Python
    ################################################################################
    # Accessors to registers unique to the MX64 and MX106 models
    
    @property
    def current(self)
    @current.setter
    def current(self, value: int)

    @property
    def torqueCtlModeEnable(self)
    @torqueCtlModeEnable.setter
    def torqueCtlModeEnable(self, value: int)

    @property
    def goalTorque(self)
    @current.setter
    def goalTorque(self, value: int)
 ```
