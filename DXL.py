import dynamixel_sdk as dxl
from threading import Lock
import inspect

# These codes are available in the EEPROM Control Table listings for
# each device at https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#supported-dynamixel

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

g_model_str_to_code_dict = {
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

g_model_code_to_str_dict = {
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

# Flags for shutdown and alarmLED registers
ERROR_BIT_VOLTAGE = 1
ERROR_BIT_ANGLE_LIMIt = 2
ERROR_BIT_OVERHEATING = 4
ERROR_BIT_RANGE = 8
ERROR_BIT_CHECKSUM = 16
ERROR_BIT_OVERLOAD = 32
ERROR_BIT_INSTRUCTION = 64

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

class DXLPort:
    def __init__(self, address: str, baud: int = 57600):
        self.lock = Lock()
        self.baud = baud
        self.port = None
        port = dxl.PortHandler(address)
        self.actuators = {}

        try:
            port.openPort()
        except Exception as x:
            raise(DXLException(str(x)))

        self.port = port

        try:
            self.port.setBaudRate(self.baud)
        except Exception as x:
            raise(DXLException(str(x)))

        self.packetHandler = dxl.PacketHandler(1.0)

    def __del__(self):
        if self.port != None:
            with self.lock:
                self.port.closePort()

    def scan(self):
        for i in range(254):
            print(f'Pinging {i}')
            actuator = self.getDXL(i)
            if actuator != None:
                print(f'Found: {i}')
                self.actuators[i] = actuator

    def getDXL(self, id: int):
        model, result, error = self.ping(id)
        if model in [MODEL_NUMBER_AX12A, MODEL_NUMBER_AX12W, MODEL_NUMBER_AX18A, MODEL_NUMBER_RX10, MODEL_NUMBER_RX24F, MODEL_NUMBER_RX28, MODEL_NUMBER_RX64]:
            return DXL_AX(self, id, model)
        elif model == MODEL_NUMBER_EX106:
            return DXL_EX(self, id, model)
        elif model in [MODEL_NUMBER_MX12W, MODEL_NUMBER_MX28]:
            return DXL_MX(self, id, model)
        elif model in [MODEL_NUMBER_MX64, MODEL_NUMBER_MX106]:
            return DXL_MX64(self, id, model)
        else:
            return None

    def modelName(self, code: int):
        return g_model_code_to_str_dict[code]

    def modelCode(self, name: str):
        return g_model_str_to_code_dict[name]

    def resultString(self, result: int):
        return self.packetHandler.getTxRxResult(result)

    def errorString(self, error: int):
        return self.packetHandler.getTxRxPacketEror(error)

    def ping(self, id: int):
        with self.lock:
            model, result, error = self.packetHandler.ping(self.port, id)
        return model, result, error

    def readUInt8(self, id: int, register: int):
        with self.lock:
            value, result, error = self.packetHandler.read1ByteTxRx(self.port, id, register)
        return value, result, error

    def readUInt16(self, id: int, register: int):
        with self.lock:
            value, result, error = self.packetHandler.read2ByteTxRx(self.port, id, register)
        return value, result, error

    def writeUInt8(self, id: int, register: int, value: int):
        with self.lock:
            result, error = self.packetHandler.write1ByteTxRx(self.port, id, register, value)
        return result, error

    def writeUInt16(self, id: int, register: int, value: int):
        with self.lock:
            result, error = self.packetHandler.write2ByteTxRx(self.port, id, register, value)
        return result, error

class DXL:
    def __init__(self, port: DXLPort, id: int, model: int):
        self.port = port
        self.id = id
        self.model = model
        self.result = None
        self.error = None

    def ping(self)->(int, int, int):
        return self.port.ping(self.id)

    @property
    def firmwareVersion(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_FIRMWARE_VERSION)
        return value

    @property
    def returnDelayTime(self): # In microseconds
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_RETURN_DELAY_TIME)
        return value << 1

    @returnDelayTime.setter
    def returnDelayTime(self, value: int): # In microseconds
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_RETURN_DELAY_TIME, value >> 1)

    @property
    def cwAngleLimit(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EEPROM_CW_ANGLE_LIMIT)
        return value

    @cwAngleLimit.setter
    def cwAngleLimit(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_CW_ANGLE_LIMIT, value)

    @property
    def ccwAngleLimit(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EEPROM_CCW_ANGLE_LIMIT)
        return value

    @ccwAngleLimit.setter
    def ccwAngleLimit(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_CCW_ANGLE_LIMIT, value)

    @property
    def temperatureLimit(self): # In degrees
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_TEMPERATURE_LIMIT)
        return value

    '''
    # Disabled. It is inadvisable to set the temperature limit to anything other
    # than that set by the factory
    @temperatureLimit.setter
    def temperatureLimit(self, value: int):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_TEMPERATURE_LIMIT, value)
    '''

    @property
    def minVoltageLimit(self): # In volts
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_MIN_VOLTAGE_LIMIT)
        return float(value) * 0.1

    @minVoltageLimit.setter
    def minVoltageLimit(self, volts: float): # In volts
        value = int(volts * 10.0)
        if value < 50:
            value = 50
        elif value > 160:
            value = 160
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_MIN_VOLTAGE_LIMIT, int(value * 10.0))

    @property
    def maxVoltageLimit(self): # In volts
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_MAX_VOLTAGE_LIMIT)
        return float(value) * 0.1

    @maxVoltageLimit.setter
    def maxVoltageLimit(self, volts: int): # In volts
        value = int(volts * 10.0)
        if value < 50:
            value = 50
        elif value > 160:
            value = 160
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_MAX_VOLTAGE_LIMIT, value)

    @property
    def maxTorque(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EEPROM_MAX_TORQUE)
        return value

    @maxTorque.setter
    def maxTorque(self, value: int):
        self.result, self.error = self.port.writeUInt16(self.id, EEPROM_MAX_TORQUE, value)

    # 0 = return status packet for ping instruction only
    # 1 = return status packet for ping and read instructions only
    # 2 = return status packet for all instructions
    @property
    def statusReturnLevel(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_STATUS_RETURN_LEVEL)
        return value

    @statusReturnLevel.setter
    def statusReturnLevel(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_STATUS_RETURN_LEVEL, value)

    @property
    def shutdown(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_SHUTDOWN)
        return value

    @shutdown.setter
    def shutdown(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_SHUTDOWN, value)

    @property
    def alarmLED(self):
        value, self.result, self.error = self.port.readUInt8(self.id, EEPROM_ALARM_LED)
        return value

    @alarmLED.setter
    def alarmLED(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, EEPROM_ALARM_LED, value)

    @property
    def torqueEnable(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_TORQUE_ENABLE)
        return value

    @torqueEnable.setter
    def torqueEnable(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_TORQUE_ENABLE, value)

    @property
    def led(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_LED)
        return value

    @led.setter
    def led(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_LED, value)

    @property
    def goalPosition(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_GOAL_POSITION)
        return value

    @goalPosition.setter
    def goalPosition(self, value):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_GOAL_POSITION, value)

    @property
    def movingSpeed(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_MOVING_SPEED)
        return value

    @movingSpeed.setter
    def movingSpeed(self, value):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_MOVING_SPEED, value)

    @property
    def torqueLimit(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_TORQUE_LIMIT)
        return value

    @torqueLimit.setter
    def torqueLimit(self, value):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_TORQUE_LIMIT, value)

    @property
    def presentPosition(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_POSITION)
        return value

    @property
    def presentSpeed(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_SPEED)
        return value

    @property
    def presentVoltage(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_VOLTAGE)
        return value

    @property
    def presentTemperature(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PRESENT_TEMPERATURE)
        return value

    @property
    def registered(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_REGISTERED)
        return value

    @property
    def moving(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_MOVING)
        return value

    @property
    def lock(self):
        value, self.result, self.error = self.port.readUInt8(self.id, RAM_LOCK)
        return value

    @lock.setter
    def lock(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, RAM_LOCK, value)

    @property
    def punch(self):
        value, self.result, self.error = self.port.readUInt16(self.id, RAM_PUNCH)
        return value

    @punch.setter
    def punch(self, value):
        self.result, self.error = self.port.writeUInt16(self.id, RAM_PUNCH, value)

class DXL_AX(DXL):
    @property
    def cwComplianceMargin(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN)
        return value

    @cwComplianceMargin.setter
    def cwComplianceMargin(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN, value)

    @property
    def ccwComplianceMargin(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CCW_COMPLIANCE_MARGIN)

    @ccwComplianceMargin.setter
    def ccwComplianceMargin(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CCW_COMPLIANCE_MARGIN, value)

    @property
    def cwComplianceSlope(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CW_COMPLIANCE_SLOPE)
        return value

    @cwComplianceSlope.setter
    def cwComplianceSlope(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CW_COMPLIANCE_MARGIN, SLOPE)

    @property
    def ccwComplianceSlope(self):
        value, self.result, self.error = self.port.readUInt8(self.id, AX_RAM_CCW_COMPLIANCE_SLOPE)
        return value

    @ccwComplianceSlope.setter
    def ccwComplianceSlope(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, AX_RAM_CCW_COMPLIANCE_SLOPE, value)

class DXL_EX(DXL_AX):
    @property
    def sensedCurrent(self):
        value, self.result, self.error = self.port.readUInt16(self.id, EX_RAM_SENSED_CURRENT)
        return value

class DXL_MX(DXL):
    @property
    def multiTurnOffset(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX_EEPROM_MULTI_TURN_OFFSET)
        return value

    @multiTurnOffset.setter
    def multiTurnOffset(self, value):
        self.result, self.error = self.port.writeUInt16(self.id, MX_EEPROM_MULTI_TURN_OFFSET, value)

    @property
    def resolutionDivider(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_EEPROM_RESOLUTION_DIVIDER)
        return value

    @resolutionDivider.setter
    def resolutionDivider(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX_EEPROM_RESOLUTION_DIVIDER, value)

    @property
    def dGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_D_GAIN)
        return value

    @dGain.setter
    def dGain(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_D_GAIN, value)

    @property
    def iGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_I_GAIN)
        return value

    @iGain.setter
    def iGain(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_I_GAIN, value)

    @property
    def pGain(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_P_GAIN)
        return value

    @pGain.setter
    def pGain(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_P_GAIN, value)

    @property
    def realtimeTick(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_REALTIME_TICK)
        return value

    @property
    def goalAcceleration(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX_RAM_GOAL_ACCELERATION)
        return value

    @goalAcceleration.setter
    def goalAcceleration(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX_RAM_GOAL_ACCELERATION, value)

class DXL_MX64(DXL_MX):
    @property
    def current(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX64_RAM_CURRENT)
        return value

    @current.setter
    def current(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX64_RAM_CURRENT, value)

    @property
    def torqueCtlModeEnable(self):
        value, self.result, self.error = self.port.readUInt8(self.id, MX64_RAM_TORQUE_CTL_MODE_ENABLE)
        return value

    @torqueCtlModeEnable.setter
    def torqueCtlModeEnable(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX64_RAM_TORQUE_CTL_MODE_ENABLE, value)

    @property
    def goalTorque(self):
        value, self.result, self.error = self.port.readUInt16(self.id, MX64_RAM_GOAL_TORQUE)
        return value

    @current.setter
    def goalTorque(self, value):
        self.result, self.error = self.port.writeUInt8(self.id, MX64_RAM_GOAL_TORQUE, value)
