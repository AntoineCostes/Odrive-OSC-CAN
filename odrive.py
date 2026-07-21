import can
import struct


VELOCITY_LIMIT = 10 # turns/sec
CURRENT_LIMIT = 5 # amperes

# Command ids
# https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/docs/NBlf3rgI8zzL4eTvLjFqdETy04VScp3e8YNoAb3BcPY/manual/can-protocol.html#overview
ID_HEARTBEAT = 1
ID_GET_ERROR = 3
ID_SET_AXIS_STATE = 7	
ID_GET_ENCODER_ESTIMATES = 9
ID_SET_CONTROLLER_MODE = 11 #0x0b
ID_SET_INPUT_POS = 12 #0x0c
ID_SET_INPUT_VEL = 13 #0x0d
ID_SET_INPUT_TORQUE = 14 #0x0e
ID_SET_LIMITS = 15 #0x0f
ID_CLEAR_ERRORS = 24 # 0x18
ID_SET_ABSOLUTE_POSITION = 25 #0x19 
ID_GET_TORQUES = 28 #0x1c

# Axis states
# https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/docs/NBlf3rgI8zzL4eTvLjFqdETy04VScp3e8YNoAb3BcPY/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.AxisState
AXISSTATE_UNDEFINED = 0
AXISSTATE_IDLE = 1
AXISSTATE_CLOSED_LOOP_CONTROL = 8

# Controller mode
# https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/docs/NBlf3rgI8zzL4eTvLjFqdETy04VScp3e8YNoAb3BcPY/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.ControlMode
CTRLMODE_UNDEFINED = 0
CTRLMODE_TORQUE_CTRL = 1
CTRLMODE_VEL_CTRL = 2
CTRLMODE_POS_CTRL = 3
CTRLMODE_NAMES = ["Undefined", "Torque", "Velocity", "Position"]

# Input mode
# https://odrive-cdn.nyc3.digitaloceanspaces.com/releases/docs/NBlf3rgI8zzL4eTvLjFqdETy04VScp3e8YNoAb3BcPY/fibre_types/com_odriverobotics_ODrive.html#ODrive.Controller.InputMode
INPUT_MODE_INACTIVE = 0
INPUT_MODE_PASSTHROUGH = 1


STATE_IDLE = 0
STATE_WAITING_FOR_CLOSED_LOOP = 1
STATE_CLOSED_LOOP = 2
STATE_ERROR = 3

class ODriveManager:
    def __init__(self):
        bus = can.interface.Bus("can0", bustype="socketcan")
        while not (self.bus.recv(timeout=0) is None): pass
        
        print("[ODrive] CAN bus opened");

        self.motors = [ODriveMotor(bus, 1), ODriveMotor(bus, 2), ODriveMotor(bus, 3)]
        
        # TODO check for status

    def update(self):
        for motor in self.motors:
            motor.readCAN()

    def setPos(self, index, value):
        if index < 0 or index > len(self.motors):
            print("[ODriveManager] cannot set pos: invalid index")
            return
        self.motors[index].setPosition(value)
        
    def setVel(self, index, value):
        if index < 0 or index > len(self.motors):
            print("[ODriveManager] cannot set vel: invalid index")
            return
        self.motors[index].setVelocity(value)
        
    def setTorque(self, index, value):
        if index < 0 or index > len(self.motors):
            print("[ODriveManager] cannot set torque: invalid index")
            return
        self.motors[index].setTorque(value)
        
    def clearErrors(self, index):
        if index < 0 or index > len(self.motors):
            print("[ODriveManager] cannot clear errors: invalid index")
            return
        self.motors[index].clearErrors()


class ODriveMotor:
    def __init__(self, bus, node_id):
        self.node_id = node_id
        self.bus = bus

        self.currentState = STATE_IDLE
        self.currentModeValue = CTRLMODE_UNDEFINED

        self.pos, self.vel = 0.0 

        self.setAxisClosedLoop()
        self.setVelocity(0.0)

    def checkCommand(self, msg, command):
        return msg.arbitration_id == (self.node_id << 5 | command)
    
    def sendCAN(self, command, payload):
        if not self.bus:
            print("[Odrive] ERROR CAN bus not defined !")
            return
        
        try:
            print("send CAN", self.node_id, command)
            self.bus.send(can.Message(
                arbitration_id=(self.node_id << 5 | command), 
                data=payload,
                is_extended_id=False
            ))

        except can.exceptions.CanOperationError:
            print("CanOperationError")

    def readCAN(self, printPosVel = False):
        for msg in self.bus:        
            # if printPosVel:
            if self.checkCommand(msg, ID_GET_ENCODER_ESTIMATES): 
                self.pos, self.vel = struct.unpack('<ff', bytes(msg.data))
                    # print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

            # heartbeat gives us errors and axis state
            if self.checkCommand(msg, ID_HEARTBEAT):
                axis_error = struct.unpack('<I', msg.data[0:4])[0]
                
                if axis_error != 0:
                    self.currentState = STATE_ERROR
                    if axis_error & 0x1:
                        print("ERROR: INITIALIZING")
                    if axis_error & 0x2:
                        print("ERROR: SYSTEM_LEVEL")
                    if axis_error & 0x4:
                        print("ERROR: TIMING_ERROR")
                    if axis_error & 0x8:
                        print("ERROR: MISSING_ESTIMATE")
                    if axis_error & 0x10: #16
                        print("ERROR: BAD_CONFIG")
                    if axis_error & 0x20: #32
                        print("ERROR: DRV_FAULT")
                    if axis_error & 0x40: #64
                        print("ERROR: MISSING_INPUT")
                    if axis_error & 0x100: #256
                        print("ERROR: DC_BUS_OVER_VOLTAGE")
                    if axis_error & 0x200: #512
                        print("ERROR: DC_BUS_UNDER_VOLTAGE")
                    if axis_error & 0x400: #1024
                        print("ERROR: DC_BUS_OVER_CURRENT")
                    if axis_error & 0x800: #2048
                        print("ERROR: DC_BUS_OVER_REGEN_CURRENT")
                    if axis_error & 0x1000: #4096
                        print("ERROR: CURRENT_LIMIT_VIOLATION")
                    if axis_error & 0x2000: #8192
                        print("ERROR: MOTOR_OVER_TEMP")
                    if axis_error & 0x4000: #16384
                        print("ERROR: INVERTER_OVER_TEMP")
                    if axis_error & 0x8000: #32768
                        print("ERROR: VELOCITY_LIMIT_VIOLATION")
                    if axis_error & 0x10000: #65536
                        print("ERROR: POSITION_LIMIT_VIOLATION")
                    if axis_error & 0x1000000: #16777216 
                        print("ERROR: WATCHDOG_TIMER_EXPIRED")
                    if axis_error & 0x2000000: #33554432 
                        print("ERROR: ESTOP_REQUESTED")
                    if axis_error & 0x4000000: #67108864 
                        print("ERROR: SPINOUT_DETECTED")
                    if axis_error & 0x8000000: #134217728 
                        print("ERROR: BRAKE_RESISTOR_DISARMED")
                    if axis_error & 0x10000000: #268435456 
                        print("ERROR: THERMISTOR_DISCONNECTED")
                    if axis_error & 0x40000000: #1073741824 
                        print("ERROR: CALIBRATION_ERROR")

                axis_state = msg.data[4]

                # if we recovered from an error, update state
                if self.currentState == STATE_ERROR and axis_error == 0:
                    if axis_state == AXISSTATE_CLOSED_LOOP_CONTROL:
                        self.currentState = STATE_CLOSED_LOOP
                    if axis_state == AXISSTATE_IDLE:
                        self.currentState = STATE_IDLE

                # if we succeeded entering closed loop, set limits
                if self.currentState == STATE_WAITING_FOR_CLOSED_LOOP and axis_state == AXISSTATE_CLOSED_LOOP_CONTROL :
                    self.sendCAN(ID_SET_LIMITS, struct.pack('<I', VELOCITY_LIMIT, CURRENT_LIMIT))
                    self.currentState = STATE_CLOSED_LOOP
                    print("Closed loop OK !")

    def setAxisClosedLoop(self):
        self.sendCAN(ID_SET_AXIS_STATE, struct.pack('<I', AXISSTATE_CLOSED_LOOP_CONTROL))

        # Wait for axis to enter closed loop control by scanning heartbeat messages
        for msg in self.bus:
            if self.checkCommand(msg, ID_HEARTBEAT): 
                error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == AXISSTATE_CLOSED_LOOP_CONTROL:
                    break
        
        self.currentState = STATE_CLOSED_LOOP

        print("Closed loop: OK")

    def setControllerMode(self, mode):
        if (self.currentState != STATE_CLOSED_LOOP):
            return

        if self.currentModeValue == mode:
            print("mode already set")
        else:
            self.sendCAN(ID_SET_CONTROLLER_MODE, struct.pack('<II', mode, INPUT_MODE_PASSTHROUGH))
            print("set mode: "+CTRLMODE_NAMES[mode])
            self.currentModeValue = mode
            
    def resetPosistion(self):
        if (self.currentState != STATE_CLOSED_LOOP):
            return
        
        if (self.currentModeValue != CTRLMODE_POS_CTRL):
            self.setControllerMode(CTRLMODE_POS_CTRL)
        
        self.sendCAN(ID_SET_ABSOLUTE_POSITION, struct.pack('<f', 0.0))

    def setPosition(self, turns):
        if (self.currentState != STATE_CLOSED_LOOP):
            self.setAxisClosedLoop()
            return
        
        print("set pos")

        if (self.currentModeValue != CTRLMODE_POS_CTRL):
            self.setControllerMode(CTRLMODE_POS_CTRL)
        
        self.sendCAN(ID_SET_INPUT_POS, struct.pack('<fhh', turns, 0, 0)) # position, velocity feedforward, torque feedforward

    def setVelocity(self, turn_per_sec):
        if (self.currentState != STATE_CLOSED_LOOP):
            self.setAxisClosedLoop()
            return
        
        print("set vel")
        
        if (self.currentModeValue != CTRLMODE_VEL_CTRL):
            self.setControllerMode(CTRLMODE_VEL_CTRL)
            
        self.sendCAN(ID_SET_INPUT_VEL, struct.pack('<ff', turn_per_sec, 0.0)) # velocity, torque feedforward

    def setTorque(self, NewtonMeters):
        if (self.currentState != STATE_CLOSED_LOOP):
            return
        
        if (self.currentModeValue != CTRLMODE_TORQUE_CTRL):
            self.setControllerMode(CTRLMODE_TORQUE_CTRL)
            
        self.sendCAN(ID_SET_INPUT_TORQUE, struct.pack('<f', NewtonMeters)) # torque

    def clearErrors(self):
        self.sendCAN(ID_CLEAR_ERRORS, b'') # no payload