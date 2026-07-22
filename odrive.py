import struct
from dataclasses import dataclass
import queue
import time
import threading
import can

VELOCITY_LIMIT = 10.0 # turns/sec
CURRENT_LIMIT = 5.0 # amperes

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

ERROR_NAMES = {
    0x1:"INITIALIZING",
    0x2:"SYSTEM_LEVEL",
    0x4:"TIMING_ERROR",
    0x8:"MISSING_ESTIMATE",
    0x10:"BAD_CONFIG", 
    0x20:"DRV_FAULT",
    0x40:"MISSING_INPUT", #64
    0x100:"DC_BUS_OVER_VOLTAGE",
    0x200:"DC_BUS_UNDER_VOLTAGE",
    0x400:"DC_BUS_OVER_CURRENT", #1024
    0x800:"DC_BUS_OVER_REGEN_CURRENT",
    0x1000:"CURRENT_LIMIT_VIOLATION",
    0x2000:"MOTOR_OVER_TEMP", 
    0x4000:"INVERTER_OVER_TEMP",
    0x8000:"VELOCITY_LIMIT_VIOLATION",
    0x10000:"POSITION_LIMIT_VIOLATION",
    0x1000000:"WATCHDOG_TIMER_EXPIRED",
    0x2000000:"ESTOP_REQUESTED",  
    0x4000000:"SPINOUT_DETECTED",
    0x8000000:"BRAKE_RESISTOR_DISARMED",
    0x10000000:"THERMISTOR_DISCONNECTED",
    0x40000000:"CALIBRATION_ERROR" #1073741824 
}

@dataclass
class EncoderEstimate:
    position: float
    velocity: float
    timestamp: float


@dataclass
class ODriveError:
    node_id: int
    axis_error: int
    timestamp: float

class ODriveCANManager:
    def __init__(self,
        node_ids = [1, 2, 3],
        channel: str = "can0",
        bustype: str = "socketcan",
        bitrate: int = 500000,
        position_timeout: float = 0.1,
    ):
        self.node_ids = list(node_ids)

        self.position_timeout = position_timeout

        self.axis_state = { node_id: AXISSTATE_UNDEFINED for node_id in self.node_ids }
        self.controller_mode = { node_id: CTRLMODE_UNDEFINED for node_id in self.node_ids }

        self.latest_encoder = { node_id: None for node_id in self.node_ids }
        self.last_heartbeat = { node_id: None for node_id in self.node_ids }
        self.error_queue = queue.Queue()

        self.lock = threading.Lock()
        self.running = True
        self.rx_thread = None
        self.fault_detected = False

        self.bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        while (self.bus.recv(timeout=0) is not None): pass
        
        print("[ODrive] CAN bus opened");
        self.start()
        # TODO check for status

        for node_id in self.node_ids:
            self._setAxisClosedLoop(node_id)
            while self.axis_state[node_id] != AXISSTATE_CLOSED_LOOP_CONTROL:
                pass
            self.setVel(node_id, 0.0)

    def start(self):
        self.rx_thread = threading.Thread(
            target=self._readCAN,
            name="ODriveCAN-RX",
            daemon=True,
        )
        self.rx_thread.start()

    def stop(self):
        self.stopMotors()
        self.running = False
        if self.rx_thread is not None:
            self.rx_thread.join(timeout=1.0)
        self.bus.shutdown()
        print("ODriveCANManager stopped")

    def _readCAN(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1) # TODO does timeout matter ?
                if msg is None:
                    continue
                
                node_id = msg.arbitration_id >> 5
                if node_id not in self.node_ids:
                    return
                
                command_id = (msg.arbitration_id & 0x1F)

                if command_id == ID_GET_ENCODER_ESTIMATES:
                    if len(msg.data) < 8:
                        return
                    position, velocity = struct.unpack("<ff", msg.data[:8])

                    estimate = EncoderEstimate(position=position, velocity=velocity, timestamp=time.monotonic())

                    with self.lock:
                        self.latest_encoder[node_id] = estimate

                elif command_id == ID_HEARTBEAT:
                    if len(msg.data) < 5:
                        return
                    now = time.monotonic()
                    
                    with self.lock:
                        self.last_heartbeat[node_id] = now

                    axis_error, axis_state = struct.unpack('<IB', bytes(msg.data[:5]))

                    # after entering closed loop, set limits
                    if self.axis_state[node_id] != AXISSTATE_CLOSED_LOOP_CONTROL:
                        if axis_state == AXISSTATE_CLOSED_LOOP_CONTROL:
                            print("Axis", node_id, "in closed loop, set limits")
                            self.sendCAN(node_id, ID_SET_LIMITS, struct.pack('<ff', VELOCITY_LIMIT, CURRENT_LIMIT))

                    # update state
                    self.axis_state[node_id] = axis_state

                    # handle error
                    if axis_error != 0:
                        if axis_error != 0:
                            error = ODriveError(node_id=node_id, axis_error=axis_error, timestamp=now)
                            print("ERROR on node", error.node_id, ":", ERROR_NAMES[error.axis_error])
                            self.error_queue.put(error)
                            self.fault_detected = True
                            # TODO fault_detected => stop motors ?

            except can.CanError as e:
                print(f"[CAN RX ERROR] {e}")

            except Exception as e:
                print(f"[RX THREAD ERROR] {e}")
                self.fault_detected = True

    def getPendingErrors(self):
            errors = []
            while True:
                try:
                    error = (self.error_queue.get_nowait())
                    errors.append(error)
                except queue.Empty:
                    break
            return errors

    def sendCAN(self, node_id, command, payload):
        if not self.bus:
            print("[Odrive] ERROR CAN bus not defined !")
            return
        
        try:
            print("send CAN", node_id, command)
            self.bus.send(can.Message(
                arbitration_id=(node_id << 5 | command), 
                data=payload,
                is_extended_id=False
            ))

        except can.exceptions.CanOperationError:
            print("CanOperationError")

    def _setAxisClosedLoop(self, node_id):
        if node_id not in self.node_ids:
            return
        if self.axis_state[node_id] == AXISSTATE_CLOSED_LOOP_CONTROL:
            return True
        print("set closed loop", node_id)
        self.sendCAN(node_id, ID_SET_AXIS_STATE, struct.pack('<I', AXISSTATE_CLOSED_LOOP_CONTROL))
        return False

    def _setControllerMode(self, node_id, mode):
        if node_id not in self.node_ids:
            return
        if not self._setAxisClosedLoop(node_id):
            print("not ready")
            return
        if self.controller_mode[node_id] != mode:
            print("set mode: "+CTRLMODE_NAMES[mode])
            self.sendCAN(node_id, ID_SET_CONTROLLER_MODE, struct.pack('<II', mode, INPUT_MODE_PASSTHROUGH))
            self.controller_mode[node_id] = mode
        return False
    
    def setPos(self, node_id, turns):
        if node_id not in self.node_ids:
            return
        if not self._setAxisClosedLoop(node_id):
            print("set pos not ready")
            return
        self._setControllerMode(node_id, CTRLMODE_POS_CTRL)
        self.sendCAN(node_id, ID_SET_INPUT_POS, struct.pack('<fhh', turns, 0, 0)) # position, velocity feedforward, torque feedforward

    def setVel(self, node_id, turn_per_sec):
        if node_id not in self.node_ids:
            return
        if not self._setAxisClosedLoop(node_id):
            print("set vel not ready")
            return
        self._setControllerMode(node_id, CTRLMODE_VEL_CTRL)
        self.sendCAN(node_id, ID_SET_INPUT_VEL, struct.pack('<ff', turn_per_sec, 0.0)) # velocity, torque feedforward

    def setTorque(self, node_id, newton_meters):
        if node_id not in self.node_ids:
            return
        if not self._setAxisClosedLoop(node_id):
            print("set torque not ready")
            return
        self._setControllerMode(node_id, CTRLMODE_TORQUE_CTRL)
        self.sendCAN(node_id, ID_SET_INPUT_TORQUE, struct.pack('<f', newton_meters)) # torque
        
    def clearErrors(self, node_id):
        self.sendCAN(node_id, ID_CLEAR_ERRORS, b'') # no payload
        
    def stopMotors(self):
        print("STOP MOTORS")
        for node_id in self.node_ids:
            self.setVel(node_id, 0.0)
