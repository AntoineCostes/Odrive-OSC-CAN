
import can
from odrive import ODriveMotor
import threading

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
import argparse

LISTENING_IP = "127.0.0.1"
LISTENING_PORT = 12345

def setPos(address, index, pos):
    if index < 0 or index > 2:
        print("invalid index")
    motors[index].setPosition(pos)
    
def setVel(address, index, vel):
    if index < 0 or index > 2:
        print("invalid index")
    motors[index].setVelocity(vel)
    
def setTorque(address, index, torque):
    if index < 0 or index > 2:
        print("invalid index")
    motors[index].setTorque(torque)

    
def clearErrors(address, index):
    motors[index].clearErrors()


if __name__ == "__main__":
    bus = can.interface.Bus("can0", bustype="socketcan")
    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    print("[ODrive] CAN bus opened");

    # TODO check if CAN is working

    motors = [ODriveMotor(bus, 1), ODriveMotor(bus, 2), ODriveMotor(bus, 3)]

    dispatcher = Dispatcher()
    dispatcher.map("/pos", setPos)
    dispatcher.map("/vel", setVel)
    dispatcher.map("/torque", setTorque)
    dispatcher.map("/clear", clearErrors)

    server = osc_server.ThreadingOSCUDPServer(
        (LISTENING_IP, LISTENING_PORT), dispatcher, timeout=10)
    print("Serving on {}".format(server.server_address))
    
    t = threading.Thread(target=server.serve_forever)
    t.daemon = True
    t.start()

    while True:
        for motor in motors:
            motor.readCAN()