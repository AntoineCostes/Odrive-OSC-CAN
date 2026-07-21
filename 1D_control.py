
from odrive import ODriveManager
import threading
import time

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

LISTENING_IP = "127.0.0.1"
LISTENING_PORT = 12345

def setPos(address, index, value):
    odrive.setPos(index, value)
    
def setVel(address, index, value):
    odrive.setPos(index, value)
    
def setTorque(address, index, value):
    odrive.setTorque(index, value)

def clearErrors(address, index):
    odrive.clearErrors(index)


if __name__ == "__main__":
    
    odrive = ODriveManager()

    dispatcher = Dispatcher()
    dispatcher.map("/pos", setPos) # index, value
    dispatcher.map("/vel", setVel) # index, value
    dispatcher.map("/torque", setTorque) # index, value
    dispatcher.map("/clear", clearErrors) # index

    server = osc_server.ThreadingOSCUDPServer(
        (LISTENING_IP, LISTENING_PORT), dispatcher, timeout=10)
    print("Serving on {}".format(server.server_address))
    
    t = threading.Thread(target=server.serve_forever)
    t.daemon = True
    t.start()

    client = SimpleUDPClient("127.0.0.1", 12000)

    last_time = time.time()

    try:
        while True:
            odrive.update()

            t = round(time.time()*1.0)
            if t > last_time:
                last_time = t
                print("log")
                client.send_message("/pos", [motor.pos for motor in odrive.motors]) 
                client.send_message("/vel", [motor.vel for motor in odrive.motors]) 

    except Exception as ex:
        print(f"Exception {ex} raised in main thread")

    finally:
        for motor in odrive.motors:
            motor.setVelocity(0.0)