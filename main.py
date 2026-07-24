
from odrive import ODriveCANManager
import threading
import time
import math

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

LISTENING_IP = "127.0.0.1"
LISTENING_PORT = 12345

def setPos(address, index, value):
    odrive.setPos(index, value)
    
def setVel(address, index, value):
    odrive.setVel(index, value)
    
def setTorque(address, index, value):
    odrive.setTorque(index, value)

def clearErrors(address, index):
    odrive.clearErrors(index)


if __name__ == "__main__":
    
    odrive = ODriveCANManager(node_ids= [1,2,3])

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

    speed = 1.0
    last_time = time.time()*speed
    age_max = 0
    dt = []

    try:
        while True:
            age = round((time.monotonic() - odrive.latest_encoder[2].timestamp)*1000) # ms
            dt.append(age)
            if age > age_max:
                age_max = age
            time.sleep(0.001)
            
            t = math.floor(time.time()*speed)
            if t > last_time:
                last_time = t
                
                print("---")
                print("age de la derniere position:", round(sum(dt)/len(dt), 2), "ms (max ", age_max, ")")

                odrive.stats.print()

                for node_id in odrive.node_ids:
                    client.send_message("/pos", [odrive.latest_encoder[node_id].position for node_id in odrive.node_ids]) 
                    client.send_message("/vel", [odrive.latest_encoder[node_id].velocity for node_id in odrive.node_ids]) 


    except Exception as ex:
        print(f"Exception {ex} raised in main thread")

    finally:
        odrive.stopMotors()
