
from odrive import ODriveCANManager
from logger import DataLogger
import threading
import time
import math
from copy import copy

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

    
class EncoderEstimate:
    position: float
    velocity: float
    timestamp: float


if __name__ == "__main__":
    
    odrive = ODriveCANManager(node_ids= [1,2,3])
    logger = DataLogger()

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

    log_speed = 1.0
    last_time = time.time()*log_speed
    estimate_ages = [] # to compute mean, usually 5 ms

    last_estimates = {} # { node_id:estimate }

    try:
        while True:
            time.sleep(0.001) # let other threads breathe

            if odrive.fault_detected:
                print("ERROR ")
                print(odrive.getPendingErrors())
                break
            
            for node_id, estimate in odrive.getNewEstimates().items():
                if estimate is not None:
                    last_estimates[node_id] = copy(estimate)
                    
                    logger.appendPoint("pos_"+str(node_id), estimate.timestamp, estimate.position)
                    logger.appendPoint("vel_"+str(node_id), estimate.timestamp, estimate.velocity)
                    
                    # compute age
                    estimate_ages.append(round((time.monotonic() - estimate.timestamp) *1000)) # ms


                    
            # print stuff log_speed times by seconds
            t = math.floor(time.time()*log_speed)
            if t > last_time:
                last_time = t
                
                print("---")
                if len(estimate_ages):
                    print(len(estimate_ages), "positions received, age mean", round(sum(estimate_ages)/len(estimate_ages), 2), "ms, age max ", max(estimate_ages), "ms")
                    estimate_ages = []
                else:
                    print("pas de nouvelle position")
                
                odrive.stats.print()

                for node_id in odrive.node_ids:
                    client.send_message("/pos", [last_estimates[node_id].position for node_id in odrive.node_ids]) 
                    client.send_message("/vel", [last_estimates[node_id].velocity for node_id in odrive.node_ids]) 


    except Exception as ex:
        print(f"Exception {ex} raised in main thread")

    finally:
        odrive.stopMotors()
