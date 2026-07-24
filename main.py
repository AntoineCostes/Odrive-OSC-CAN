
from odrive import ODriveCANManager
from logger import Logger
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
    global odrive
    odrive.setPos(index, value)
    
def setVel(address, index, value):
    global odrive
    odrive.setVel(index, value)
    
def setTorque(address, index, value):
    global odrive
    odrive.setTorque(index, value)

def clearErrors(address, index):
    global odrive
    odrive.clearErrors(index)
    
def plot(address):
    global stop_and_plot
    stop_and_plot = True
    
def t0(address):
    global logger
    logger.reset()

    
class EncoderEstimate:
    position: float
    velocity: float
    timestamp: float


if __name__ == "__main__":
    
    odrive = ODriveCANManager(node_ids= [1,2,3])
    logger = Logger()
    stop_and_plot = False

    dispatcher = Dispatcher()
    dispatcher.map("/pos", setPos) # index, value
    dispatcher.map("/vel", setVel) # index, value
    dispatcher.map("/torque", setTorque) # index, value
    dispatcher.map("/clear", clearErrors) # index
    dispatcher.map("/plot", plot)
    dispatcher.map("/t0", t0)

    server = osc_server.ThreadingOSCUDPServer(
        (LISTENING_IP, LISTENING_PORT), dispatcher, timeout=10)
    print("Serving on {}".format(server.server_address))
    
    t = threading.Thread(target=server.serve_forever)
    t.daemon = True
    t.start()

    client = SimpleUDPClient("127.0.0.1", 12000)

    log_speed = .2
    last_time = time.time()*log_speed
    estimate_ages = [] # to compute mean, usually 5 ms

    last_estimates = {} # { node_id:estimate }

    logger.reset()
    try:
        while True:
            time.sleep(0.001) # let other threads breathe
            
            if stop_and_plot:
                print("STOP AND PLOT")
                odrive.stop() 
                logger.save_plots()
                stop_and_plot = False
                while True:
                    plot.pause(0.1)
                    time.sleep(1.0)
                    
                
            if odrive.fault_detected:
                print("ERROR ")
                print(odrive.getPendingErrors())
                break
            
            for node_id, estimate in odrive.getNewEstimates().items():
                if estimate is not None:
                    last_estimates[node_id] = copy(estimate)
                    
                                  
                    logger.record(
                        label="pos_"+str(node_id),
                        value=estimate.position,
                        timestamp=estimate.timestamp,
                        plot_id="position",
                        curve_id=str(node_id),
                        style_id="actual",
                    )
                    logger.record(
                        label="vel_"+str(node_id),
                        value=estimate.velocity,
                        timestamp=estimate.timestamp,
                        plot_id="velocity",
                        curve_id=str(node_id),
                        style_id="actual",
                    )
                    
                    # compute age
                    estimate_ages.append(round((time.monotonic() - estimate.timestamp) *1000)) # ms


                    
            # print stuff log_speed times by seconds
            t = math.floor(time.time()*log_speed)
            if t > last_time:
                last_time = t
                
                print("---")
                if len(estimate_ages):
                    print(len(estimate_ages), "positions received over", 1.0/log_speed, "s, age mean", round(sum(estimate_ages)/len(estimate_ages), 2), "ms, age max ", max(estimate_ages), "ms")
                    estimate_ages = []
                else:
                    print("pas de nouvelle position")
                
                odrive.stats.print()

                for node_id in odrive.node_ids:
                    #print([last_estimates[node_id].timestamp, last_estimates[node_id].position, last_estimates[node_id].velocity])
                    client.send_message("/pos", [last_estimates[node_id].position for node_id in odrive.node_ids]) 
                    client.send_message("/vel", [last_estimates[node_id].velocity for node_id in odrive.node_ids]) 


    except Exception as ex:
        print(f"Exception {ex} raised in main thread")

    finally:
        odrive.stop()
