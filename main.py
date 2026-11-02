
from odrive import ODriveCANManager
from logger import Logger
from spatializer import Spatializer
import threading
import time
import math
from copy import copy

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
from pythonosc.udp_client import SimpleUDPClient

LISTENING_IP = "127.0.0.1"
LISTENING_PORT = 12345

# Ã  adapter selon le rÃ©seau de fil. C est gÃ©nÃ©ralement en 2
A_id = 1
B_id = 3
C_id = 2
node_ids = [A_id, B_id, C_id] # pour le client OSC 0 = A, 1 = B, 2 = C

def setPos(address, index, value):
    global odrive, logger
    odrive.setPos(node_ids[index], value)

    logger.record(
        label="pos_"+str(index),
        value=value,
        timestamp=time.monotonic(),
        plot_id="position",
        curve_id=str(index),
        style_id="target",
    )
    
def setVel(address, index, value):
    global odrive, logger
    odrive.setVel(node_ids[index], value)
    
    logger.record(
        label="vel_"+str(index),
        value=value,
        timestamp=time.monotonic(),
        plot_id="velocity",
        curve_id=str(index),
        style_id="target",
    )
    
def set3DVel(address, vx, vy, vz):
    global odrive, logger, spatializer, radius

    t = time.monotonic()
    logger.record(
        label="vel_x",
        value=vx,
        timestamp=t,
        plot_id="velocity",
        curve_id=str(4),
        style_id="target",
    )
    logger.record(
        label="vel_y",
        value=vy,
        timestamp=t,
        plot_id="velocity",
        curve_id=str(5),
        style_id="target",
    )
    logger.record(
        label="vel_z",
        value=vz,
        timestamp=t,
        plot_id="velocity",
        curve_id=str(6),
        style_id="target",
    )
    positions = [last_estimates[A_id].position, last_estimates[B_id].position, last_estimates[C_id].position]
    v_motors = spatializer.velTransform([vx, vy, vz], positions, radius)
    
    
    if v_motors is None:
        odrive.setVel(A_id, 0.0)
        odrive.setVel(B_id, 0.0)
        odrive.setVel(C_id, 0.0)
    else:
        odrive.setVel(A_id, v_motors[0])
        odrive.setVel(B_id, v_motors[1])
        odrive.setVel(C_id, v_motors[2])
        
        logger.record(
            label="motor_x",
            value=v_motors[0],
            timestamp=t,
            plot_id="velocity",
            curve_id=str(1),
            style_id="actual",
        )
        logger.record(
            label="motor_y",
            value=v_motors[1],
            timestamp=t,
            plot_id="velocity",
            curve_id=str(2),
            style_id="actual",
        )
        logger.record(
            label="motor_z",
            value=v_motors[2],
            timestamp=t,
            plot_id="velocity",
            curve_id=str(3),
            style_id="actual",
        )
    
def setTorque(address, index, value):
    global odrive
    odrive.setTorque(node_ids[index], value)
    print("torque");

def clearErrors(address, index):
    global odrive
    odrive.clearErrors(node_ids[index])
    
def plot(address):
    global stop_and_plot
    stop_and_plot = True
    
def reset(address):
    global logger
    logger.reset()

def calibrate(address, value):
    global calibration_step
    calibration_step = value

    if calibration_step == 0:
        print("==== FIN DE CALIBRATION")
    elif calibration_step == 1:
        print("==== DEBUT DE CALIBRATION ")
        print("Amener le mobile au point A puis valider")
    elif calibration_step == 2:
        odrive.resetPosistion(A_id)
        print("moteur A initialisÃ‰")
        print("descendre a la verticale du point A puis valider")
    elif calibration_step == 3: 
        hA = round(last_estimates[A_id].position*2*3.14*0.02,2)
        print("Hauteur du point A : ", hA)
        print("Amener le mobile au point B puis valider")
    elif calibration_step == 4:
        odrive.resetPosistion(B_id)
        print("moteur B initialisÃ‰")
        print("descendre Ã€ la verticale du point B puis valider")
    elif calibration_step == 5:
        hB = round(last_estimates[B_id].position*2*3.14*0.02,2)
        print("Hauteur du point B : ", hB)
        print("Amener le mobile au point C puis valider")
    elif calibration_step == 6:
        odrive.resetPosistion(C_id)
        print("moteur C initialisÃ‰")
        print("descendre Ã€ la verticale du point C puis valider")
    elif calibration_step == 7:
        hC = round(last_estimates[C_id].position*2*3.14*0.02,2)
        print("Hauteur du point C : ", hC)
        print("Calibration terminÃ‰se !")
        calibration_step = 0

    
class EncoderEstimate:
    position: float
    velocity: float
    timestamp: float


if __name__ == "__main__":
    
    odrive = ODriveCANManager()
    logger = Logger()
    spatializer = Spatializer()
    stop_and_plot = False
    calibration_step = 0
    radius = [0.025, 0.025, 0.025]

    dispatcher = Dispatcher()
    dispatcher.map("/pos", setPos) # index, value
    dispatcher.map("/vel", setVel) # index, value
    dispatcher.map("/3d", set3DVel) # index, value
    dispatcher.map("/torque", setTorque) # index, value
    dispatcher.map("/clear", clearErrors) # index
    dispatcher.map("/plot", plot)
    dispatcher.map("/reset", reset)
    dispatcher.map("/calibrate", calibrate)

    server = osc_server.ThreadingOSCUDPServer(
        (LISTENING_IP, LISTENING_PORT), dispatcher, timeout=10)
    print("Serving on {}".format(server.server_address))
    
    t = threading.Thread(target=server.serve_forever)
    t.daemon = True
    t.start()

    client = SimpleUDPClient("127.0.0.1", 12000)

    log_speed = 0.1
    last_time = time.time()*log_speed
    estimate_ages = [] # to compute mean, usually 5 ms

    last_estimates = {} # { node_id:estimate }

    logger.reset()
    try:
        while True:
            time.sleep(0.001) # let other threads breathe
            
            if stop_and_plot:
                print("STOP AND PLOT")
                odrive.stopMotors()
                logger.save_plots()
                stop_and_plot = False

            # log pos_vel
            for node_id, estimate in odrive.getNewEstimates().items():
                if estimate is not None:
                    last_estimates[node_id] = copy(estimate)
                     
                    #print([last_estimates[i] for i in odrive.node_ids])
                    #if all([last_estimates[node_id] is not None for node_id in odrive.node_ids]): 
                    #    print("pos", [round(last_estimates[node_id].position,2) for node_id in odrive.node_ids])#, "vel", [round(last_estimates[node_id].velocity,2) for node_id in odrive.node_ids])
                                                  
                    logger.record(
                        label="pos_"+str(node_id),
                        value=estimate.position,
                        timestamp=estimate.timestamp,
                        plot_id="position",
                        curve_id=str(node_id),
                        style_id="actual",
                    )
                    #logger.record(
                    #    label="vel_"+str(node_id),
                    #    value=estimate.velocity,
                    #    timestamp=estimate.timestamp,
                    #    plot_id="velocity",
                    #    curve_id=str(node_id),
                    #    style_id="actual",
                    #)
                    
                    # compute age
                    estimate_ages.append(round((time.monotonic() - estimate.timestamp) *1000)) # ms

                    
            # log_speed times by seconds, print stuff
            
            t = math.floor(time.time()*log_speed)
            if t > last_time:
                last_time = t
                
                print("---")
                # print CAN perfs
                if False:
                    if len(estimate_ages):
                        print(len(estimate_ages), "positions received over", 1.0/log_speed, "s, age mean", round(sum(estimate_ages)/len(estimate_ages), 2), "ms, age max ", max(estimate_ages), "ms")
                        estimate_ages = []
                    else:
                        print("pas de nouvelle position")
                    
                    odrive.stats.print()
                
                print("length (A, B, C)", [round(last_estimates[node_ids[index]].position*2*3.14*radius[index],2) for index in [0, 1, 2]])
                #print("pos", [round(last_estimates[node_id].position*2*3.14*0.02,2) for node_id in odrive.node_ids])#, "vel", [round(last_estimates[node_id].velocity,2) for node_id in odrive.node_ids])
                          
                #for node_id in odrive.node_ids:
                    #print(node_id, [round(last_estimates[node_id].timestamp,2), round(last_estimates[node_id].position,2), round(last_estimates[node_id].velocity,2)])
                    #client.send_message("/pos", [last_estimates[node_id].position for node_id in odrive.node_ids]) 
                    #client.send_message("/vel", [last_estimates[node_id].velocity for node_id in odrive.node_ids]) 


    except Exception as ex:
        print(f"Exception {ex} raised in main thread")

    finally:
        print("=================== EXCEPTION, STOP AND EXIT ============");
        odrive.stop()
