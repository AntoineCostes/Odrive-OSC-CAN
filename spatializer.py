import csv
import numpy as np

PI = 3.1415926535

class Spatializer:
    def __init__(self, filepath  = "./calibration.csv"):
        self.nodes_id = [3, 2, 1]
        self.radius = [0.0395/2, 0.0202, 0.021]
        self.l0 = [0.0, 0.0, 0.0] # 0.0 if calibration is done
        
        params = {}

        with open(filepath, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                variable = row["variable"]
                valeur = row["value"]
                params[variable] = valeur

        # Distances between the 3 attachment points A, B, and C in meters
        self.l_AB = float(params["l_AB"])
        self.l_BC = float(params["l_BC"])
        self.l_CA = float(params["l_CA"])
        # height of the 3 attachment points in meters
        h_A = float(params["h_A"])
        h_B = float(params["h_B"])
        h_C = float(params["h_C"])
        
        #=======================================#
        # Computation of characteristic lengths #
        #=======================================#

        # postion of A, B, C in the robot coordinate
        # Alk-kashi: lBC² = lAB² + lCA² -2*lAB*lCA*Da/lCA
        self.DA = (self.l_AB**2+self.l_CA**2-self.l_BC**2)/(2*self.l_AB)
        self.DB = self.l_AB - self.DA
        self.DC = np.sqrt(self.l_CA**2-self.DA**2) # pythagore

        # Projection of D, E, F are projections of A, B, C on the ground
        l_DE = np.sqrt(self.l_AB**2-(h_A-h_B)**2)
        l_EF = np.sqrt(self.l_BC**2-(h_B-h_C)**2)
        l_FD = np.sqrt(self.l_CA**2-(h_C-h_A)**2)

        DD = (l_DE**2+l_FD**2-l_EF**2)/(2*l_DE) # Al kashi
        DE = l_DE - DD
        DF = np.sqrt(l_FD**2-DD**2)


        #====================================#
        # Computation of the rotation matrix #
        #====================================#

        # Position of A, B, C in the ground coordinate
        A = np.array([-DD,0,h_A])
        B = np.array([DE,0,h_B])
        C = np.array([0,DF,h_C])

        # Computation of the expression of a base of the robot coordinate in the ground coordinates
        e_x = B - A
        e_x = e_x/np.linalg.norm(e_x)

        e_y = C - A
        e_y = e_y - e_x*e_y.dot(e_x)
        e_y = e_y/np.linalg.norm(e_y)

        e_z = np.cross(e_x,e_y)
        e_z = e_z/np.linalg.norm(e_z)

        self.R_s_r = np.vstack([e_x, e_y, e_z])


        
    def velTransform(self, velocity_setpoint, motor_turns):
        
        if len(velocity_setpoint) != 3 or len(motor_turns) != 3:
            print("ERROR vel and pos should be 3D")
            return [0.0, 0.0, 0.0]

        # compute position in ground reference
        lengths = [
                2*PI*motor_turns[0]*self.radius[0]+self.l0[0],
                2*PI*motor_turns[1]*self.radius[1]+self.l0[1],
                2*PI*motor_turns[2]*self.radius[2]+self.l0[2]
                ]
        
        if (abs(lengths[0] - lengths[1]) > self.l_AB):
            print("No intersection between spheres A and B: one is included in the other")
            return None
        if (lengths[0] + lengths[1] < self.l_AB):
            print("No intersection between spheres A and B: Radius are to short")
            return None

        if abs(lengths[1] - lengths[2]) > self.l_BC:
            print("No intersection between spheres B and C: one is included in the other")
            return None
        if (lengths[1] + lengths[2] < self.l_BC):
            print("No intersection between spheres B and C: Radius are to short")
            return None
        if abs(lengths[0] - lengths[2]) > self.l_CA:
            print("No intersection between spheres C and A: one is included in the other")
            return None
        if (lengths[0] + lengths[2] < self.l_CA):
            print("No intersection between spheres C and A: Radius are to short")
            return None

        
        x = (lengths[0]**2 - lengths[1]**2 + self.DB**2 - self.DA**2) / (2* (self.DA+self.DB))
        y = (-2 * self.DA * x + self.DC**2 - self.DA**2 - lengths[2]**2 + lengths[0]**2) / (2 * self.DC)
        z2 = lengths[2]**2 - x**2 -(self.DC-y)**2
        if z2 < 0 :
            print("No intersection with sphere C")
            return None
        z = - np.sqrt(z2)

        position_ground = [x, y, z]
        print("position_ground", position_ground)
        
        # calculate speed command
        v = self.R_s_r @ velocity_setpoint
        print(velocity_setpoint, " => ", v)
        v_a = (v[0]*(self.DA+x) + v[1]*y + v[2]*z)/( lengths[0] * self.radius[0] * 2 * PI)
        v_b = (-v[0]*(self.DB-x) + v[1]*y + v[2]*z)/( lengths[1] * self.radius[1] * 2 * PI)
        v_c = (v[0]*x - v[1]*(self.DC-y) + v[2]*z)/( lengths[2] * self.radius[2] * 2 * PI)

        velocity_motors = [v_a, v_b, v_c]
        print("velocity_motors", velocity_motors)
