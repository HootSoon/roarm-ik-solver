from roarm_sdk.roarm import roarm
import math
import time

class m3Arm(roarm):
    '''
    RoArm IK solver & SDK Communication wrapper for m3-pro
    '''
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        super().__init__(roarm_type="roarm_m3", port=port, baudrate=baudrate)

        self.r1 = 10
        self.r2 = 7.5
        self.r3 = 7

        #Store X,Y,Z and Phi
        self.pos = [0, 0, 0, 0]
        self.angles = [0, 0, 90, 0, 0, 10]




    def generate_ik(self, phi, x, y, z):
        self.pos[0] = x
        self.pos[1] = y
        self.pos[2] = z
        self.pos[3] = phi
        
        phi_rad = math.radians(phi)
        base_angle_rad = math.atan2(y, x)
        R = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

        D3x = R
        D3y = z

        D2x = D3x - self.r3 * math.cos(phi_rad)
        D2y = D3y - self.r3 * math.sin(phi_rad)

        d = math.sqrt(math.pow(D2x, 2) + math.pow(D2y, 2))

        if d == 0 or d > (self.r1 + self.r2):
            return None

        a = math.atan2(D2y, D2x)
        
        cos_b = (math.pow(self.r1, 2) + math.pow(d, 2) - math.pow(self.r2, 2)) / (2 * self.r1 * d)
        b = math.acos(max(-1.0, min(1.0, cos_b)))
        
        math_theta1 = a + b

        cos_elbow = (math.pow(self.r1, 2) + math.pow(self.r2, 2) - math.pow(d, 2)) / (2 * self.r1 * self.r2)
        inner_elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))
        math_theta2 = math.pi - inner_elbow 

        robot_shoulder = (math.pi / 2) - math_theta1
        robot_elbow = math_theta2 
        
        robot_shoulder_deg = math.degrees(robot_shoulder)
        robot_elbow_deg = math.degrees(robot_elbow)
        robot_base_deg = math.degrees(base_angle_rad)

        robot_wrist_deg = 90 - robot_shoulder_deg - robot_elbow_deg - phi

        return [robot_base_deg, robot_shoulder_deg, robot_elbow_deg, robot_wrist_deg, 0, 10]



    def wait_for_arrival(self, target_angles, tolerance=2.0, timeout=3.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                current_angles = self.joints_angle_get()
                if current_angles is not None and len(current_angles) >= 4:
                    max_diff = max(abs(current_angles[i] - target_angles[i]) for i in range(4))
                    if max_diff <= tolerance:
                        break
            except TypeError:
                pass
            time.sleep(0.05)


    def move_to_xyz(self, phi, x, y, z, wait=True):
        angles = self.generate_ik(phi, x, y, z)
        if angles:
            self.joints_angle_ctrl(angles, 500, 254)
            if wait:
                self.wait_for_arrival(angles)



    def draw_line(self, phi, x1, y1, z1, x2, y2, z2, steps=30):
        last_valid_angles = None
        for i in range(steps + 1):
            t = i / steps
            cx = x1 + (x2 - x1) * t
            cy = y1 + (y2 - y1) * t
            cz = z1 + (z2 - z1) * t
            angles = self.generate_ik(phi, cx, cy, cz)
            if angles:
                self.joints_angle_ctrl(angles, 800, 254)
                last_valid_angles = angles
            time.sleep(0.05)
        
        if last_valid_angles:
            self.wait_for_arrival(last_valid_angles)


class m2Arm(roarm):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        super().__init__(roarm_type="roarm_m2", port=port, baudrate=baudrate)

        self.r1 = 10
        self.r2 = 12.5

        self.pos = [0, 0, 0]
        self.angles = [0, 0, 90, 0]

    def generate_ik(self, x, y, z):
        self.pos[0] = x
        self.pos[1] = y
        self.pos[2] = z

        base_angle_rad = math.atan2(y, x)
        R = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

        d = math.sqrt(math.pow(R, 2) + math.pow(z, 2))

        if d == 0 or d > (self.r1 + self.r2):
            return None

        a = math.atan2(z, R)
        
        cos_b = (math.pow(self.r1, 2) + math.pow(d, 2) - math.pow(self.r2, 2)) / (2 * self.r1 * d)
        b = math.acos(max(-1.0, min(1.0, cos_b)))
        
        math_theta1 = a + b

        cos_elbow = (math.pow(self.r1, 2) + math.pow(self.r2, 2) - math.pow(d, 2)) / (2 * self.r1 * self.r2)
        inner_elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))
        math_theta2 = math.pi - inner_elbow 

        robot_shoulder = (math.pi / 2) - math_theta1
        robot_elbow = math_theta2 
        
        robot_shoulder_deg = math.degrees(robot_shoulder)
        robot_elbow_deg = math.degrees(robot_elbow)
        robot_base_deg = math.degrees(base_angle_rad)

        return [robot_base_deg, robot_shoulder_deg, robot_elbow_deg, 0]


    def wait_for_arrival(self, target_angles, tolerance=2.0, timeout=3.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                current_angles = self.joints_angle_get()
                if current_angles is not None and len(current_angles) >= 3:
                    max_diff = max(abs(current_angles[i] - target_angles[i]) for i in range(3))
                    if max_diff <= tolerance:
                        break
            except TypeError:
                pass
            time.sleep(0.05)


    def move_to_xyz(self, x, y, z, wait=True):
        angles = self.generate_ik(x, y, z)
        if angles:
            self.joints_angle_ctrl(angles, 500, 254)
            if wait:
                self.wait_for_arrival(angles)


    def draw_line(self, x1, y1, z1, x2, y2, z2, steps=30):
        last_valid_angles = None
        for i in range(steps + 1):
            t = i / steps
            cx = x1 + (x2 - x1) * t
            cy = y1 + (y2 - y1) * t
            cz = z1 + (z2 - z1) * t
            angles = self.generate_ik(cx, cy, cz)
            if angles:
                self.joints_angle_ctrl(angles, 800, 254)
                last_valid_angles = angles
            time.sleep(0.05)
        
        if last_valid_angles:
            self.wait_for_arrival(last_valid_angles)
