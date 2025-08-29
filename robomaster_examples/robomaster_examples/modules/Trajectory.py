# TODO verify the yaw control law.

import numpy as np
import yaml

def normalize(v):
    norm = np.linalg.norm(v)
    assert norm > 0
    return v / norm

class Polynomial:
    def __init__(self, p):
        self.p = p

    # evaluate a polynomial using horner's rule
    def eval(self, t):
        assert t >= 0
        x = 0.0
        for i in range(0, len(self.p)):
            x = x * t + self.p[len(self.p) - 1 - i]
        return x

    def derivative(self):
        return Polynomial([(i+1) * self.p[i+1] for i in range(0, len(self.p) - 1)])


class TrajectoryOutput:
    def __init__(self):
        self.pos = None   # position [m]
        self.vel = None   # velocity [m/s]
        self.acc = None   # acceleration [m/s^2]
        self.omega = None # angular velocity [rad/s]
        self.yaw = None   # yaw angle [rad]
        self.v_filtered = None # filtered velocity [m/s]


# 4d single polynomial piece for x-y-z-yaw, includes duration.
class Polynomial4D:
    def __init__(self, duration, px, py, pz, pyaw):
        self.duration = duration
        self.px = Polynomial(px)
        self.py = Polynomial(py)
        self.pz = Polynomial(pz)
        self.pyaw = Polynomial(pyaw)
        self._v_f_t = np.array(np.zeros(3))

    def derivative(self):
        return Polynomial4D(
        self.duration,
        self.px.derivative().p,
        self.py.derivative().p,
        self.pz.derivative().p,
        self.pyaw.derivative().p)

    def eval(self, t):
        result = TrajectoryOutput()
        # flat variables
        result.pos = np.array([self.px.eval(t), self.py.eval(t), self.pz.eval(t)])
        result.yaw = self.pyaw.eval(t)

        # 1st derivative
        derivative = self.derivative()
        result.vel = np.array([derivative.px.eval(t), derivative.py.eval(t), derivative.pz.eval(t)])
        dyaw = derivative.pyaw.eval(t)

        # 2nd derivative
        derivative2 = derivative.derivative()
        result.acc = np.array([derivative2.px.eval(t), derivative2.py.eval(t), derivative2.pz.eval(t)])

        # 3rd derivative
        derivative3 = derivative2.derivative()
        jerk = np.array([derivative3.px.eval(t), derivative3.py.eval(t), derivative3.pz.eval(t)])

        thrust = result.acc + np.array([0, 0, 9.81]) # add gravity

        z_body = normalize(thrust)
        x_world = np.array([np.cos(result.yaw), np.sin(result.yaw), 0])
        y_body = normalize(np.cross(z_body, x_world))
        x_body = np.cross(y_body, z_body)

        jerk_orth_zbody = jerk - (np.dot(jerk, z_body) * z_body)
        h_w = jerk_orth_zbody / np.linalg.norm(thrust)

        result.omega = np.array([-np.dot(h_w, y_body), np.dot(h_w, x_body), z_body[2] * dyaw])
        result.v_filtered = np.array(self._v_f_t)
        return result

class TrajectoryAsFn:
    def __init__(self, x, y, z, vx, vy, vz, t, yaw=None, omega=None):
        exec(f"self.x = lambda {str(t)}: "+str(x))
        exec(f"self.y = lambda {str(t)}: "+str(y))
        exec(f"self.z = lambda {str(t)}: "+str(z))
        exec(f"self.vx = lambda {str(t)}: "+str(vx))
        exec(f"self.vy = lambda {str(t)}: "+str(vy))
        exec(f"self.vz = lambda {str(t)}: "+str(vz))
        
        # TODO 
        # Add yaw and yaw rate (omega) functions if provided
        if yaw is not None:
            exec(f"self.yaw = lambda {str(t)}: "+str(yaw))
        else:
            self.yaw = lambda t: 0.0
            
        if omega is not None:
            exec(f"self.omega_z = lambda {str(t)}: "+str(omega))
        else:
            self.omega_z = lambda t: 0.0
            
        self._v_f_t = np.array(np.zeros(3))
        
    def eval(self, t):
        result = TrajectoryOutput()
        result.pos = np.array([self.x(t), self.y(t), self.z(t)])
        result.vel = np.array([self.vx(t), self.vy(t), self.vz(t)])
        result.yaw = self.yaw(t)
        result.omega = np.array([0.0, 0.0, self.omega_z(t)])  # TODO verify Only using z component for yaw rate
        result.v_filtered = np.array(self._v_f_t)
        return result

class Trajectory:
    def __init__(self):
        self.polynomials = None
        self.duration = None
        self.asFn = False
        self.nb_drones = None

    def load(self, filename, i, x=None, y=None, z=None, vx=None, vy=None, vz=None, yaw=None, omega=None, time=None, duration=None):
        if filename.endswith(".yml") or filename.endswith(".yaml"):
            self.asFn = True
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
                self.duration = data["duration"]
                if "import" in data:
                    for imp in data["import"]:
                        exec("import "+imp)
                self.nb_drones = 1
                self.polynomials = []
                
                # TODO verify
                # Check if yaw and omega parameters exist in YAML
                yaw_param = data.get(f"yaw{str(i)}", "0.0")  # Default to 0 if not specified
                omega_param = data.get(f"omega{str(i)}", "0.0")  # Default to 0 if not specified
                
                self.polynomials.append(TrajectoryAsFn(
                    data[f"x{str(i)}"],
                    data[f"y{str(i)}"],
                    data[f"z{str(i)}"],
                    data[f"vx{str(i)}"],
                    data[f"vy{str(i)}"],
                    data[f"vz{str(i)}"],
                    data["time"],
                    yaw_param,
                    omega_param
                ))
                f.close()
        elif filename.endswith(".csv"):
            self.nb_drones = 1
            data = np.loadtxt(filename, delimiter=",", skiprows=1, usecols=range(33))
            self.polynomials = [Polynomial4D(row[0], row[1:9], row[9:17], row[17:25], row[25:33]) for row in data]
            self.duration = np.sum(data[:,0])
        elif x and y and z and vx and vy and vz and time and duration is not None:
            self.asFn = True
            self.duration = duration
            self.nb_drones = 1
            self.polynomials = [TrajectoryAsFn(x, y, z, vx, vy, vz, time, yaw, omega)]
        else:
            raise ValueError(f"Unknown file format or missing parameters")
    
    def eval(self, t, drone_no=1):
        assert t >= 0, f"Time cannot be negative, got {t}"
        if t > self.duration:
            print(f"Time out of bounds: {t} > {self.duration}")
            return None

        if not self.asFn:
            current_t = 0.0
            for p in self.polynomials:
                if t < current_t + p.duration:
                    return p.eval(t - current_t)
                current_t = current_t + p.duration
        if self.asFn:
            return self.polynomials[drone_no-1].eval(t)

    def get_control(self, t, dt, pose:np.ndarray, prev_pose:np.ndarray, drone_no=1):
        '''
        Compute position control for a drone to follow the trajectory for X,Y,Z
        '''
        assert pose.shape == prev_pose.shape, f"Pose shapes must match, got {pose.shape} and {prev_pose.shape}"
        assert len(pose) >= 3, f"Pose must have at least 3 elements, got {len(pose)}"
        
        # Position control gains
        kpx = 0.6
        kvx = 0.1
        kpy = 0.6
        kvy = 0.1
        kpz = 2.5
        kvz = 0.1
        
        # TODO verify
        # Yaw control gain
        kp_yaw = 0.8
        kv_yaw = 0.1
        
        alpha = 0.7
        e = self.eval(t, drone_no)
        if e is None:
            if len(pose) > 3:
                return None, None, None, None
            return None, None, None
            
        p_ref = e.pos
        v_ref = e.vel
        yaw_ref = e.yaw if hasattr(e, 'yaw') else 0.0
        omega_ref = e.omega[2] if hasattr(e, 'omega') and e.omega is not None else 0.0

        # Position velocity filtering (for x, y, z)
        v_raw = (pose[:3] - prev_pose[:3]) / dt
        v_f_t1 = alpha * e.v_filtered + (1-alpha) * v_raw
        self.polynomials[drone_no-1]._v_f_t = v_f_t1

        # Position control
        u = np.zeros(4)  # TODO including yaw control
        u[0] = kpx * (p_ref[0] - pose[0]) + kvx * (v_ref[0] - v_f_t1[0])
        u[1] = kpy * (p_ref[1] - pose[1]) + kvy * (v_ref[1] - v_f_t1[1])
        u[2] = kpz * (p_ref[2] - pose[2]) + kvz * (v_ref[2] - v_f_t1[2])
        u[3] = 0.0
        yaw_ref = 0.0
        
        # Yaw control if available in pose
        if len(pose) > 3 and len(prev_pose) > 3:
            yaw = pose[3]
            prev_yaw = prev_pose[3]
            
            # Calculate yaw rate
            yaw_rate_raw = (yaw - prev_yaw) / dt
            
            # Normalize yaw error to [-π, π]
            yaw_error = yaw_ref - yaw
            while yaw_error > np.pi:
                yaw_error -= 2 * np.pi
            while yaw_error < -np.pi:
                yaw_error += 2 * np.pi
                
            u[3] = kp_yaw * yaw_error + kv_yaw * (omega_ref - yaw_rate_raw)
        return u, v_ref, p_ref, yaw_ref