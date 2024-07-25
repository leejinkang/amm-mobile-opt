import numpy as np


class UTILS:
    def __init__(self):

        self.r = 0.1  # wheel radius [m]
        self.dt = 0.1 # [s]
        
        self.body_x_length = 0.9  # forward
        self.body_y_length = 0.6
        self.wheel_x_length = 0.5
        self.wheel_y_length = 0.7
        self.wheel_radius = 0.12

        self.fr_xy = (0.35, -0.25)
        self.br_xy = (-0.35, -0.25)
        self.bl_xy = (-0.35, 0.25)
        self.fl_xy = (0.35, 0.25)


    def VehicleModel(self, _state, _u):
        '''
        assumption : single ICR(non slip)

        r : wheel radius
        u : input vector [delta_fl, delta_fr, delta_rl, delta_rr
                        , omega_fl, omega_fr, omega_rl, omega_rr]

        Vx, Vy : each wheel contact point velocity
        x_dot, y_dot, theta_dot : robot body velocity

        return : next time robot body position new_x, new_y, new_theta
        '''

        V_x = [_u[4 + i] * self.r * np.cos(_u[i]) for i in range(4)]
        V_y = [_u[4 + i] * self.r * np.sin(_u[i]) for i in range(4)]
        
        x_dot = np.mean(V_x)
        y_dot = np.mean(V_y)
        theta_dot = (V_y[1] - V_y[3]) / self.wheel_y_length - (V_x[0] - V_x[2]) / self.wheel_x_length
        
        new_x = _state[0] + x_dot * np.cos(_state[2]) * self.dt - y_dot * np.sin(_state[2]) * self.dt
        new_y = _state[1] + x_dot * np.sin(_state[2]) * self.dt + y_dot * np.cos(_state[2]) * self.dt
        new_theta = _state[2] + theta_dot * self.dt
        
        return np.array([new_x, new_y, new_theta])


    def StateErrorCost(self, ref_point, _state, weight):
        '''
        input : ref_point, _state(x, y, theta), weight
        output : each step tracking error cost
        '''

        err_mat = np.array([ref_point[0] - _state[0], ref_point[1] - _state[1], ref_point[2] - _state[2]])
        cost = err_mat.T @ weight @ err_mat

        return cost
        
    def InputMinimizeCost(self, _input, weight):
        cost = _input.T @ weight @ _input
        return cost


               


    def SingleICR(self, x_dot, y_dot, theta_dot):
        
        '''
        input : body velocity x_dot, y_dot, theta_dot
        output : equelity constraints delta_i, omega_i

        Single ICR constraint

        
        '''

        x = np.copy(x_init)
        constraints = []
        
        for t in range(N):
            u = U[8*t:8*(t+1)]
            V_x = [u[4 + i] * self.wheel_radius * np.cos(u[i]) for i in range(4)]
            V_y = [u[4 + i] * self.wheel_radius * np.sin(u[i]) for i in range(4)]
            
            v_x = np.mean(V_x)
            v_y = np.mean(V_y)
            omega = (V_y[1] - V_y[3]) / self.wheel_y_length - (V_x[0] - V_x[2]) / W
            
            constraints.extend([
                V_x[0] - (v_x - omega * (self.wheel_y_length/2)),
                V_y[0] - (v_y + omega * (self.wheel_x_length/2)),
                V_x[1] - (v_x - omega * (self.wheel_y_length/2)),
                V_y[1] - (v_y - omega * (self.wheel_x_length/2)),
                V_x[2] - (v_x + omega * (self.wheel_y_length/2)),
                V_y[2] - (v_y + omega * (self.wheel_x_length/2)),
                V_x[3] - (v_x + omega * (self.wheel_y_length/2)),
                V_y[3] - (v_y - omega * (self.wheel_x_length/2))
            ])
            
            x = dynamics(x, u)
        
        return constraints
    

