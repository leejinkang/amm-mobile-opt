import rclpy
from rclpy.node import Node

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

from trajectory import *
from utils import UTILS

'''

state : robot position [x, y, theta]
input : wheel steering / speed [gamma_fl,fr,rl,rr , omega_fl,fr,rl,rr]

cost : path tracking error minimize + input vector minimize + slip angle minimize
constraints : single ICR, min/max motor velocity, vehicle dynamic

'''

mpc_utils = UTILS()

class MPC:

    def __init__(self, trajecrory, initial_state):
        self.horizon = 5
        self.dt = 0.01
        self.trajectory = trajecrory
        self.state = initial_state

        self.U = np.ones(8 * self.horizon)
        self.Q = 0.1
        self.R = 0.1

    def CostFunction(self, *args):
        init_state, ref_points, N, Q, R = args
        cost = 0
        _state = np.copy(init_state)
        
        for t in range(N):
            _input = self.U [8*t:8*(t+1)]
            cost_state_error = mpc_utils.StateErrorCost(ref_points[t], _state, self.Q)
            cost_input_minimize = mpc_utils.InputMinimizeCost(self, _input, self.R)
            cost += cost_state_error + cost_input_minimize
            _state = mpc_utils.VehicleModel(_state, _input)
        
        return cost
    
    def constraint_function(self, U, *args):
        x_init, N = args
        x = np.copy(x_init)
        constraints = []
        
        for t in range(N):
            u = U[8*t:8*(t+1)]
            V_x = [u[4 + i] * r * np.cos(u[i]) for i in range(4)]
            V_y = [u[4 + i] * r * np.sin(u[i]) for i in range(4)]
            
            v_x = np.mean(V_x)
            v_y = np.mean(V_y)
            omega = (V_y[1] - V_y[3]) / L - (V_x[0] - V_x[2]) / W
            
            constraints.extend([
                V_x[0] - (v_x - omega * (L/2)),
                V_y[0] - (v_y + omega * (W/2)),
                V_x[1] - (v_x - omega * (L/2)),
                V_y[1] - (v_y - omega * (W/2)),
                V_x[2] - (v_x + omega * (L/2)),
                V_y[2] - (v_y + omega * (W/2)),
                V_x[3] - (v_x + omega * (L/2)),
                V_y[3] - (v_y - omega * (W/2))
            ])
            
            x = dynamics(x, u)
        
        return constraints
    def mpc(self):
   
        # 최적화 문제 설정
        args = (self.state, x_ref, N, Q, R)
        constraints = [{'type': 'eq', 'fun': constraint_function, 'args': (x_current, N)}]
        result = minimize(cost_function, U0, args=args, constraints=constraints, method='SLSQP', options={'disp': False, 'maxiter': 500})

        # 최적화 결과에서 입력 추출
        U_opt = result.x
        u_opt = U_opt[:8]
        # 상태 갱신
        x_current = dynamics(x_current, u_opt)
        x_history.append(x_current)
    
    def run(self):
        pass

if __name__ == '__main__':
    mpc = MPC()
    mpc.run()

