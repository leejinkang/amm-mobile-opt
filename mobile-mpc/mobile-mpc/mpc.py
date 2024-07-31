import numpy as np
import time
from trajectory import *
from utils import UTILS

from pydrake.all import MathematicalProgram, Solve, Variable
from pydrake.solvers import MathematicalProgram, Solve

'''

state : robot velocity [vx, vy, w]
input : wheel velocity [vx_[fl,fr,rl,rr] , vy_[fl,fr,rl,rr]

cost : reference body velocity error minimize + input minimize + slip angle minimize
constraints : min/max wheel velocity

'''

mpc_utils = UTILS()

class MPC:

    def __init__(self):
        self.horizon = 5
        self.dt = 0.01

        self.rx = 0.1
        self.ry = 0.1
        self.U = np.ones(8 * self.horizon) # vx_[fl, fr, rl, rr], vy_[fl, fr, rl, rr]
        self.Q = {'vx' : 0.1, 'vy' : 0.1, 'w' : 0.1}
        self.R = {'vx_' : 10., 'vy_' : 10.}

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0

    def mpc(self, ref_vx, ref_vy, ref_w):
   
        prog = MathematicalProgram()

        # state
        vx = prog.NewContinuousVariables(self.horizon + 1, 'vx')
        vy = prog.NewContinuousVariables(self.horizon + 1, 'vy')
        w = prog.NewContinuousVariables(self.horizon + 1, 'w')

        # input
        vx_fl = prog.NewContinuousVariables(self.horizon, 'vx_fl')
        vx_fr = prog.NewContinuousVariables(self.horizon, 'vx_fr')
        vx_rl = prog.NewContinuousVariables(self.horizon, 'vx_rl')
        vx_rr = prog.NewContinuousVariables(self.horizon, 'vx_rr')
        vy_fl = prog.NewContinuousVariables(self.horizon, 'vy_fl')
        vy_fr = prog.NewContinuousVariables(self.horizon, 'vy_fr')
        vy_rl = prog.NewContinuousVariables(self.horizon, 'vy_rl')
        vy_rr = prog.NewContinuousVariables(self.horizon, 'vy_rr')

        # initial state setting
        prog.AddBoundingBoxConstraint(self.vx, self.vx, vx[0])
        prog.AddBoundingBoxConstraint(self.vy, self.vy, vy[0])
        prog.AddBoundingBoxConstraint(self.w, self.w, w[0])

        #kinematics constraints
        K = 4*(self.rx**2 + self.ry**2)
        for i in range(self.horizon):
            prog.AddConstraint(vx[i+1] == (vx_fl[i] + vx_fr[i] + vx_rl[i] + vx_rr[i]) / 4)
            prog.AddConstraint(vy[i+1] == (vy_fl[i] + vy_fr[i] + vy_rl[i] + vy_rr[i]) / 4)
            prog.AddConstraint(w[i+1] == (-self.rx/K*vx_fl[i] + self.ry/K*vy_fl[i]) +
                                            (self.rx/K*vx_fr[i] + self.ry/K*vy_fr[i]) + 
                                            (-self.rx/K*vx_rl[i] - self.ry/K*vy_rl[i]) +
                                            (self.rx/K*vx_rr[i] - self.ry/K*vy_rr[i]))

        #cost function : target velocity error
        for i in range(self.horizon+1):
            prog.AddCost(self.Q['vx'] * (vx[i] - ref_vx[i])**2)
            prog.AddCost(self.Q['vy'] * (vy[i] - ref_vy[i])**2)
            prog.AddCost(self.Q['w'] * (w[i] - ref_w[i])**2)

        # cost function : 입력 변화 최소화
        for i in range(self.horizon-1):
            prog.AddCost(self.R['vx_'] * (vx_fl[i+1] - vx_fl[i])**2)
            prog.AddCost(self.R['vx_'] * (vx_fr[i+1] - vx_fr[i])**2)
            prog.AddCost(self.R['vx_'] * (vx_rl[i+1] - vx_rl[i])**2)
            prog.AddCost(self.R['vx_'] * (vx_rr[i+1] - vx_rr[i])**2)
            prog.AddCost(self.R['vy_'] * (vy_fl[i+1] - vy_fl[i])**2)
            prog.AddCost(self.R['vy_'] * (vy_fr[i+1] - vy_fr[i])**2)
            prog.AddCost(self.R['vy_'] * (vy_rl[i+1] - vy_rl[i])**2)
            prog.AddCost(self.R['vy_'] * (vy_rr[i+1] - vy_rr[i])**2)

        result = Solve(prog)

        opt_vx = result.GetSolution(vx)
        opt_vy = result.GetSolution(vy)
        opt_w  = result.GetSolution(w)

        opt_vx_fl = result.GetSolution(vx_fl)
        opt_vx_fr = result.GetSolution(vx_fr)
        opt_vx_rl = result.GetSolution(vx_rl)
        opt_vx_rr = result.GetSolution(vx_rr)
        opt_vy_fl = result.GetSolution(vy_fl)
        opt_vy_fr = result.GetSolution(vy_fr)
        opt_vy_rl = result.GetSolution(vy_rl)
        opt_vy_rr = result.GetSolution(vy_rr)

        return [opt_vx, opt_vy, opt_w], [opt_vx_fl[0], opt_vx_fr[0], 
                                         opt_vx_rl[0], opt_vx_rr[0], 
                                         opt_vy_fl[0], opt_vy_fr[0], 
                                         opt_vy_rl[0], opt_vy_rr[0]]
    
    def main(self):

        ref_vx = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0])
        ref_vy = np.array([0., 0., 0., 0., 0., 0.])
        ref_w  = np.array([1., 1., 1., 1., 1., 1.])

        prev_time = time.time()
        self.fl_xy = (0.35, 0.25)
        self.fr_xy = (0.35, -0.25)
        self.rl_xy = (-0.35, 0.25)
        self.rr_xy = (-0.35, -0.25)
        x_coords = np.array([0.35, 0.35, -0.35, -0.35]) * 10
        y_coords = np.array([0.25, -0.25, 0.25, -0.25]) * 10
        vx_coords = [0., 0., 0., 0.]
        vy_coords = [0., 0., 0., 0.]
        plt.ion()

        fig, ax = plt.subplots()
        quiver = ax.quiver(x_coords, y_coords, vx_coords, vy_coords, angles='xy', scale_units='xy', scale=1)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Real-time Vector Field Visualization')
        ax.grid()

        timer = 0.

        while True:
            current_time = time.time()
            dt = current_time - prev_time
            timer += dt
            ref_w = np.array([1., 1., 1., 1., 1., 1.]) * np.sin(timer) * 20
            opt_state, opt_input = self.mpc(ref_vx, ref_vy, ref_w)
            # print(opt_state)
            # print("aaaa")
            self.vx, self.vy, self.w = opt_state[0][1], opt_state[1][1], opt_state[2][1]

            print(np.round(self.vx, 2), np.round(self.vy, 2), np.round(self.w, 2))
            # print(opt_input[:2])
            # print(opt_input[2:4])
            # print(opt_input[4:6])
            # print(opt_input[6:8])
            vx_coords = np.array(opt_input[:4]) * 0.5
            vy_coords = np.array(opt_input[4:8]) * 0.5
            


            quiver.remove()
            quiver = ax.quiver(x_coords, y_coords, vx_coords, vy_coords, angles='xy', scale_units='xy', scale=1)
            plt.draw()
            plt.pause(0.1)

            prev_time = current_time

            time.sleep(0.1)
        plt.ioff()
        plt.show()
        


if __name__ == '__main__':
    mpc = MPC()
    mpc.main()

