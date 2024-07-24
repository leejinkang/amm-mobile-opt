import rclpy
from rclpy.node import Node

import numpy as np
from pydrake.all import MathematicalProgram, Solve, Variable
from pydrake.solvers import MathematicalProgram, Solve

dt = 0.01
N = 5  # 예측 시간 범위 (시간 스텝 수)


class MPC:

    def __init__(self):
        pass
    


    def simulate_mpc(self, x0, target_path_x, target_path_y, target_path_yaw):
        # MPC 파라미터
        
        # 최적화 문제 설정
        prog = MathematicalProgram()
        
        # 변수 정의 (상태 및 입력)
        x = prog.NewContinuousVariables(N+1, 'x')
        y = prog.NewContinuousVariables(N+1, 'y')
        yaw = prog.NewContinuousVariables(N+1, 'yaw')
        vx = prog.NewContinuousVariables(N, 'vx')
        
        vy = prog.NewContinuousVariables(N, 'vy')
        omega = prog.NewContinuousVariables(N, 'omega')
        
        # 초기 상태 설정
        prog.AddBoundingBoxConstraint(x0[0], x0[0], x[0])
        prog.AddBoundingBoxConstraint(x0[1], x0[1], y[0])
        prog.AddBoundingBoxConstraint(x0[2], x0[2], yaw[0])

        #최대/최소 속도 제약 조건
        prog.AddBoundingBoxConstraint(-MAX_CONST_SPEED_X, MAX_CONST_SPEED_X, vx[0])
        prog.AddBoundingBoxConstraint(-MAX_CONST_SPEED_Y, MAX_CONST_SPEED_Y, vy[0])
        prog.AddBoundingBoxConstraint(-MAX_CONST_SPEED_W, MAX_CONST_SPEED_W, omega[0])

        # # 동역학 제약 조건
        for i in range(N):
            # prog.AddConstraint(x[i+1] == x[i] + dt * (vx[i] * np.cos(yaw[i]) - vy[i] * np.sin(yaw[i])))
            # prog.AddConstraint(y[i+1] == y[i] + dt * (vx[i] * np.sin(yaw[i]) + vy[i] * np.cos(yaw[i])))
            prog.AddConstraint(x[i+1] == x[i] + dt * (vx[i] * np.cos(yaw[i]) - vy[i] * np.sin(yaw[i])))
            prog.AddConstraint(y[i+1] == y[i] + dt * (vx[i] * np.sin(yaw[i]) + vy[i] * np.cos(yaw[i])))
            prog.AddConstraint(yaw[i+1] == yaw[i] + dt * omega[i])


        # 목표 함수 (경로 오차 최소화)
        for i in range(N+1):
            prog.AddCost(Q_distance * (x[i] - target_path_x[i])**2)
            prog.AddCost(Q_distance * (y[i] - target_path_y[i])**2)
            prog.AddCost(Q_angle * (yaw[i] - target_path_yaw[i])**2)

        #입력 변화 최소화 (옵션)
        for i in range(N-1):
            prog.AddCost(100*(vx[i+1] - vx[i])**2)
            prog.AddCost(100*(vy[i+1] - vy[i])**2)
            prog.AddCost(100*(omega[i+1] - omega[i])**2)

        
        # 최적화 문제 해결
        result = Solve(prog)
        
        # 결과 확인
        
        if result.is_success():
            optimized_x = result.GetSolution(x)
            optimized_y = result.GetSolution(y)
            optimized_yaw = result.GetSolution(yaw)
            optimized_vx = result.GetSolution(vx)
            optimized_vy = result.GetSolution(vy)
            optimized_omega = result.GetSolution(omega)

            return optimized_x, optimized_y, optimized_yaw, optimized_vx, optimized_vy, optimized_omega

        else:
            pass

    def iteration(self):
        pass

