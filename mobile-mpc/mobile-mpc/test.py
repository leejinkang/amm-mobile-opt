import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# 로봇의 기하학적 특성
L = 1.0  # 휠베이스 (미터)
W = 1.0  # 트랙 (미터)
r = 0.1  # 휠 반지름 (미터)
dt = 0.1  # 시간 간격 (초)

# 예측 시간 단계
N = 5

# 초기 상태와 목표 경로
x_init = np.array([0, 0, 0])
trajectory = np.array([np.linspace(0, 10, 100), np.linspace(0, 10, 100), np.zeros(100)])

# 비용 함수 가중치
Q = np.diag([1.0, 1.0, 1.0])  # 상태 오차 비용
R = np.diag([0.1] * 4 + [1.0] * 4)  # 입력 비용

# 상태 갱신 방정식
def dynamics(x, u):
    V_x = [u[4 + i] * r * np.cos(u[i]) for i in range(4)]
    V_y = [u[4 + i] * r * np.sin(u[i]) for i in range(4)]
    
    v_x = np.mean(V_x)
    v_y = np.mean(V_y)
    omega = (V_y[1] - V_y[3]) / L - (V_x[0] - V_x[2]) / W
    
    new_x = x[0] + (v_x * np.cos(x[2]) - v_y * np.sin(x[2])) * dt
    new_y = x[1] + (v_x * np.sin(x[2]) + v_y * np.cos(x[2])) * dt
    new_theta = x[2] + omega * dt
    
    return np.array([new_x, new_y, new_theta])

# 비용 함수 정의
def cost_function(U, *args):
    x_init, trajectory, N, Q, R = args
    cost = 0
    x = np.copy(x_init)
    
    for t in range(N):
        u = U[8*t:8*(t+1)]
        x_ref = trajectory[:, t]
        state_error = x - x_ref
        cost += state_error.T @ Q @ state_error + u.T @ R @ u
        x = dynamics(x, u)
    
    return cost

# 제약 조건 정의
def constraint_function(U, *args):
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

# 현재 상태에 가장 가까운 trajectory index 찾기
def find_closest_index(x_current, trajectory):
    distances = np.linalg.norm(trajectory[:2, :] - x_current[:2, None], axis=0)
    return np.argmin(distances)

# 초기 입력 설정
U0 = np.ones(8 * N) * 0.1

print("UO : ", U0)
# MPC 루프
x_current = x_init
x_history = [x_current]

for i in range(len(trajectory[0]) - N):
    closest_index = find_closest_index(x_current, trajectory)
    x_ref = trajectory[:, closest_index:closest_index+N+1]
    
    # 최적화 문제 설정
    args = (x_current, x_ref, N, Q, R)
    print("args : ", args)
    constraints = [{'type': 'eq', 'fun': constraint_function, 'args': (x_current, N)}]
    result = minimize(cost_function, U0, args=args, constraints=constraints, method='SLSQP', options={'disp': False, 'maxiter': 500})
    print("###################################")

    # 최적화 결과에서 입력 추출
    U_opt = result.x
    u_opt = U_opt[:8]
    # 상태 갱신
    x_current = dynamics(x_current, u_opt)
    x_history.append(x_current)

x_history = np.array(x_history)

# 결과 출력
plt.plot(trajectory[0], trajectory[1], 'r--', label='Reference Path')
plt.plot(x_history[:, 0], x_history[:, 1], 'b-', label='MPC Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()
