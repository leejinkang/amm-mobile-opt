import numpy as np
import casadi as ca

# 로봇의 기하학적 특성
L = 1.0  # 휠베이스 (미터)
W = 1.0  # 트랙 (미터)
r = 0.1  # 휠 반지름 (미터)
dt = 0.1  # 시간 간격

# 상태 벡터와 입력 정의
N = 10  # 예측 시간 단계
x = ca.MX.sym('x', 3, N + 1)  # [x, y, theta]
u = ca.MX.sym('u', 8, N)  # [steering angles (4), wheel speeds (4)]

# 초기 상태와 목표 경로
x_init = np.array([0, 0, 0])
trajectory = np.array([np.linspace(0, 10, N + 1), np.linspace(0, 10, N + 1), np.zeros(N + 1)])

# 비용 함수 가중치
Q = np.diag([1.0, 1.0, 1.0])  # 상태 오차 비용
R = np.diag([0.1] * 4 + [1.0] * 4)  # 입력 비용 (휠의 회전 속도 비용 높임)

# 슬립 각 계산 함수
def slip_angle(theta, V_x, V_y):
    beta = ca.atan2(V_y, V_x)
    return theta - beta

# 상태 갱신 방정식
def dynamics(x, u, t):
    V_x = [u[4 + i, t] * r * ca.cos(u[i, t]) for i in range(4)]
    V_y = [u[4 + i, t] * r * ca.sin(u[i, t]) for i in range(4)]

    new_x = x[0, t] + dt * (sum(V_x) / 4)
    new_y = x[1, t] + dt * (sum(V_y) / 4)
    new_theta = x[2, t] + dt * ((V_y[1] - V_y[0] + V_y[3] - V_y[2]) / (2 * L) + (V_x[1] - V_x[0] + V_x[3] - V_x[2]) / (2 * W))

    return ca.vertcat(new_x, new_y, new_theta)

# 비용 함수 및 제약 조건 정의
cost = 0
g = []  # 제약 조건 리스트

for t in range(N):
    state_error = x[:, t] - trajectory[:, t]
    cost += ca.mtimes([state_error.T, Q, state_error])  # 경로 오차 비용
    cost += ca.mtimes([u[:, t].T, R, u[:, t]])  # 입력 비용

    V_x = [u[4 + i, t] * r * ca.cos(u[i, t]) for i in range(4)]
    V_y = [u[4 + i, t] * r * ca.sin(u[i, t]) for i in range(4)]
    slip_angles = [slip_angle(u[i, t], V_x[i], V_y[i]) for i in range(4)]
    
    for slip in slip_angles:
        cost += ca.fabs(slip)  # 슬립 각 최소화

    x_next = dynamics(x, u, t)
    g.append(x[:, t + 1] - x_next)

# 초기 상태 제약 조건 추가
g.append(x[:, 0] - x_init)

# 최적화 문제 정의
g = ca.vertcat(*g)
nlp = {'x': ca.vertcat(ca.reshape(x, -1, 1), ca.reshape(u, -1, 1)), 'f': cost, 'g': g}
opts = {'ipopt.print_level': 0, 'print_time': 0}
solver = ca.nlpsol('solver', 'ipopt', nlp, opts)

# 초기 추정값
x0 = np.zeros((3, N + 1))
u0 = np.zeros((8, N))
x0[:, 0] = x_init
w0 = np.concatenate([x0.flatten(), u0.flatten()])

# 제약 조건 경계
lbg = np.zeros(g.shape)
ubg = np.zeros(g.shape)

# 최적화 문제 해결
sol = solver(x0=w0, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()

# 결과 추출
x_opt = w_opt[:3 * (N + 1)].reshape((3, N + 1))
u_opt = w_opt[3 * (N + 1):].reshape((8, N))

# 결과 출력
print("Optimal state trajectory:")
print(x_opt)
print("Optimal input trajectory:")
print(u_opt)
