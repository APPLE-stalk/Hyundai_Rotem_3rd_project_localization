# EKF + IMU(roll, pitch, yaw) + GNSS(x, y, z) 기반 Localization

# 2D 기준
from utils.config import SHARED
shared = SHARED

import numpy as np

class EKF_kd:
    def __init__(self, dt, state_dim=3, meas_dim=2):
        self.dt = dt

        # 상태 백터 정의 [x, y, yaw]
        self.x = np.zeros((state_dim, 1))
        
        # 상태 공분산
        self.P = np.eye(state_dim)
        # 초기에 eye로 설정, 추후 업데이트 하며 알아서 조정됨
        # [[1. 0. 0.] 
        # [0. 1. 0.] 
        # [0. 0. 1.]]
        
        # 프로세스 노이즈(x, y, yaw), 모델링되지 않은 외란이나 제어 입력의 불확실성등 시스템의 동작에 포함된 "내부 노이즈"을 의미함
        # 시간이 지나면서 "state"가 얼마나 "더 불확실해지는지"를 반영
        # 따라서 이 값이 클수록 EKF는 **센서의 측정치(Observation)**에 더 의존하게 됩니다.
        # 반복되며 학습되지는 않는 고정된 값
        self.Q = np.diag([5.0, 5.0, shared['ekf_var']['Q_yaw']])**2
        # [[25.          0.          0.        ]
        # [ 0.         25.          0.        ]
        # [ 0.          0.          0.03046174]]
        
        
        # 관측 노이즈 (GPS_x, GPS_y, IMU yaw각)
        self.R = np.diag([5.0, 5.0, shared['ekf_var']['R_yaw']])**2
        
    def predict(self, x, z, yaw_rad, v):
        self.Q = np.diag([5.0, 5.0, shared['ekf_var']['Q_yaw']])**2
        self.R = np.diag([5.0, 5.0, shared['ekf_var']['R_yaw']])**2
        '''
        v: 전차 속도(입력으로 들어옴, km/h)
        '''
        # x, y, yaw = self.x.flatten()
        dt = self.dt
        
        v = v/3.6
        
        # motion model
        fx = np.array([
            [x + v * np.cos(yaw_rad) * dt],
            [z + v * np.sin(yaw_rad) * dt],
            [yaw_rad]
        ])
        
        # Jacobian of motion model
        # 상태 전이 자코비안
        F = np.array([
            [1.0, 0.0, -v * np.sin(yaw_rad) * dt],
            [0.0, 1.0,  v * np.cos(yaw_rad) * dt],
            [0.0, 0.0, 1.0]
        ])
        
        self.x = fx # motion model 기반 상태 예측
        # self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi  # ← yaw 정규화 # 180 부근 정규화
        self.P = F @ self.P @ F.T + self.Q # 오차 공분산 예측


    def update(self, z):
        '''
        z: 관측값 (GPS로 측정한 x, z)값
        '''
        # 관측모델 자코비안
        H = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]  # yaw 측정값도 직접 사용
    ])
        
        z = z.reshape((3, 1))
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        # self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi  # ← update 후에도 적용 180도 부근 정규화
        self.P = (np.eye(3) - K @ H) @ self.P

    def get_state(self):
            return self.x.flatten()




# ==========================
# # 제어 입력
# u = [v, yaw_rate]

# # 동역학 모델
# def motion_model(x, u, dt):
#     x_pos, y_pos, yaw = x
#     v, yaw_rate = u

#     if abs(yaw_rate) < 1e-5:  # 직선 주행
#         x_pos += v * np.cos(yaw) * dt
#         y_pos += v * np.sin(yaw) * dt
#     else:  # 회전 포함
#         x_pos += (v / yaw_rate) * (np.sin(yaw + yaw_rate*dt) - np.sin(yaw))
#         y_pos += (v / yaw_rate) * (-np.cos(yaw + yaw_rate*dt) + np.cos(yaw))
#         yaw += yaw_rate * dt

#     return np.array([x_pos, y_pos, yaw])


# # EKF 구성 예시
# from filterpy.kalman import ExtendedKalmanFilter
# import numpy as np

# ekf = ExtendedKalmanFilter(dim_x=3, dim_z=2)
# # 상태 3차원 (x, y, yaw)
# # 측정 2차원 (v, yaw_rate)
# ekf.x = np.array([59.35, 27.23, 0.])  # 초기 위치와 yaw

# ekf.P *= 1.0      # 초기 상태 공분산
# ekf.R = np.diag([0.5, 0.5])  # GPS 위치 오차
# ekf.Q = np.eye(3) * 0.01     # 시스템 잡음

# def H_jacobian(x):
#     return np.array([[1, 0, 0], [0, 1, 0]])  # GPS는 x, y만 측정

# for step in range(len(data)):
#     # 입력값 추출
#     v = data[step]['velocity']
#     yaw_now = data[step]['yaw']
#     yaw_prev = data[step-1]['yaw']
#     dt = data[step]['dt']
#     yaw_rate = (yaw_now - yaw_prev) / dt

#     u = [v, yaw_rate]

#     # 예측
#     ekf.predict_update(z=data[step]['gps'][:2],
#                     hx=measurement_model,
#                     fx=lambda x, dt=dt: motion_model(x, u, dt),
#                     HJacobian=H_jacobian)