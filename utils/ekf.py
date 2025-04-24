# EKF + IMU(roll, pitch, yaw) + GNSS(x, y, z) 기반 Localization

# 2D 기준



import numpy as np

class EKF_kd:
    def __init__(self, dt, state_dim=3, meas_dim=2):
        self.dt = dt

        # 상태 백터 정의 [x, y, yaw]
        self.x = np.zeros((state_dim, 1))
        
        # 상태 공분산
        self.P = np.eye(state_dim)
        
        # 프로세스 노이즈(x, y, yaw)
        self.Q = np.diag([0.2, 0.2, np.deg2rad(2.0)])**2
        
        # 관측 노이즈 (GPS_x, GPS_y, IMU yaw각)
        self.R = np.diag([1.0, 1.0, 10])**2
        
    def predict(self, x, z, yaw, v):
        '''
        v: 전차 속도(입력으로 들어옴, km/h)
        '''
        # x, y, yaw = self.x.flatten()
        dt = self.dt
        
        # motion model
        fx = np.array([
            [x + v * np.cos(yaw) * dt],
            [z + v * np.sin(yaw) * dt],
            [yaw]
        ])
        
        # Jacobian of motion model
        F = np.array([
            [1.0, 0.0, -v * np.sin(yaw) * dt],
            [0.0, 1.0,  v * np.cos(yaw) * dt],
            [0.0, 0.0, 1.0]
        ])
        
        self.x = fx
        self.P = F @ self.P @ F.T + self.Q


    def update(self, z):
        '''
        z: 관측값 (GPS로 측정한 x, z)값
        '''
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