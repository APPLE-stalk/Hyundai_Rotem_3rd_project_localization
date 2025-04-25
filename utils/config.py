SHARED = {
    'pre_playerPos': {
        'x':0,
        # 'y':0,
        'z':0
    },
    'cur_playerPos': {
        'x':0,
        # 'y':0,
        'z':0
    },
    'cur_playerPos_noise': {
        'x':0,
        # 'y':0,
        'z':0
    },
    'del_playerPos':{
        'x': [],
        'z': []
    },
    # 'del_playerPos_x': [],
    # 'del_playerPos_z': [],
    
    # 차체 속도
    "tank_cur_vel_ms": 0.0,
    
    "pre_tank_vel_kh": 0.0,
    "cur_tank_vel_kh": 0.0,
    "tar_tank_vel_kh": 0.0,

    
    "vel_data": [],
    
    
    # 차체 yaw 각도
    # 'pre_pre_tank_yaw_deg': 0.0, # -pi, +pi 예외처리용 변수
    "pre_acc_tank_yaw_deg":0.0, # 누적 각도로 -pi, pi 예외처리
    "cur_acc_tank_yaw_deg": 0.0, # 누적 각도로 -pi, pi 예외처리
    "deg_acc_cnt":0.0, # 몇 바퀴 누적되었는지 카운트
    
    "pre_tank_yaw_deg": 0.0,
    "cur_tank_yaw_deg": 0.0,
    "cur_tank_yaw_deg_noise": 0.0,
    "tar_tank_yaw_deg": 0.0,
    
    "log_cur_tank_yaw_deg": [],
    "log_cur_tank_yaw_deg_noise": [], 
    "log_cur_est_playerPos_yaw_deg": [],
    
    # "cur_tank_yaw_rad": 0.0,
    
    
    
    # ========================================
    
    # EKF로 추정한 이전 전차 자세 정보
    'pre_est_playerPos':{
        'x': 0.0,
        'z': 0.0,
        'yaw_deg': 0.0,
        'pre_yaw_deg': 0.0
    },
    # EKF로 추정한 전차 자세정보
    'cur_est_playerPos':{
        'x': 0.0,
        'z': 0.0,
        'yaw_deg': 0.0
    },
    # EKF로 추정한 전차 속도
    # "est_cur_tank_vel_kh":0.0,
    
    'ekf_var':{
        'Q_yaw': 0.2,
        'R_yaw': 2.0
    }

    
    
}