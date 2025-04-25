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
    "pre_tank_yaw_deg": 0.0,
    "cur_tank_yaw_deg": 0.0,
    "cur_tank_yaw_deg_noise": 0.0,
    "tar_tank_yaw_deg": 0.0,
    
    # "cur_tank_yaw_rad": 0.0,
    
    
    
    # ========================================
    
    # EKF로 추정한 이전 전차 자세 정보
    'pre_est_playerPos':{
        'x': 0,
        'z': 0,
        'yaw_deg': 0
    },
    # EKF로 추정한 전차 자세정보
    'cur_est_playerPos':{
        'x': 0,
        'z': 0,
        'yaw_deg': 0
    },
    # EKF로 추정한 전차 속도
    # "est_cur_tank_vel_kh":0.0,

    
    
}