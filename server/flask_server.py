from flask import Flask, request, jsonify
from utils.config import SHARED
from utils.ekf import EKF_kd
import numpy as np
# flask 앱
app = Flask(__name__)

shared = SHARED

#Localization
ekf = EKF_kd(dt = 0.1)
move_command = []

def cw_deg_to_rad(deg: float | np.ndarray) -> float | np.ndarray:
    """
    시계방향 deg (0°=전방) → 수학 좌표계 rad (CCW+, 범위 [-π, π))
    """
    rad = np.deg2rad(90.0 - deg)
    return (rad + np.pi) % (2 * np.pi) - np.pi   # 항상 [-π, π)

def rad_to_cw_deg(rad: float | np.ndarray) -> float | np.ndarray:
    """
    수학 좌표계 rad → 시계방향 deg (0°=전방, 범위 [0, 360))
    """
    # ① 범위 정규화
    rad = (rad + np.pi) % (2 * np.pi) - np.pi     # [-π, π)

    # ② 각도 변환
    deg = 90.0 - np.rad2deg(rad)                  # 0° 기준 회전
    return deg % 360.0                            # [0, 360)

@app.route('/info', methods=['POST'])
def info():
    data = request.get_json(force=True)
    
    # print("📨 /info data received:", data['time'])
    
    # 이전 정보 저장
    shared['pre_playerPos']['x'] = shared['cur_playerPos']['x']
    shared['pre_playerPos']['z'] = shared['cur_playerPos']['z']
    shared['pre_tank_yaw_deg'] = shared['cur_tank_yaw_deg']
    shared['pre_tank_vel_kh'] = shared['cur_tank_vel_kh']
    
    # 이전 추정값 저장
    shared['pre_est_playerPos']['x'] = shared['cur_est_playerPos']['x']
    shared['pre_est_playerPos']['z'] = shared['cur_est_playerPos']['z']
    shared['pre_est_playerPos']['yaw_deg'] = shared['cur_est_playerPos']['yaw_deg']
    
    
    # 최신 정보 업데이트
    shared['cur_playerPos']['x'] = data['playerPos']['x']
    shared['cur_playerPos']['z'] = data['playerPos']['z']
    
    shared['cur_tank_yaw_deg'] = data['playerBodyX']
    # shared['cur_tank_yaw_rad'] = cw_deg_to_rad(data['playerBodyX'])
    
    # 노이즈 넣은 최신 정보
    shared['cur_playerPos_noise']['x'] = shared['cur_playerPos']['x'] + np.random.normal(0, 5) # 현실성 주기 위해 랜덤 노이즈 표준편차 *m급으로 넣어줌
    shared['cur_playerPos_noise']['z'] = shared['cur_playerPos']['z'] + np.random.normal(0, 5)
    shared['cur_tank_yaw_deg_noise'] = (data['playerBodyX'] + np.random.normal(0, 10))%360  # 현실성 주기 위해 랜덤 노이즈 10도 급으로 넣어줌
    
    # ====================================================================================  연산/알고리즘/추정 시작
    # 위치 delta 구하기, [현재 위치 - 이전 위치]
    del_playerPos_x = shared['cur_playerPos']['x'] - shared['pre_playerPos']['x']
    del_playerPos_z = shared['cur_playerPos']['z'] - shared['pre_playerPos']['z']
    
    
    # 전후진 구분 알고리즘 개발 용 시각화 리스트, 전후진 제어 알고리즘 동작에 필요 없음
    shared['del_playerPos']['x'].append(del_playerPos_x)
    shared['del_playerPos']['z'].append(del_playerPos_z)
    
    
    # 이동 벡터
    v_move = np.array([del_playerPos_x, del_playerPos_z]) 
    
    # 월드 좌표계 기준의 전차의 yaw(deg -> 라디안)의 벡터화
    v_forward = np.array([np.cos(np.deg2rad(90 - shared['cur_tank_yaw_deg'])), np.sin(np.deg2rad(90 - shared['cur_tank_yaw_deg']))])  
    
    # 방향 판단
    moving_direction = np.sign(np.dot(v_forward, v_move)) # 두 벡터 내적 이용, +1: 전진, -1: 후진
    
    
    # 차체 전후진 속도 제어용
    shared['tank_cur_vel_ms'] = data['playerSpeed']*moving_direction # 전후진 고려
    shared['cur_tank_vel_kh'] = data['playerSpeed']*3.6*moving_direction # 전후진 고려
    shared['vel_data'].append(shared['cur_tank_vel_kh']) # dash 시각화 용
    
    
    
    # EKF Localization
    # prediction <- [이전x 예측값, 이전z 예측값, 이전 yaw 예측값, 이전 속도 입력값]
    ekf.predict(shared['pre_est_playerPos']['x'], shared['pre_est_playerPos']['z'], cw_deg_to_rad(shared['pre_est_playerPos']['yaw_deg']), shared['pre_tank_vel_kh']) 
    # update <- GPS 센서 값 [현재x, 현재z]d
    # print('raddddddddd: ', cw_deg_to_rad(shared['cur_tank_yaw_deg_noise']))
    
    ud =  np.array([shared['cur_playerPos_noise']['x'],shared['cur_playerPos_noise']['z'], cw_deg_to_rad(shared['cur_tank_yaw_deg_noise'])])
    
    ekf.update(ud)
    udt_x, udt_z, udt_yaw_rad = ekf.get_state()
    shared['cur_est_playerPos']['x'] = udt_x
    shared['cur_est_playerPos']['z'] = udt_z
    shared['cur_est_playerPos']['yaw_deg'] = rad_to_cw_deg(udt_yaw_rad)
    print('ssssssssss: ', shared['cur_est_playerPos']['yaw_deg'])
    
    # shared['est_playerPos']['x'] = 11
    # shared['est_playerPos']['z'] = 22
    # shared['est_playerPos']['yaw_deg'] = 33
    print('hi')
    
    
    
    if not data:
        return jsonify({"error": "No JSON received"}), 400
    
    # Auto-pause live-graphafter 15 seconds
    #if data.get("time", 0) > 15:
    #    return jsonify({"status": "success", "control": "pause"})
    # Auto-reset after 15 seconds
    #if data.get("time", 0) > 15:
    #    return jsonify({"stsaatus": "success", "control": "reset"})
    return jsonify({"status": "success", "control": ""})


@app.route('/get_move', methods=['GET'])
def get_move():
    
    
    # global move_command
    # if move_command:
    #     command = move_command.pop(0)
    #     print(f"🚗 Move Command: {command}")
    #     return jsonify(command)
    # else:
    #     return jsonify({"move": "STOP", "weight": 1.0})
    
    
    # move_command.clear()
    
    # if vel_control > 0:
    #     # return {"move": "W", "weight": vel_control}
    #     move_command.append({'move':'W', 'weight': vel_control})
    
    # else:
    #     # return {"move": "S", "weight": (-1)*vel_control}
    #     move_command.append({'move':'S', 'weight': (-1)*vel_control})
    
    # if steer_control > 0:
    #     # return {"move": "D", "weight": steer_control}
    #     move_command.append({'move':'D', 'weight': steer_control})
    
    # else:
    #     # return {"move": "A", "weight": (-1)*steer_control}
    #     move_command.append({'move':'A', 'weight': (-1)*steer_control})
    
    
    # print(move_command)
    return jsonify(move_command)
    # return jsonify({'move':'STOP'})

@app.route('/update_position', methods=['POST'])
def update_position():
    # data = request.get_json()
    # if not data or "position" not in data:
    #     return jsonify({"status": "ERROR", "message": "Missing position data"}), 400

    # try:
    #     x, y, z = map(float, data["position"].split(","))
    #     current_position = (int(x), int(z))
    #     # print(f"📍 Position updated: {current_position}")
    #     return jsonify({"status": "OK", "current_position": current_position})
    # except Exception as e:
    #     return jsonify({"status": "ERROR", "message": str(e)}), 400
    return jsonify({"hh":'hi'})





@app.route('/get_action', methods=['GET'])
def get_action():
    return jsonify({"hh":'hi'})
    
@app.route('/start', methods=['GET'])
def start():
    return jsonify({"hh":'hi'})
