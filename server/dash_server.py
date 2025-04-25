from dash import Dash, dcc, html
from dash.dependencies import Output, Input, State
import plotly.graph_objs as go
import os
import numpy as np
import open3d as o3d


from utils.config import SHARED
shared = SHARED

def load_voxel_data(path='C:/pycode/3rd_Project/mapping/3d_LiDAR_fake/open3d/downsampling_outliner_point_cloud_0.5.ply', voxel_size=0.5):
    if not os.path.exists(path):
        return np.empty((0, 3))
    
    # 포인트 클라우드 불러오기
    pcd = o3d.io.read_point_cloud(path)
    
    # 포인트 클라우드를 Voxel Grid로 변환
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)
    
    # Voxel 중심 좌표 추출
    voxels = voxel_grid.get_voxels()
    voxel_centers = np.array([voxel.grid_index for voxel in voxels], dtype=np.float32)
    
    # Voxel 좌표를 원래 공간으로 변환
    voxel_centers *= voxel_size
    voxel_centers += np.array(voxel_grid.origin)

    return voxel_centers

def create_3d_scatter(voxel_centers):
    colors = voxel_centers[:, 2]  # Z값 기준으로 컬러맵 생성
    fig = go.Figure(data=[
        go.Scatter3d(
            x=voxel_centers[:, 0],
            y=voxel_centers[:, 1],
            z=voxel_centers[:, 2],
            mode='markers',
            marker=dict(
                size=1,
                color=colors,  # 색상 지정 'Cividis', 'Jet', 'Plasma'
                colorscale='Viridis',
                colorbar=dict(title='Height'),
                opacity=1 # 점 투명도 1: 불투명, 0: 투명
            )
        ),
        go.Scatter3d(
            x=[shared['cur_playerPos']['x']],
            y=[shared['cur_playerPos']['z']],  # z축 위치는 z, y 전환 여부에 따라 수정 가능
            z=[9.5],  
            mode='markers',
            marker=dict(size=5, color='red'),
            opacity=1
        ),
        go.Scatter3d( # 칼만필터로 추정한 정보
            x=[shared['cur_est_playerPos']['x']],
            y=[shared['cur_est_playerPos']['z']],  
            z=[9.5],  
            mode='markers',
            marker=dict(size=5, color='blue'),
            opacity=1
        ),
        go.Scatter3d( # 칼만필터에 들어가는 센서값 + 노이즈 정보
            x=[shared['cur_playerPos_noise']['x']], 
            y=[shared['cur_playerPos_noise']['z']],  
            z=[9.5],  
            mode='markers',
            marker=dict(size=5, color='green'),
            opacity=1
        )
    ])
    fig.update_layout(scene=dict(
        xaxis_title='X', yaxis_title='Y', zaxis_title='Z', aspectmode='data'),  # 👈 각 축 스케일 비율을 실제 데이터대로 유지),
        margin=dict(l=0, r=0, t=0, b=0)
    )
    return fig

def create_yaw_figure() -> go.Figure:
    """Yaw 로그(진값·노이즈·EKF)를 선 그래프로 출력."""
    true_log  = shared.get("log_cur_tank_yaw_deg", [])[-100:]
    noise_log = shared.get("log_cur_tank_yaw_deg_noise", [])[-100:]
    est_log   = shared.get("log_cur_est_playerPos_yaw_deg", [])[-100:]
    t_len = max(len(true_log), len(noise_log), len(est_log))
    t = list(range(t_len))
    
    fig = go.Figure()
    if true_log:
        fig.add_scatter(x=t[:len(true_log)], y=true_log, mode="lines", name="True", line=dict(width=2))
    if noise_log:
        fig.add_scatter(x=t[:len(noise_log)], y=noise_log, mode="lines", name="Measurement", line=dict(width=1, dash="dot"))
    if est_log:
        fig.add_scatter(x=t[:len(est_log)], y=est_log, mode="lines", name="EKF", line=dict(width=2))
    fig.update_layout(xaxis_title="Step", yaxis_title="Yaw [deg]", margin=dict(l=40, r=20, t=20, b=40))
    return fig


def create_dash_app():
    app = Dash(__name__,assets_folder="assets",        # 기본값 "assets"
            assets_url_path="/hide_slider_ticks.css" )     # (선택) URL 경로)
    
    app.layout = html.Div([
        html.H4("Voxel 3D Map (Point Cloud → Voxel)"),
        dcc.Graph(id='voxel-graph'),
        
        html.Div([
            html.Label("자동 최신화:"),
            dcc.Checklist(
                options=[{'label': '활성화', 'value': 'on'}],
                value=['on'],
                id='autorefresh-toggle',
                inline=True
            ),
            html.Button('수동 새로고침', id='manual-refresh', n_clicks=0),
        ], style={'margin-top': '10px'}),
        
        html.Div(id='pose-info', style={'margin-top': '20px', 'fontSize': 18}),
        
        dcc.Graph(id="yaw-graph"),
        
        # ── Q/R 슬라이더 영역 ──
        html.Div([
            html.Div([
                html.Label("프로세스 공분산 Q_yaw"),
                dcc.Slider(id="slider-q-yaw", min=0.01, max=10.0, step=0.01, value=shared['ekf_var']['Q_yaw'],
                        tooltip={"always_visible": True}),
            ], style={"margin-bottom": "15px"}),
            html.Div([
                html.Label("측정 공분산 R_yaw"),
                dcc.Slider(id="slider-r-yaw", min=0.01, max=10.0, step=0.01, value=shared['ekf_var']['R_yaw'],
                        tooltip={"always_visible": True}),
            ]),
            html.Div(id="slider-values", style={"margin-top": "10px", "fontSize": 16, "fontWeight": "bold"}),
        ], style={"padding": "0 20px 20px 20px"}),
        
        dcc.Interval(id='interval', interval=400, n_intervals=0, disabled=False)
    ])

    @app.callback(
        Output('interval', 'disabled'),
        Input('autorefresh-toggle', 'value')
    )
    def toggle_autorefresh(value):
        return 'on' not in value

    @app.callback(
        Output('voxel-graph', 'figure'),
        [Input('interval', 'n_intervals'),
        Input('manual-refresh', 'n_clicks')],
        [State('autorefresh-toggle', 'value')],
        prevent_initial_call=True
    )
    def update_graph(n_intervals, n_clicks, toggle_value):
        voxel_centers = load_voxel_data()
        return create_3d_scatter(voxel_centers)
    
    @app.callback(
        Output('pose-info', 'children'),
        Input('interval', 'n_intervals')
    )
    def update_pose_info(n):
        return "전차 추정 위치: x = {0:.2f}, z = {1:.2f}, yaw = {2:.2f}°".format(shared['cur_est_playerPos']['x'], shared['cur_est_playerPos']['z'], shared['cur_est_playerPos']['yaw_deg'])
    
    # Yaw 그래프 업데이트
    @app.callback(Output("yaw-graph", "figure"), Input("interval", "n_intervals"))
    def update_yaw(_):
        return create_yaw_figure()
    
    # 슬라이더: 공유변수 업데이트 + 값 표시 (그래프는 건드리지 않음)
    @app.callback(Output('slider-values', 'children'), [Input('slider-q-yaw', 'value'), Input('slider-r-yaw', 'value')])
    def update_qr_display(q_val, r_val):
        shared['ekf_var']['Q_yaw'] = float(q_val)
        shared['ekf_var']['R_yaw'] = float(r_val)
        return f'Q_yaw = {q_val:.2f}     R_yaw = {r_val:.2f}'


    return app