# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='cannyedge_test1',
#             executable='canny_edge_detector',
#             name='canny_edge_detector',
#             output='screen',
#             remappings=[
#                 ('/image_raw', '/camera/color/image_raw')
#             ]
#         )
#     ])


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node', 
            name='camera',
            output='screen',
            parameters=[
                # 'filters': 적용할 후처리 필터 순서 (쉼표 없이 필터 키워드 나열)
                #   decimation: 해상도 하위 샘플링으로 노이즈 제거 및 성능 향상
                #   spatial: 엣지 보존 필터로 픽셀 노이즈 감소
                #   temporal: 프레임 간 EMA 스무딩으로 일시적 노이즈 완화
                #   hole_filling: 비측정 영역(홀)을 주변 값으로 보정
                { 'filters': 'decimation,spatial,temporal,hole_filling' },

                # Decimation Filter 옵션
                # 'decimation.filter_magnitude': 하위 샘플링 비율 (2면 가로/세로 1/2 해상도)
                { 'decimation.filter_magnitude': 2 },

                # Spatial Filter 옵션
                # 'spatial.filter_magnitude': 필터 반복 횟수 (기본 2)
                # 'spatial.filter_smooth_alpha': 스무딩 강도 (0~1, 클수록 부드러움)
                # 'spatial.filter_smooth_delta': 엣지 보존 임계치 (disparity 단계)
                # 'spatial.filter_holes_fill': 인접 픽셀로 홀 보정 반경 (0=없음, N 픽셀)
                { 'spatial.filter_magnitude': 2 },
                { 'spatial.filter_smooth_alpha': 0.7 },
                { 'spatial.filter_smooth_delta': 6 },
                { 'spatial.filter_holes_fill': 10 },

                # Temporal Filter 옵션
                # 'temporal.filter_smooth_alpha': 시간 EMA 스무딩 계수 (0~1)
                # 'temporal.filter_smooth_delta': 시간 필터 엣지 임계치
                # 'temporal.filter_holes_fill': 과거 프레임 홀 보존 수
                { 'temporal.filter_smooth_alpha': 0.5 },
                { 'temporal.filter_smooth_delta': 20 },
                { 'temporal.filter_holes_fill': 10 },

                # Hole Filling Filter 옵션
                # 'hole_filling.filter_holes_fill': 홀 보정 시 인접 픽셀 범위 (픽셀 단위)
                { 'hole_filling.filter_holes_fill': 10 },

                # Rectify 활성화
                { 'rectify': False },
                # 해상도 변경 (Crop 없이 해상도만 줄이기)
                # { 'enable_depth_crop': True },
                # { 'depth_width': 400 },
                # { 'depth_height': 400 },


                # --- ROI(Region of Interest) 설정 추가 ---
                # { 'roi_x1': 170 },
                # { 'roi_y1': 0 },
                # { 'roi_x2': 848 },
                # { 'roi_y2': 480 },

                # Clip Distance: 지정 거리(m) 초과 픽셀 제거 Nan으로
                { 'clip_distance': 1.0 },
            ],
        )



    ])



