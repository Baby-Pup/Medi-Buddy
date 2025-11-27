import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    # 환경 변수 need_compile을 불러오되, 없으면 기본값은 'False'
    compiled = os.environ.get('need_compile', 'False')

    # 런치 인자 설정 (기본값 지정)
    enable_save = LaunchConfiguration('enable_save', default='true').perform(context)
    slam_method = LaunchConfiguration('slam_method', default='slam_toolbox').perform(context)
    sim = LaunchConfiguration('sim', default='false').perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ.get('MASTER', '/')).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ.get('HOST', '/')).perform(context)

    # 런치 인자 선언 (외부에서 수정 가능하도록)
    enable_save_arg = DeclareLaunchArgument('enable_save', default_value=enable_save)
    slam_method_arg = DeclareLaunchArgument('slam_method', default_value=slam_method)
    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    # 프레임 이름 prefix 설정 (로봇 네임스페이스 기반)
    frame_prefix = '' if robot_name == '/' else f'{robot_name}/'
    use_sim_time = 'true' if sim == 'true' else 'false'

    # 좌표계 프레임 이름 지정
    map_frame = f'{frame_prefix}map'
    odom_frame = f'{frame_prefix}odom'
    base_frame = f'{frame_prefix}base_footprint'

    # slam 패키지 경로 설정 (빌드 버전 or 개발 소스)
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')       # 설치된 패키지 경로
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'           # 로컬 워크스페이스 경로

    # 로봇 구동 관련 launch 파일 포함 (robot.launch.py)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name
        }.items(),
    )

    # SLAM 관련 launch 파일 포함 (slam_base.launch.py)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/slam_base.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,                 # 시뮬레이션 시간 사용 여부
            'map_frame': map_frame,                       # map 프레임 이름
            'odom_frame': odom_frame,                     # odom 프레임 이름
            'base_frame': base_frame,                     # base 프레임 이름
            'scan_topic': f'{frame_prefix}scan_raw',      # 스캔 토픽 (예: /robot1/scan_raw)
            'enable_save': enable_save                    # 맵 저장 여부
        }.items(),
    )

    # SLAM 방식이 slam_toolbox일 경우 실행 그룹 구성
    if slam_method == 'slam_toolbox':
        bringup_launch = GroupAction(
            actions=[
                PushRosNamespace(robot_name),             # 네임스페이스 적용
                base_launch,                              # 로봇 관련 launch 먼저 실행
                TimerAction(                              # 10초 후 SLAM launch 실행
                    period=10.0,
                    actions=[slam_launch],
                ),
            ]
        )

    # 실행할 액션 리스트 반환
    return [sim_arg, master_name_arg, robot_name_arg, slam_method_arg, bringup_launch]

def generate_launch_description():
    # LaunchDescription 객체 생성 및 launch_setup 실행
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    # LaunchDescription 생성
    ld = generate_launch_description()

    # LaunchService 생성 (런치 실행 관리)
    ls = LaunchService()

    # LaunchDescription 등록
    ls.include_launch_description(ld)

    # Launch 실행 (로봇 → SLAM 순차 실행)
    ls.run()
