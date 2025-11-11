import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    # 환경 변수 need_compile 불러오기 (패키지 빌드 여부 확인용)
    compiled = os.environ['need_compile']

    # 런치 인자(sim, master_name, robot_name)를 context에서 가져오기
    sim = LaunchConfiguration('sim', default='false').perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)

    # 런치 인자 선언 (외부에서 인자를 전달받을 수 있게 함)
    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    # 프레임 및 토픽 이름 접두어 설정 (로봇 네임스페이스 기반)
    frame_prefix = '' if robot_name == '/' else '%s/' % robot_name
    topic_prefix = '' if robot_name == '/' else '/%s' % robot_name
    use_sim_time = 'true' if sim == 'true' else 'false'

    # 주요 프레임 이름 설정
    map_frame = 'map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)

    # 주요 토픽 이름 설정
    depth_camera_topic = '/ascamera/camera_publisher/depth0/image_raw'.format(topic_prefix)
    depth_camera_info = '/ascamera/camera_publisher/rgb0/camera_info'.format(topic_prefix)
    rgb_camera_topic = '/ascamera/camera_publisher/rgb0/image'.format(topic_prefix)
    odom_topic = '{}/odom'.format(topic_prefix)
    scan_topic = '{}/scan_raw'.format(topic_prefix)

    # slam 패키지 경로 설정 (빌드된 패키지인지 소스 실행인지 구분)
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')   # 설치된 패키지 경로
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'       # 로컬 워크스페이스 경로

    # 로봇 관련 launch 파일 포함 (robot.launch.py)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
        launch_arguments={
            'sim': sim,
            'master_name': master_name,
            'robot_name': robot_name,
            'action_name': 'horizontal',  # 기본 액션 이름 설정
        }.items(),
    )

    # RTAB-Map 관련 launch 파일 포함 (SLAM 기능 담당)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,  # 시뮬레이션 시간 동기화 여부
        }.items(),
    )

    # 전체 bringup 구성 (네임스페이스 적용 및 순차 실행)
    bringup_launch = GroupAction(
        actions=[
            PushRosNamespace(robot_name),  # 모든 노드를 로봇 네임스페이스 하위에 실행
            base_launch,                   # 로봇 관련 launch 먼저 실행
            TimerAction(                   # 10초 후 RTAB-Map launch 실행
                period=10.0,
                actions=[rtabmap_launch],
            ),
        ]
    )

    # 실행할 액션 목록 반환
    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]

def generate_launch_description():
    # LaunchDescription 객체 생성 및 launch_setup 함수 호출
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    # LaunchDescription 객체 생성
    ld = generate_launch_description()

    # LaunchService 객체 생성 (런치 실행 관리)
    ls = LaunchService()

    # LaunchDescription 등록
    ls.include_launch_description(ld)

    # Launch 실행 (자율주행 및 SLAM 프로세스 순차 실행)
    ls.run()
