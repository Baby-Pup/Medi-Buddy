import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    # 환경 변수 need_compile을 불러와서 빌드된 버전인지(패키지 설치) 확인
    compiled = os.environ['need_compile']

    # 런치 인자(sim, master_name, robot_name)를 context에서 가져옴 (기본값은 환경변수 사용)
    sim = LaunchConfiguration('sim', default='false').perform(context)
    master_name = LaunchConfiguration('master_name', default=os.environ['MASTER']).perform(context)
    robot_name = LaunchConfiguration('robot_name', default=os.environ['HOST']).perform(context)

    # 런치 인자들을 런치 시스템에 등록 (외부에서 인자 전달 가능하도록)
    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    # 네임스페이스, 프레임, 토픽 이름 등에 접두어(prefix) 설정
    frame_prefix = '' if robot_name == '/' else '%s/' % robot_name
    topic_prefix = '' if robot_name == '/' else '/%s' % robot_name
    use_sim_time = 'true' if sim == 'true' else 'false'

    # 좌표계 프레임 이름 정의 (map, odom, base 등)
    map_frame = 'map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)

    # 카메라 및 오도메트리 관련 토픽 이름 정의
    depth_camera_topic = '/ascamera/camera_publisher/depth0/image_raw'.format(topic_prefix)
    depth_camera_info = '/ascamera/camera_publisher/rgb0/camera_info'.format(topic_prefix)
    rgb_camera_topic = '/ascamera/camera_publisher/rgb0/image'.format(topic_prefix)
    odom_topic = '{}/odom'.format(topic_prefix)
    # scan_topic = '{}/scan_raw'.format(topic_prefix)  # (필요 시 사용 가능)

    # 컴파일된 패키지를 쓸지, 개발 중인 소스 경로를 쓸지 선택
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')  # 설치된 패키지 경로 찾기
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'       # 개발 중인 로컬 경로 사용

    # 기본 로봇 관련 launch 파일 포함 (robot.launch.py)
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

    # RTAB-Map 관련 launch 파일 포함 (SLAM)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,  # 시뮬레이션 시간 사용 여부 전달
        }.items(),
    )

    # 전체 bringup 구성 (네임스페이스 적용 + 순차 실행)
    bringup_launch = GroupAction(
        actions=[
            PushRosNamespace(robot_name),  # 모든 노드를 로봇 네임스페이스 하위에 배치
            base_launch,                   # 먼저 base_launch 실행
            TimerAction(                   # 10초 후에 RTAB-Map 실행 (순차적 실행을 위해)
                period=10.0,
                actions=[rtabmap_launch],
            ),
        ]
    )

    # launch 시스템이 실행할 액션 리스트 반환
    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]

def generate_launch_description():
    # LaunchDescription 객체 생성 및 launch_setup 실행
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    # 단독 실행 시 LaunchService를 이용해 런치 실행
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
