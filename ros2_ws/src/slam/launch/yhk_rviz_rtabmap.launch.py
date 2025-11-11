import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription, ExecuteProcess

def launch_setup(context):
    # 환경 변수 need_compile을 불러와서, 빌드된 버전인지(설치된 패키지) 혹은 소스 실행 중인지 구분
    compiled = os.environ['need_compile']

    # need_compile 값에 따라 slam 패키지 경로를 다르게 설정
    if compiled == 'True':
        slam_package_path = get_package_share_directory('slam')        # 설치된 패키지의 공유 디렉토리 경로
    else:
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'            # 개발 중인 로컬 워크스페이스 경로

    # RViz2 실행 프로세스 정의
    rviz_node = ExecuteProcess(
            cmd=[
                'rviz2',                                               # RViz2 프로그램 실행
                'rviz2',                                               # (중복 인자: 실행 명령어로 두 번 들어가 있음)
                '-d',                                                  # 특정 설정 파일(.rviz)을 지정하는 옵션
                os.path.join(slam_package_path, 'rviz/rtabmap.rviz')   # slam 패키지 내의 rtabmap.rviz 파일 경로 지정
            ],
            output='screen'                                            # RViz 실행 로그를 터미널 화면에 출력
        )

    # Launch 시스템이 실행할 액션 리스트 반환 (여기서는 RViz만 실행)
    return [rviz_node]

def generate_launch_description():
    # LaunchDescription 객체를 생성하고, OpaqueFunction으로 launch_setup 함수를 호출하도록 지정
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # LaunchDescription 객체 생성
    ld = generate_launch_description()

    # LaunchService 객체 생성 (런치 실행 전체를 관리)
    ls = LaunchService()

    # LaunchDescription을 LaunchService에 등록
    ls.include_launch_description(ld)

    # LaunchService 실행 → RViz2가 실행됨
    ls.run()
