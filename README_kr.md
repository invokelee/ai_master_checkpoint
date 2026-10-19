# ai_master_checkpoint

[English](README.md) | 한국어

TurtleBot4 시뮬레이션에서 `/odom`, `/scan`을 이용해 목표 지점까지 자율 주행하고,
도착 시 `/camera/image_raw`의 이미지를 저장한 뒤 결과 리포트를 자동 생성하는 ROS 2 패키지입니다.

## 실행

```bash
cd ~/ros2_ws
colcon build --packages-select ai_master_checkpoint
source install/setup.bash
ros2 launch ai_master_checkpoint checkpoint.launch.py
```

launch 한 번으로 모든 노드가 시작되며, 미션이 끝나면 (`mission_controller` 종료) 자동으로 전체가 종료됩니다.
시뮬레이터는 미리 실행되어 있어야 하며, 로봇은 시작 위치에 있어야 합니다.

## 구성

```
ai_master_checkpoint/
├── checkpoint_report.txt          # 실행 시 자동 생성되는 결과 리포트
├── checkpoint_result/
│   └── arrival_image.png          # 도착 시 저장되는 카메라 이미지
├── ai_master_checkpoint/
│   ├── mission_controller.py      # 주행, 도착 판정, 이미지/리포트 저장
│   ├── robot_monitor.py           # /odom, /scan 상태 진단 로그
│   └── navigation_algorithm.py    # 제공된 FastObstacleNavigator (수정하지 않음)
├── launch/
│   └── checkpoint.launch.py
├── package.xml
├── setup.cfg
└── setup.py
```

## 노드

### mission_controller

| 구분 | 토픽 | 타입 |
|---|---|---|
| 구독 | `/odom` | `nav_msgs/Odometry` |
| 구독 | `/scan` | `sensor_msgs/LaserScan` |
| 구독 | `/camera/image_raw` | `sensor_msgs/Image` |
| 발행 | `/cmd_vel` | `geometry_msgs/Twist` |

동작:

1. `/odom`에서 위치(x, y)와 yaw를 얻습니다.
2. `/scan`에서 `inf`, `nan`, `range_min`/`range_max` 밖의 값을 제외하고, 5개 구간별 최소 거리를 계산합니다.

   | 구간 | 각도 (왼쪽이 +) |
   |---|---|
   | front | -15° ~ 15° |
   | front_left | 15° ~ 60° |
   | front_right | -60° ~ -15° |
   | left | 60° ~ 120° |
   | right | -120° ~ -60° |

   유효한 값이 없는 구간은 `inf`로 전달합니다.
3. 위치, 목표 좌표, 구간별 거리를 `FastObstacleNavigator.compute()`에 전달하고, 반환된 선속도와 각속도를 `/cmd_vel`로 발행합니다.
4. 목표까지의 거리가 `goal_tolerance` 이하이면 정지합니다.
5. 정지 후 카메라 프레임을 `checkpoint_result/arrival_image.png`로 저장하고, 도착 메시지를 출력한 뒤 `checkpoint_report.txt`를 작성하고 종료합니다.

파라미터:

| 이름 | 기본값 | 설명 |
|---|---|---|
| `target_x` | -5.62021446228027 | 목표 x |
| `target_y` | -3.63326478004456 | 목표 y |
| `goal_tolerance` | 0.3 | 도착 판정 반경 (m) |
| `control_rate` | 10.0 | 제어 주기 (Hz) |
| `package_dir` | (자동 탐색) | 결과 파일을 저장할 패키지 소스 경로 |

패키지 소스 경로는 현재 디렉터리와 `~/ros2_ws` 아래의 `ai_master_checkpoint/` 또는 `src/ai_master_checkpoint/`에서 `package.xml`을 찾아 자동으로 결정합니다.

### robot_monitor

`/odom`과 `/scan`을 구독하여 주기적으로 위치, yaw, 속도, 최소 장애물 거리와 방향, 유효 range 수, 수신 주기(Hz)를 출력합니다.
데이터가 없거나 `stale_timeout` 동안 갱신되지 않으면 경고를 출력합니다.

| 이름 | 기본값 | 설명 |
|---|---|---|
| `report_period` | 1.0 | 출력 주기 (s) |
| `stale_timeout` | 2.0 | 데이터 갱신이 없을 때 경고하는 시간 (s) |

## 결과 파일

- `checkpoint_report.txt`: 시스템 정보, ROS 2 토픽과 메시지 타입, 토픽 주파수, 토픽 필드 설명, RViz2 Fixed Frame(`odom`), TF 프레임, TF 설명, 미션 결과가 들어 있습니다. 실행할 때마다 덮어씁니다.
- `checkpoint_result/arrival_image.png`: 도착 시 `/camera/image_raw`에서 저장한 이미지입니다.

리포트의 시스템 정보, 토픽 목록, 토픽 주파수, 미션 결과는 실행 시점에 측정한 값이고, 필드 설명과 TF 관련 항목은 고정된 문구입니다.

## 참고

- 이미지는 `cv_bridge` 없이 numpy와 OpenCV로 변환합니다 (`bgr8`, `rgb8`, `bgra8`, `rgba8`, `mono8` 지원).
- `navigation_algorithm.py`는 제공된 파일이며 수정하지 않았습니다. 이 파일의 docstring 때문에 `colcon test`의 `test_pep257`은 실패합니다.
