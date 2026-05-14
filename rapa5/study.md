# ToF 낭떠러지 감지 시스템 정리

## 1. 하드웨어 구성

| 장치 | 포트 | 역할 |
|---|---|---|
| OpenCR | `/dev/ttyACM2` | 모터 드라이버, IMU, 오도메트리 |
| VL53L8CX (Pico 2) | `/dev/ttyACM3` | 8×8 ToF 거리 센서 |

- OpenCR ↔ ROS: Dynamixel SDK (baudrate 1,000,000)
- ToF ↔ ROS: USB 시리얼 (baudrate 115,200), 텍스트 프로토콜

---

## 2. 시리얼 데이터 형식 (Pico → ROS)

```
Frame #22653
       C0  C1  C2  C3  C4  C5  C6  C7
R0     706  755  811  869  966 1050 1172 ----
R1     694  744  794  871  934 1031 1121 1779
...
R7     ...
```

- 단위: mm
- `----`: 측정 불가 (Out of Range, OOR)
- 펌웨어(Pico)가 `target_status` 판단 후 유효값 또는 `----`로 변환해서 전송
  → ROS 레벨에서는 별도 status 필드 없음, `OOR = -1`로 대체

---

## 3. ROS 노드 구성

```
[VL53L8CX Pico]
      │ /dev/ttyACM3 (serial)
      ▼
[tof_serial_node]  ── /tof/grid (Float32MultiArray, 64개 float, OOR=-1.0)
                   ── /tof/heatmap (sensor_msgs/Image, BGR8 480×480)
                   ── /tof/pointcloud (sensor_msgs/PointCloud2)
      │
      ▼
[cliff_detector_node]  ── /cliff_detected (std_msgs/Bool)
      │
      ▼
[cliff_manager_node]  ── /motor_power 서비스 호출 (std_srvs/SetBool)
                      ── TTS 음성 출력 (gTTS + pygame)
```

---

## 4. 좌표계 (8×8 그리드)

```
        C0(왼쪽) ──────────── C7(오른쪽)
   R0  [ 먼 영역 ← 센서 FOV 상단 ]  ← R0 ≈ 1100~1600mm
   R1  [                          ]
   R2  [                          ]
   R3  [                          ]
   R4  [                          ]
   R5  [                          ]  ← CLIFF_ROWS 시작
   R6  [                          ]
   R7  [ 가까운 바닥 ← 센서 FOV 하단 ] ← R7 ≈  700~900mm
```

- R0: 센서 기준 위쪽(먼 거리), R7: 아래쪽(가까운 바닥)
- 낭떠러지 감지 대상: **R5, R6, R7** (하단 3행)
- 실제 앞/뒤 방향은 센서 장착 방향에 따라 다름 — 물리적으로 확인 필요

---

## 5. 색상 체계

### `/tof/heatmap` (RViz2)

`cv2.COLORMAP_JET` 적용, **프레임 내 상대 정규화**:

| 색상 | 의미 |
|---|---|
| 파란색 | 현재 프레임에서 가장 가까운 픽셀 |
| 초록 → 노랑 | 중간 거리 |
| 빨간색 | 현재 프레임에서 가장 먼 픽셀 |
| 회색 (128,128,128) | OOR |

> 절대 거리 기준이 아님. 프레임마다 min/max가 바뀌면 색상도 바뀜.

### 터미널 viewer (VMware)

heatmap과 동일한 방향으로 통일 완료:

| 색상 | 의미 |
|---|---|
| 파란색 | 가까운 픽셀 (min 근처) |
| 초록 → 노랑 | 중간 거리 |
| 빨간색 | 먼 픽셀 (max 근처) |
| 회색 | OOR (OOR 처리 버그 수정 완료) |

---

## 6. OOR(status) 처리 현황

| 위치 | 처리 방식 |
|---|---|
| `tof_serial_node.py` `_publish_heatmap` | `valid_mask = grid != OOR` — 히트맵 색상 계산에서 OOR 제외 |
| `tof_serial_node.py` `_timer_cb` | `v != OOR` — 로그 통계에서 OOR 제외 |
| `cliff_detector_node.py` `_grid_cb` | `v < 0` — OOR도 낭떠러지로 판정 |

---

## 7. 낭떠러지 감지 로직 (`cliff_detector_node.py`)

### 개념

ToF 센서가 바닥을 향해 기울어진 상태로 장착되어 있음.  
정상적인 바닥이라면 R5~R7 영역의 거리값은 700~900mm 정도로 일정함.  
로봇이 낭떠러지(계단, 턱) 앞에 다가가면 바닥이 끊겨 거리값이 갑자기 커지거나 OOR이 됨.  
→ 이 변화를 감지해서 `/cliff_detected = True`를 퍼블리시하고 모터를 정지시킴.

```
정상 상태          낭떠러지 접근
┌──────────┐       ┌──────────┐
│ 센서     │       │ 센서     │
│  ↓↓↓    │       │  ↓↓↓    │
│ 바 닥    │       │ 바닥  ↓  │  ← 거리 급증 또는 OOR
│ ~800mm  │       │       공백│
└──────────┘       └──────────┘
```

### 파라미터

```python
CLIFF_ROWS      = [5, 6, 7]  # 하단 3행 (로봇 바로 아래 바닥 영역)
CLIFF_DIST_MM   = 800.0       # 이 값 이상이면 낭떠러지 (바닥이 멀어짐)
MIN_PIXELS      = 4           # 24개 픽셀(3행×8열) 중 4개 이상 조건 충족 시
DEBOUNCE_FRAMES = 3           # 연속 3프레임 감지 시 확정 (노이즈 오감지 방지)
```

판정 조건: `v < 0 (OOR) or v >= 800mm` 인 픽셀이 4개 이상, 3프레임 연속

---

## 8. 실행 명령

```bash
# TurtleBot3 bringup (OpenCR ACM2)
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-03
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=30 \
  ros2 launch turtlebot3_bringup robot.launch.py usb_port:=/dev/ttyACM2

# ToF 노드 (기본값 ACM1이므로 ACM3 명시 필요)
ros2 run vl53l8cx_ros2 tof_serial_node --ros-args -p port:=/dev/ttyACM3

# 낭떠러지 감지
ros2 run vl53l8cx_ros2 cliff_detector_node

# 낭떠러지 관리 (모터 제어 + TTS)
ros2 run cliff_manager cliff_manager_node
```

---

## 9. 주요 트러블슈팅

### colcon --symlink-install이 Python 파일을 심링크하지 않는 문제

Python 패키지는 `egg-link` 방식으로 `build/` 디렉토리를 가리킴.  
소스 수정 후 `colcon build --symlink-install --packages-select <패키지명>` 재실행 필요.

```
install/ → (egg-link) → build/vl53l8cx_ros2/vl53l8cx_ros2/
                               ↑
                         소스에서 복사된 파일 (심링크 아님)
```

### 포트 fallback으로 OpenCR에 잘못 연결되는 문제

기존 `_open_serial`의 candidates 리스트가 `ACM0 → USB0 → ACM1` 순으로 시도해서  
OpenCR이 연결된 ACM0에 먼저 붙어버리는 문제 → fallback 제거, 지정 포트만 사용하도록 수정.

### ROS_DOMAIN_ID 불일치

Pi: `ROS_DOMAIN_ID=30` 고정.  
VMware/PC에서도 동일하게 설정 필요:
```bash
export ROS_DOMAIN_ID=30
```
VMware NAT 모드면 DDS 멀티캐스트 불통 → Bridged 모드 또는 FastDDS unicast peer 설정 필요.
