## 국제 대학생 EV 자율주행 경진대회

ROS2 기반 1/5 자율주행 차량에서 장애물 회피를 위한 DWA local planner와 path tracking controller를 구현한 프로젝트입니다.

### Role

전체 자율주행 시스템 중 경로 생성 및 경로 제어 파트를 담당했습니다.

* DWA cost function 설계
* 주행 환경에 따른 DWA parameter tuning
* Pure Pursuit / Stanley controller 기반 조향 제어 및 parameter tuning
* DWA local path와 path tracking controller 연동


## DWA Local Planner

본 프로젝트에서는 차량 주변 장애물 상황을 반영하여 주행 가능한 local path를 선택하기 위해 DWA 기반 지역 경로 계획 알고리즘을 구현했습니다.

일반적인 DWA처럼 속도와 조향각을 모두 탐색하는 방식이 아니라, 본 코드에서는 예측 속도를 고정한 상태에서 여러 조향각 후보를 생성하고 각 후보 경로를 비교하는 방식으로 구성했습니다. 각 후보 경로는 자전거 모델을 기반으로 미래 위치를 예측하며, 전역 경로 추종성, 장애물 회피 안정성, heading 정렬 정도를 함께 고려하여 최종 local path를 선택합니다.

### Main Features

* 현재 차량 위치, heading, 조향 상태, 장애물 좌표를 입력으로 사용
* 제한된 조향각 범위 안에서 여러 개의 local path 후보 생성
* 자전거 모델 기반 미래 위치 예측
* 장애물과 가까운 후보 경로를 collision check 단계에서 사전 제거
* 전역 경로와의 횡방향 이격, 장애물 거리, heading error를 기반으로 cost 계산
* cost가 가장 낮은 후보 경로를 최종 `selected_path`로 선택

### Cost Function

DWA 후보 경로는 다음 요소들을 기반으로 평가됩니다.

* Global path와의 횡방향 거리
* 후보 경로의 초반점, 중간점, 끝점에서의 장애물 거리
* 후보 경로의 최종 heading과 global path heading의 차이
* 중앙 후보 경로가 막혔을 때 좌우 후보 경로의 장애물 분포
* 주행 모드에 따른 cost weight 조정

초기 설계 단계에서는 global path와의 거리, 곡률, target point 거리 등 여러 cost 항목을 함께 고려하도록 구성했습니다. 이후 연습주행 환경과 실제 대회 환경의 장애물 배치, 노면 조건, GPS 안정성이 달라지면서 실제 주행에서는 일부 cost weight를 0으로 두고, 대회 환경에서 안정적으로 동작한 항목 중심으로 weight를 재조정했습니다.

장애물 회피는 단순히 경로 끝점만 확인하지 않고, 경로 초반점과 중간점까지 함께 평가하도록 구성했습니다. 이를 통해 차량이 주행 중 장애물에 가까워지는 후보 경로를 줄이고, 더 안정적인 회피 경로를 선택할 수 있도록 했습니다.

또한 중앙 후보 경로가 연속적으로 막히는 상황에서는 좌우 후보군의 장애물 분포를 비교하여 한쪽 방향의 후보를 제거합니다. 이 로직을 통해 중앙 장애물 주변에서 불필요하게 위험한 방향으로 경로가 선택되는 경우를 줄였습니다.

### Output

DWA 모듈은 최종적으로 다음 값을 반환합니다.

* `selected_path`: cost가 가장 낮은 최종 선택 경로
* `candidate_paths`: collision check 이후 남은 후보 경로들

선택된 `selected_path`는 이후 path tracking 모듈로 전달되며, Pure Pursuit와 Stanley 기반 조향 제어에서 목표 경로로 사용됩니다.

---

## Pure Pursuit & Stanley Path Tracking

본 프로젝트에서는 DWA가 선택한 local path를 실제 차량이 추종하기 위한 조향 제어기로 Pure Pursuit와 Stanley controller를 함께 사용했습니다.

Pure Pursuit는 local path 위의 look-ahead target point를 추종하는 방식으로 조향각을 계산하고, Stanley controller는 경로 heading error와 앞바퀴 기준 cross-track error를 보정합니다. 최종 조향각은 두 제어기의 출력을 가중합하여 생성했습니다.

```python
target_steer = P_steer * 1.4 + stanley_steer * 0.4
```

현재 구조에서는 Pure Pursuit가 주 조향 제어 역할을 하고, Stanley controller는 경로 방향 오차와 횡방향 오차를 보정하는 보조 제어 역할을 합니다.

### Pure Pursuit

Pure Pursuit는 차량의 현재 위치에서 local path를 따라 일정 거리만큼 앞에 있는 목표점을 선택하고, 해당 목표점을 향하도록 조향각을 계산합니다.

* 현재 차량 위치와 가장 가까운 local path index 탐색
* 속도 기반 look-ahead distance 계산
* look-ahead distance만큼 앞에 있는 target point 선택
* 차량 heading과 target point 방향 차이를 이용한 조향각 계산
* 최종 조향각을 차량 조향 한계 범위로 제한

속도가 증가할수록 더 먼 목표점을 바라보도록 look-ahead distance를 설정하여, 저속에서는 민감하게 반응하고 고속에서는 더 부드러운 조향이 가능하도록 구성했습니다.

### Stanley Controller

Stanley controller는 차량의 앞바퀴 위치를 기준으로 경로와의 오차를 계산합니다. 조향각은 local path의 진행 방향과 차량 heading의 차이, 그리고 앞바퀴 기준 cross-track error를 함께 반영하여 계산됩니다.

* 차량 중심이 아닌 앞바퀴 위치 기준 오차 계산
* local path yaw와 차량 heading 차이 계산
* 앞바퀴 기준 cross-track error 계산
* heading error와 횡방향 오차 보정항을 더해 Stanley 조향각 계산
* 최종 조향각을 차량 조향 한계 범위로 제한

이를 통해 차량이 단순히 목표점만 따라가는 것이 아니라, 선택된 local path의 진행 방향과 횡방향 위치를 함께 맞추도록 보정했습니다.

### Control Flow

전체 제어 흐름은 다음과 같습니다.

```text
DWA local path selection
-> Cubic spline interpolation
-> Pure Pursuit steering calculation
-> Stanley steering correction
-> Weighted steering fusion
-> Final steering command
```

DWA는 주행 가능한 local path를 선택하고, Pure Pursuit와 Stanley controller는 선택된 경로를 실제 차량이 따라갈 수 있는 최종 조향 명령으로 변환합니다.

---

## Path Tracking Integration

`path_tracking_dwa.py`는 DWA local planner와 Pure Pursuit / Stanley controller를 연결하는 path tracking 통합 모듈입니다.

이 모듈은 DWA가 선택한 local path를 조향 제어기가 사용할 수 있는 연속적인 경로 형태로 변환하고, 최종 steering command를 생성하는 중간 계층 역할을 합니다.

### Overall Flow

```text
Current vehicle state
-> DWA local path planning
-> Selected path extraction
-> Cubic spline interpolation
-> Pure Pursuit steering calculation
-> Stanley steering correction
-> Final steering output
```

### Main Features

* 현재 차량 위치, heading, speed, 조향값, 장애물 정보를 입력으로 사용
* DWA를 실행하여 `selected_path`와 `candidate_paths` 생성
* DWA가 선택한 경로를 cubic spline으로 보간
* 보간된 local path를 Pure Pursuit와 Stanley controller에 전달
* Pure Pursuit 조향값과 Stanley 보정값을 가중합하여 최종 steer 계산
* 최종 steer, 선택 경로, 후보 경로, goal point 반환

### Spline Interpolation

DWA에서 반환되는 `selected_path`는 비교적 적은 수의 예측점으로 구성되어 있습니다. 따라서 Pure Pursuit와 Stanley controller가 안정적으로 경로를 추종할 수 있도록 cubic spline을 이용해 경로를 더 촘촘하고 부드럽게 보간했습니다.

보간 결과로 생성되는 값은 다음과 같습니다.

* `selected_rx`: 보간된 x 좌표
* `selected_ry`: 보간된 y 좌표
* `selected_ryaw`: 보간된 경로 yaw
* `selected_rk`: 경로 곡률
* `selected_s`: 누적 경로 거리

이를 통해 단순한 DWA 예측점이 아니라, 실제 조향 제어에 적합한 연속적인 local path를 생성할 수 있습니다.

### Steering Fusion

정상 동작 시 최종 조향각은 Pure Pursuit와 Stanley controller의 결과를 가중합하여 계산합니다.

```python
target_steer = P_steer * 1.4 + stanley_steer * 0.4
steer = np.clip(target_steer, -23, 23)
```

Pure Pursuit는 local path 위의 목표점을 추종하는 주 조향 제어 역할을 하고, Stanley controller는 heading error와 cross-track error를 보정하는 보조 제어 역할을 합니다. 최종 조향각은 실제 차량 조향 한계를 고려하여 제한합니다.

### Fallback Logic

DWA 실행 중 오류가 발생하거나 선택된 경로가 충분하지 않은 경우에는 fallback으로 Pure Pursuit 기반 조향을 사용합니다.

또한 spline 보간에 실패한 경우에는 selected path의 일부 점을 이용해 경로 방향을 추정하고, Pure Pursuit 조향값과 결합하여 조향값을 반환하도록 구성했습니다.

### Output

`gps_tracking()` 함수는 최종적으로 다음 값을 반환합니다.

* `steer`: 최종 조향각
* `selected_path`: DWA가 선택한 최적 local path
* `candidate_paths`: collision check 이후 남은 DWA 후보 경로들
* `goal_point`: Pure Pursuit가 추종한 목표점

즉, `path_tracking_dwa.py`는 DWA가 선택한 회피 경로를 실제 차량이 따라갈 수 있는 조향 명령으로 변환하는 핵심 연결부입니다.



> 본 repository는 전체 ROS2 workspace가 아니라, 대회에서 담당했던 DWA local planner와 path tracking controller 핵심 코드만 정리한 것입니다.