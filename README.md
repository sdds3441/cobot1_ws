## 📌 주요 실행 파일 안내

### 작업영역 x:500-200, y:-600, z:500-200


### `topic_test.py` (Main)
#### ros2 run dsr_rokey2 topic_test

토픽 실험

### `service_test.py` (Main)
#### ros2 run dsr_rokey2 service_test

서비스 실험

### `sky_cleaner.py` (Main)
#### ros2 run dsr_rokey2 move_basic

이 프로젝트의 메인 실행 파일입니다.  
청소 동작 및 전체 로직을 담당하며, 실제 로봇 동작을 시작할 때 실행해야 하는 핵심 코드입니다.


### `home.py` (Home Position)
#### ros2 run dsr_rokey2 home

로봇을 기본 자세(Home Position)로 복귀시키는 기능을 수행합니다.  
로봇을 초기 자세 또는 대기 상태로 이동시키고 싶을 때 실행합니다.



### `cleanning_task.py` (Cleaning Task)
#### 역할: **청소 동작 수행**

청소 경로 이동, 구역별 동작 제어 등 실제 **청소 로직을 관리**하는 파일입니다.  
`sky_cleaner.py`에서 호출되어 전체 청소 시퀀스 안에서 동작합니다.



### `grip_tool.py` (Tool Pickup)
#### 역할: **청소 도구 자동 픽업**

청소 도구(브러쉬, 패드 등)를 로봇 그리퍼로 **집고 장착하는 기능**을 담당합니다.  
로봇이 작업 시작 전 필요한 도구를 자동으로 확보하는 과정에 사용됩니다.
