# Autonomous_MultiDrone_Racing
## Drone1_Setting

### 1. PX4 Firmware + Airsim Test
본 과정은 기본적인 UE4를 이용한 Airsim 시뮬레이터와 PX4 Firmware를 연동하는 과정입니다.
1. Drone1_Setting에 들어있는 settings.json파일을 문서/Airsim 경로에 복사하고, 기존에 존재하던 settings.json파일은 삭제
2. UE4_Racing_Map 폴더의 맵파일 Blocks를 실행시킨 이후 Play버튼 클릭
3. WSL를 관리자모드로 실행 후 다음 명령어 실행 (첫 실행시 빌드과정에서 시간 소요, Ubuntu 터미널에서도 동일)
```
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```
4. 드론이 연결되며 UE4 왼쪽 상단에 [Got GPS Lock] 문구가 뜨는지 확인

-------------------

### 2. PX4 Firmware + Airsim + Mavros Test
본 과정은 1번 과정에서 연동한 Airsim 시뮬레이터와 PX4와 함께 ROS 토픽을 받기 위한 Mavros를 연동하는 과정입니다. 
1번 과정에서 사용한 Settings.json을 그대로 사용합니다.
1. UE4_Racing_Map 폴더의 맵파일 Blocks를 실행시킨 이후 Play버튼 클릭
2. WSL를 관리자모드로 2개 실행 후, 각각의 창에서 다음 코드를 순서대로 실행
```
roslaunch mavros px4.launch fcu_url:="udp://:14535@127.0.0.1:18570"
```
```
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```
4. 드론이 연결되며 UE4 왼쪽 상단에 [Got GPS Lock] 문구가 뜨는지 확인
   + 만약 정상적으로 드론이 연결되지 않는다면, 포트번호에 문제가 있을 수 있습니다.
   + mavros를 실행시키는 코드를 다음 코드로 실행해보세요.
   + ```
      roslaunch mavros px4.launch fcu_url:="udp://:14030@127.0.0.1:14280"
      ```
5. 다음 명령어를 실행 후 rostopic들이 정상적으로 출력되는지 확인
```
cd
rostopic list
rostopic echo /mavros/imu/data_raw
```
6. rosservice list를 확인 후 drone 제자리 비행 명령
   + 다음 명령어를 실행하여 드론이 호버링(제자리에서 비행)하는지 확인
   + ```
      rosservice list
      rosservice call /mavros/cmd/arming "value: true"
      ```


-------------------

### 3. PX4 Firmware + Airsim + Mavros + Airsim ROS Wrapper Test
본 과정은 2번 과정에서 연동한 Airsim 시뮬레이터와 PX4, Mavros와 함께 Airsim ROS Wrapper를 통해 Airsim의 ROS Topic까지 수신할 수 있도록하는 과정입니다.
1. "1. PX4 Firmware + Airsim Test"에서 저장한 Airsim settings.json 파일에 들어가 모두 주석 처리 후 (윈도우의 문서/Airsim/settings.json파일) 아래 코드에 해당하는 부분의 밑 코드만 주석을 해제 시킨 후 저장
   + ```
      //##########################################################################
      //################ 3. PX4 Firmware + Airsim + Mavros Test ##################
      //################ This settings.json file is for 1 Drone ##################
      //##########################################################################
      ```
2. 앞서 실행한 PX4 + Mavros 실행
```
roslaunch mavros px4.launch fcu_url:="udp://:14535@127.0.0.1:18570"
```
```
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```
3. Airsim ROS Wrapper 실행
```
export WSL_HOST_IP=127.0.0.1
cd AirSim/ros/
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
```
4. 드론이 호버링하다가 멈추는 현상이 나타나면 정상적으로 실행된 것입니다.


