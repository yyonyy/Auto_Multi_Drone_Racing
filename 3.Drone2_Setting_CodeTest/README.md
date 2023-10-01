# Autonomous_MultiDrone_Racing
## Drone2_Setting_Code_Test

1. Mavros의 px4.launch 복사
```
cd ~/race_ws/src/mavros/mavros/launch
cp px4.launch px4_1.launch
cp px4.launch px4_2.launch
```
2. Mavros의 px4.launch 수정
px4_1.launch파일은 그대로 사용하고, px4_2.launch파일의 경우 tgt_system을 2로 수정합니다.
```
nano px4_2.launch
```
해당 명령어를 통해 launch 파일 진입 후 "tgt_system"이 있는 줄을 찾아
```
<arg name="tgt_system" default="2" /> 
```
으로 바꾸어 주세요.

3. Airsim settins.json 경로 수정
( 본 파일에 들어있는 2개 드론용 settings.json을 (윈도우의 문서/Airsim/) 경로로 수정해주세요. 해당 경로에 Drone1_setting을 위해 들어있던 settings.json파일은 이름을 바꿔주시거나 삭제해 주셔야 합니다. )

4. drone2.launch 파일 생성 및 패키징
```
cd ~/race_ws/src/simulation/launch
```
해당 경로에 drone2.launch파일을 옮겨주세요.(본 3.Drone2_Setting_CodeTest폴더에 들어 있는 drone2.launch을 cd ~/race_ws/src/simulation/launch에 옮겨주시면 됩니다. )
5. 패키지 빌드
```
cd ~/race_ws/src
catkin build
```
____________________

## 실행 순서
1. UE4_Racing_Map의 Blocks project를 Unreal Engine으로 실행 및 Play 버튼 클릭
2. Mavros 실행 ( 각각의 터미널에서 실행해 주셔야 합니다.)
```
export ROS_NAMESPACE=drone1
cd ~/race_ws
source devel/setup.bash
roslaunch mavros px4_1.launch fcu_url:="udp://:14535@127.0.0.1:18570"
```
```
export ROS_NAMESPACE=drone2
cd ~/race_ws
source devel/setup.bash
roslaunch mavros px4_2.launch fcu_url:="udp://:14536@127.0.0.1:18571"
```
3. PX4 실행
```
cd ~/PX4-Autopilot
./Tools/sitl_multiple_run.sh 2
```
4. Airsim Ros Wrapper 실행
```
export WSL_HOST_IP=127.0.0.1
cd AirSim/ros/
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP
```
5. Simulation 패키지의 launch 파일 실행
```
cd ~/race_ws/src/simulation/src
chmod +x drone1.py
cd ~/race_ws
source devel/setup.bash
roslaunch simulation drone1.launch
```
(위의 명령어와 동시에 실행시키는경우 드론이 바로 충돌하니 조금 간격을 두고 실행해주시는게 좋습니다.)
```
cd ~/race_ws/src/simulation/src
chmod +x drone2.py
cd ~/race_ws
source devel/setup.bash
roslaunch simulation drone2.launch
```
