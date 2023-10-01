# Autonomous_MultiDrone_Racing
## Drone1_Code_Test

ros melodic에서 사용하는 python 2.7을 3으로 업그레이드 후 다음 과정들을 진행해주세요.
```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-melodic-desktop-full --fix-missing
```
```
sudo apt install python-pip
pip3 install --upgrade pip
pip3 install opencv-python
```

#### 1. simulation이라는 이름의 패키지 생성
```
cd ~/race_ws/src
catkin_create_pkg simulation rospy std_msgs
```
#### 2. drone1.launch 파일 경로 수정 
```
cd ~/race_ws/src
mkdir -p simulation/launch
cd ~/race_ws/src/simulation/launch
```
해당 경로에 drone1.launch파일을 옮겨주세요.(위의 터미널 실행 후 본 2.Drone1_codeTest폴더에 들어 있는 drone1.launch을 cd ~/race_ws/src/simulation/launch에 옮겨주시면 됩니다. )
#### 3. drone1.py 파일 경로 수정 
```
cd ~/race_ws/src/simulation/src

```
(2.Drone1_codeTest폴더에 들어 있는 drone1.py을 cd ~/race_ws/src/simulation/src에 옮겨주시면 됩니다. )
#### 4. 패키지 빌드
```
cd ~/race_ws/src
catkin build

```


-------------------

### CODE_TEST
[WSL 18.04 터미널 4개]
#### 1. UE4_Racing_Map의 Blocks project를 Unreal Engine으로 실행 및 Play 버튼 클릭

#### 2. Mavros 실행
```
export ROS_NAMESPACE=drone1
cd ~/race_ws
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:="udp://:14535@127.0.0.1:18570"

```
#### 3. PX4 실행
```
cd ~/PX4-Autopilot
make px4_sitl_default none_iris

```
#### 4. Airsim Ros Wrapper 실행
```
export WSL_HOST_IP=127.0.0.1
cd AirSim/ros/
source devel/setup.bash
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$WSL_HOST_IP

```
#### 5. simulation 패키지의 drone1.launch 실행
```
cd ~/race_ws/src/simulation/src
chmod +x drone1.py
cd ~/race_ws
source devel/setup.bash
roslaunch simulation drone1.launch
```

#### (참고) 언리얼엔진의 시점 변경방법 (디스플레이창 클릭 후)
F → 드론에 대한 1인칭

B → 드론에 대한 3인칭

M → 외부카메라 조종 (방향키와 WASD로 고정된 카메라 이동 가능)