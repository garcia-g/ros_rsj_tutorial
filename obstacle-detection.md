---
title: 障害物認識と回避
date: 2020-01-23
---

# 障害物認識と回避

- Table of contents
{:toc}

TurtleBot3は、LDSのデータによって移動・停止できます。TurtleBot3が移動している際に、前方の障害物を検知した場合は停止します。

**実際のロボットを使う場合：**
Turtlebot3を起動します（[参照](./turtlebot-basics.html#実際のTurtleBotを操作)）。

次に[サンプルプログラム](#サンプルプログラムを実行)から続きます。

**シミュレーションで試したい場合：**

下記のROSパケージをダウンロードすることをおすすめします（コンビニのシミュレーション環境）。

```shell
$ cd ~/catkin_make/src/
$ git clone https://github.com/igra9/rsj_seminar_2021_navigation.git
$ cd ..
$ catkin_make
```

シミュレーションを起動します。

```shell
$ roslaunch rsj_seminar_2021_navigation turtlebot3_conbini.launch
```

PCの性能によってシミュレータを起動するのに多少時間がかかります。
Gazeboの画面が表示されるまでに待ちます。

## サンプルプログラムを実行

**リモートPCで**　obstacle fileを起動します。

```shell
$ roslaunch turtlebot3_example turtlebot3_obstacle.launch
```

このプログラムのソースコードが簡単な処理を行なっています。

```python
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Obstacle():
    def __init__(self):
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
        
    def get_scan(self):
        scan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 1            # 1 <= samples_view <= samples
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def obstacle(self):
        twist = Twist()
        turtlebot_moving = True

        while not rospy.is_shutdown():
            lidar_distances = self.get_scan()
            min_distance = min(lidar_distances)

            if min_distance < SAFE_STOP_DISTANCE:
                if turtlebot_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self._cmd_pub.publish(twist)
                    turtlebot_moving = False
                    rospy.loginfo('Stop!')
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self._cmd_pub.publish(twist)
                turtlebot_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)

def main():
    rospy.init_node('turtlebot3_obstacle')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass
```

上記のプログラム（`/opt/ros/kinetic/lib/turtlebot3_example/turtlebot3_obstacle.py`）を新しいファイルにコピーします。

新しいROS packageを作成するか`rsj_seminar_2021_navigation`のパケージに追加します。
```shell
$ cd ~/catkin_ws/src/rsj_seminar_2021_navigation
$ mkdir src
```
上記のフォルダーに作成したファイル`name.py`を保存します（適切に名前を変更してください）。

新しいパケージを作成するには：
```shell
$ cd ~/catkin_ws/src
$ catkin_create_pkg package-name message_generation rospy sensor_msgs geometry_msgs
$ cd package-name
$ mkdir src
```
上記のフォルダーに作成したファイルを保存します（拡張子はpyにします）。

```shell
$ chmod u+x ~/catkin_ws/src/package-name/src/name.py
```

`~/catkin_ws/src/package-name/CMakeLists.txt`のファイルに下記を追加します。
```CMakeLists
catkin_install_python(PROGRAMS src/name.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```
ファイルを保存して閉じます。

ビルドします。
```shell
$ cd ~/catkin_ws
$ catkin_make
```

実行を確認します。
```
$ rosrun package-name name.py
```

**課題** ロボットを停止するのではなくて障害物を避けて引き続いて移動できるようにプログラム`name.py`を変更します。


## 座標指令

TurtleBot3は、2次元上の点(x, y)とz-angularで移動することができる。たとえば、(0.5, 0.3, 60)を入れると、TurtleBot3は点（x = 0.5m, y = 0.3m）に移動し、60度回転します。

このプログラムを実行するには
```shell
 roslaunch turtlebot3_example turtlebot3_pointop_key.launch
 ```

実行すると下記の表示があります。

```shell
control your Turtlebot3!
-------------------------
Insert xyz - coordinate
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-------------------------

| x | y | z |
```

適切な座標を入力し、`Enter`{: style="border: 1px solid black" }キーを押すとロボットが動きます。

**課題** 上記のプログラムを参考しながら指定したポイントまで障害物を回避しながらロボットを移動させます。

`turtlebot3_pointtop_key.py`は　`/opt/ros/kinetic/lib/turtlebot3_example/`にあります。


<button type="button" class="bth btn-primary btn-lg">[
    <span style="color:black">**メインページへ**</span>](index.html)</button>