---
title: ROSを用いたマップ取得
date: 2020-01-23
---

# ROSを用いたマップ取得

- Table of contents
{:toc}

**SLAM（Simultaneous Localization and Mapping）**は、任意の空間の現在位置を推定して地図を描く手法です。 SLAMは、TurtleBotの前身からよく知られている機能です。

## SLAMノードの実行

roscoreを実行します。

``` bash
$ roscore
```

TurtleBot3のアプリケーションを起動するための基本的なパッケージを起動します。

新しいターミナルウィンドウを開き、TurtleBotと接続します。

```shell
  ssh pi@192.168.xxx.xxx (The IP 192.168.xxx.xxx is your Raspberry Pi’s IP or hostname)
```
パスワードは**turtlebot**です。

接続ができましたら下記のコマンドでTurtleBot3を起動します。

``` bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

新しいターミナルを開き、SLAMファイルを起動します。

``` bash
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```

{% capture slam_tip %}
**ヒント**: 上記のコマンドを実行すると、視覚化ツールRVizも実行されます。 RVizを個別に実行する場合は、次のいずれかのコマンドを使用します。

  - $ rviz -d \`rospack find turtlebot3_slam\`/rviz/turtlebot3_gmapping.rviz
  - $ rviz -d \`rospack find turtlebot3_slam\`/rviz/turtlebot3_cartographer.rviz
  - $ rviz -d \`rospack find turtlebot3_slam\`/rviz/turtlebot3_hector.rviz
  - $ rviz -d \`rospack find turtlebot3_slam\`/rviz/turtlebot3_karto.rviz
  - $ rviz -d \`rospack find turtlebot3_slam\`/rviz/turtlebot3_frontier_exploration.rviz

{% endcapture %}

<div class="notice--info">{{ slam_tip | markdownify }}</div>

{% capture notice_03 %}
**注釈**: さまざまなSLAMメソッドをサポートしています
- TurtleBot3は、さまざまなSLAMメソッドの中で、Gmapping、Cartographer、Hector、およびKartoをサポートしています。 これを行うには、 `slam_methods：= xxxxx`オプションを変更します。
- `slam_methods`オプションには` gmapping`、 `cartographer`、` hector`、 `karto`、` frontier_exploration`が含まれ、それらの1つを選択できます。
- たとえば、kartoを使用するには、次のようにします:
```shell
$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto
```
{% endcapture %}
<div class="notice--info">{{ notice_03 | markdownify }}</div>

{% capture notice_04 %}
**注釈**: SLAMパッケージの依存関係パッケージをインストールします
- `Gmapping`の場合: <br>
Gmappingに関連するパッケージは、[事前準備](linux_and_ros_install.html＃ROS1 依存パッケージのインストール)ページですでにインストールされています。
- `Cartographer`の場合:
```shell
$ sudo apt-get install ros-kinetic-cartographer ros-kinetic-cartographer-ros \
  ros-kinetic-cartographer-ros-msgs ros-kinetic-cartographer-rviz
```
- `Hector Mapping`の場合:
```shell
$ sudo apt-get install ros-kinetic-hector-mapping
```
- `Karto`の場合:
```shell
$ sudo apt-get install ros-kinetic-slam-karto
```
- `Frontier Exploration`の場合: <br>
Frontier Explorationはgmappingを使用しており、次のパッケージをインストールする必要があります。
```shell
$ sudo apt-get install ros-kinetic-frontier-exploration ros-kinetic-navigation-stage
```
{% endcapture %}
<div class="notice--info">{{ notice_04 | markdownify }}</div>

今回は`Gmapping`を使用します。


## 遠隔操作ノードの実行

新しいターミナルを開き、[前回の実習](turtlebot-basics.html/#キーボードでロボットを操作)で使用した遠隔操作ノードを実行します。 次のコマンドを使用すると、ユーザーはロボットを制御してSLAM操作を手動で実行できます。 速度の変更が速すぎたり、回転が速すぎたりするなどの激しい動きを避けることが重要です。 ロボットを使用して地図を作成する場合、ロボットは測定対象の環境の隅々までスキャンする必要があります。 きれいな地図を作成するにはある程度の経験が必要なので、SLAMを複数回練習してノウハウを作成しましょう。 マッピングプロセスを次の図に示します。

``` bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

``` bash
  Control Your TurtleBot3!
  ---------------------------
  Moving around:
          w
     a    s    d
          x

  w/x : increase/decrease linear velocity
  a/d : increase/decrease angular velocity
  space key, s : force stop

  CTRL-C to quit
```

![](/images/turtlebot3/slam_running_for_mapping.png)

## チューニングガイド

Gmappingには、さまざまな環境のパフォーマンスを変更するための多くのパラメーターがあります。 パラメーター全体に関する情報は、[ROS WiKi](http://wiki.ros.org/gmapping)で入手するか、[ROS Robot Programming](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51)の第11章を参照してください。

このチューニングガイドでは、重要なパラメーターを設定するためのヒントをいくつか紹介します。 環境に応じてパフォーマンスを変更したい場合は、このヒントが役立つ可能性があり、時間を節約できます。

下記のパラメータのデフォルト値は`/opt/ros/kinetic/share/turtlebot3_slam/config/gmapping_params.yaml`のファイルに定義されています。`rosparam set`で変更することができます。

_**maxUrange**_ 
- デフォルト値：3.0
- このパラメーターは、LIDARセンサーの最大使用可能範囲を設定します。
```shell
$ rosparam set /turtlebot3_slam_gmapping/maxUrange X
```

_**map_update_interval**_
- デフォルト値：2.0 
- マップの更新間の時間（秒単位）。 これを低く設定すると、マップがより頻繁に更新されます。 ただし、より大きな計算負荷が必要になります。 このパラメーターの設定は、環境によって異なります。
```shell
$ rosparam set /turtlebot3_slam_gmapping/map_update_interval X
```

![](/images/turtlebot3/tuning_map_update_interval.png)

_**minimumScore**_ 
- デフォルト値：50 
- スキャンマッチングの結果を考慮するための最小スコア。 このパラメーターにより、ポーズ推定のジャンプを回避できます。
   これが適切に設定されている場合は、（Slamのノードが起動しているターミナルで）以下の情報を見ることができます。

  ```
  Average Scan Matching Score=278.965
  neff= 100
  Registering Scans:Done
  update frame 6
  update ld=2.95935e-05 ad=0.000302522
  Laser Pose= -0.0320253 -5.36882e-06 -3.14142
  ```

  この設定が高すぎる場合は、以下の警告が表示されます。

  ```
  Scan Matching Failed, using odometry. Likelihood=0
  lp:-0.0306155 5.75314e-06 -3.14151
  op:-0.0306156 5.90277e-06 -3.14151
  ```
```shell
$ rosparam set /turtlebot3_slam_gmapping/minimumScore X
```

_**linearUpdate**_ 
- デフォルト値：1.0 
- ロボットが移動すると、毎回スキャン処理が行われます。
```shell
$ rosparam set /turtlebot3_slam_gmapping/linearUpdate X
```

_**angularUpdate**_ 
- デフォルト値：0.2
- ロボットが回転すると、毎回スキャン処理が行われます。 これをlinearUpdateよりも小さく設定することを推奨します。
```shell
$ rosparam set /turtlebot3_slam_gmapping/angularUpdate X
```

パラメータの値を確認したい場合は：
```shell
$ rosparam get パラメータ名
```

どいうパラメータがあるかのとパラメータ名を調べるには：
```shell
$ rosparam list
```


## マップの保存

すべての作業が完了したので、`map_saver`ノードを実行してマップファイルを作成します。 マップは、ロボットのオドメトリ、tf情報、およびロボットが移動したときのセンサーのスキャン情報に基づいて描画されます。これらのデータは、前のサンプルビデオのRVizで見ることができます。作成されたマップは、`map_saver`が実行されているディレクトリに保存されます。ファイル名を指定しない限り、マップ情報を含む`map.pgm`および`map.yaml`ファイルとして保存されます。

``` bash
$ rosrun map_server map_saver -f ~/map
```

`-f`オプションは、マップファイルが保存されているフォルダーとファイル名を参照します。`~/map`をオプションとして使用すると、`map.pgm`と `map.yaml`がユーザーのホームフォルダー`~/`（$HOME：`/home/<username>`）のmapフォルダーに保存されます。

## マップ

ROSコミュニティでよく使用されている2次元の `Occupancy Grid Map（OGM）`を使用します。 下の図に示すように、前の[マップの保存](#マップの保存)セクションから取得したマップ。**白色**はロボットが移動可能な空き領域、**黒色**はロボットが動作できない占有領域です。**灰色**は未知の領域です。 このマップは[ナビゲーション][ros-navigation.html]で使用されます。

![](/images/turtlebot3/map.png)

次の図は、TurtleBot3を使用して大きなマップを作成した結果を示しています。 移動距離が約350メートルの地図を作成するのに約1時間かかりました。

![](/images/turtlebot3/large_map.png)

自分で作成したマップ（map.pgm）をダブルクリックで開けます。

## 参考文献

- gmapping
  - [ROS WIKI](http://wiki.ros.org/gmapping), [Github](https://github.com/ros-perception/slam_gmapping)

- cartographer 
  - [ROS WIKI](http://wiki.ros.org/cartographer), [Github](https://github.com/googlecartographer/cartographer)

- hector
  - [ROS WIKI](http://wiki.ros.org/hector_slam), [Github](https://github.com/tu-darmstadt-ros-pkg/hector_slam)

- karto
  - [ROS WIKI](http://wiki.ros.org/slam_karto), [Github](https://github.com/ros-perception/slam_karto)

- frontier_exploration 
  - [ROS WIKI](http://wiki.ros.org/frontier_exploration), [Github](https://github.com/paulbovbel/frontier_exploration)


<button type="button" class="bth btn-primary btn-lg">[
    <span style="color:black">**メインページへ**</span>](index.html)</button>
<button type="button"  class="bth btn-success btn-lg">
    [<span style="color:black">**次の実習へ**</span>](ros-navigation.html)</button>