{% capture notice_01 %}
**警告**:
- この章の内容は、**TurtleBot3 Burger**のメインコンピューターとなる `Raspberry Pi 3`に対応しています。 この指示をリモートPC（デスクトップPCまたは、ラップトップ）で**実施しない**でください。
- セットアップ作業には、電源と時間が必要なためバッテリーは適していません。この作業では、SMPS(ACアダプタ)の使用を推奨します。
{% endcapture %}

### RaspbianベースのLinuxをインストールする

**警告**: Raspberry Pi 3にLinuxをインストールするには、SDカードに少なくとも**8GB**の空き容量が必要です。
{: .notice--warning}

RaspbianベースのLinuxディストリビューションイメージを提供しています。 それらはTurtleBot3に関連するROSおよびROSパッケージと共にプリインストールされています。 TurtleBot3 BurgerおよびWaffle Piモデルをサポートしています。 このディストリビューションイメージでは、Wolfram、Mathematica、Minecraft Pi、Oracle Java SEなどの非フリーソフトウェアが削除されています。

#### リモートPCで
- Raspbian for TurtleBot3に基づくLinuxディストリビューションイメージをダウンロードします。
  - [ダウンロードリンク](http://www.robotis.com/service/download.php?no=1738)
  - SHA256 (image_rpi_20190429.img.zip) : eb8173f3727db08087990b2c4e2bb211e70bd54644644834771fc8b971856b97
  - SHA256 (image_rpi_20190429.img): 7a868c275169b1f02c04617cc0cce9654fd8222623c78b22d0a27c73a9609398
- ダウンロード後、ダウンロードしたファイルを解凍します。
- SDカードのイメージを書き込み手順
  - [etcher.io](https://etcher.io/)にアクセスし、Etcher SDカードイメージユーティリティをダウンロードし、インストールします。
  - Etcherを実行し、コンピューターまたはラップトップにダウンロードしたLinuxイメージを選択します。
  - SDカードドライブを選択します。
  - 書き込みを選択して、イメージをSDカードに転送します。
- （他の書き込み方法）Linuxでは`dd`コマンドを使用できます。Windowsではアプリケーション`win32 Disk Imager`を使用できます。 完全な手順については、[こちら](https://elinux.org/RPi_Easy_SD_Card_Setup#Using_the_Linux_command_line)（Linuxユーザーの場合）および[こちら](https://elinux.org/RPi_Easy_SD_Card_Setup#Using_the_Win32DiskImager_program)（Windowsユーザーの場合）をご覧ください。

#### TurtleBot PCで
まだ電源に繋いでいないことを確認してください。{: style="color: red"}
- Raspberry PiをモニターにHDMIケーブルで接続し、キーボードとマウスをRaspberry Piに接続します。
- SDカードをRaspberry Piに差し込みます。
- 電源に繋いでください。
- Raspbian OSのインストール後、ユーザー名**pi**とパスワード**turtlebot**でログインできます。 

- SDカード全体を使用するようにファイルシステムを拡張します。
  ```
  sudo raspi-config
  (select 7 Advanced Options > A1 Expand Filesystem)
  ```

- [ワイヤレス・ネットワークへの接続ガイド](https://projects.raspberrypi.org/en/projects/raspberry-pi-using/3)

- ネットワークタイムプロトコル（NTP）サーバーにクエリを送信して、コンピューターの日付と時刻を同期および設定します。
  ```
  sudo apt-get install ntpdate
  sudo ntpdate ntp.ubuntu.com
  ```

- パスワード、ロケール、タイムゾーンを変更したい場合(オプション):
  1. sudo raspi-config > 1 Change User Password
  1. sudo raspi-config > 4 Localisation Options > I1 Change Locale
  1. sudo raspi-config > 4 Localisation Options > I2 Change Timezone

- ROSのネットワーク設定 [(reference link)][network_configuration]
	```
	nano ~/.bashrc
	(modify `localhost` to REMOTE_PC_IP and RASPBERRY_PI_3_IP)

	export ROS_MASTER_URI=http://REMOTE_PC_IP:11311
	export ROS_HOSTNAME=RASPBERRY_PI_3_IP
	```

	```
	source ~/.bashrc
	```

##### リモートPC
- ワイヤレス構成が完了したら、デスクトップまたはラップトップからSSH経由でRaspberry Piに接続できます。[(参照リンク)][enable_ssh_server_in_raspberry_pi]:
  ```
  ssh pi@192.168.xxx.xxx (The IP 192.168.xxx.xxx is your Raspberry Pi's IP or hostname)
  ```
{% capture notice_03 %}
**注釈**: **公式 Raspbian Stretchとの変更点**
- [Raspbian Stretch with desktop](https://www.raspberrypi.org/downloads/raspbian/)をベースにしています。 RaspbianはDebian Stretchをベースにしています。
- Wolfram、Mathematica、Minecraft Pi、Oracle Java SEなどの非フリーソフトウェアを削除しています。
- libreofficeを削除してイメージのサイズを小さくしています。
- raspi-configを使用してSSHおよびカメラ機能を有効化しています。
- パスワードを変更しています: **turtlebot**
- ROSおよびTurtleBot3のソフトウェアがインストール済みです。
  - [ROS Kinetic Kame](http://wiki.ros.org/kinetic)と依存パッケージ
  - [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) Raspberry Pi Cameraの依存パッケージ
  - [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)レーザ距離センサの依存パッケージ
  - [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)および、 [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
  のパッケージ
  - インストール済みのROSパッケージ (132パッケージ): actionlib, actionlib_msgs, angles, bond, bond_core, bondcpp, bondpy, camera_calibration_parsers, camera_info_manager, catkin, class_loader, cmake_modules, collada_parser, collada_urdf, common_msgs, compressed_image_transport, control_msgs, cpp_common, cv_bridge, diagnostic_aggregator, diagnostic_analysis, diagnostic_common_diagnostics, diagnostic_msgs, diagnostic_updater, diagnostics, dynamic_reconfigure, eigen_conversions, eigen_stl_containers, executive_smach, filters, gencpp, geneus, genlisp, genmsg, gennodejs, genpy, geometric_shapes, geometry, geometry_msgs, hls_lfcd_lds_driver, image_transport, joint_state_publisher, kdl_conversions, kdl_parser, message_filters, message_generation, message_runtime, mk, nav_msgs, nodelet, nodelet_core, nodelet_topic_tools, octomap (plain cmake), opencv3 (plain cmake), orocos_kdl (plain cmake), pluginlib, python_orocos_kdl (plain cmake), python_qt_binding, random_numbers, raspicam_node, resource_retriever, robot, robot_model, robot_state_publisher, ros, ros_base, ros_comm, ros_core, rosbag, rosbag_migration_rule, rosbag_storage, rosbash, rosboost_cfg, rosbuild, rosclean, rosconsole, rosconsole_bridge, roscpp, roscpp_core, roscpp_serialization, roscpp_traits, roscreate, rosgraph, rosgraph_msgs, roslang, roslaunch, roslib, roslint, roslisp, roslz4, rosmake, rosmaster, rosmsg, rosnode, rosout, rospack, rosparam, rospy, rosserial_msgs, rosserial_python, rosservice, rostest, rostime, rostopic, rosunit, roswtf, self_test, sensor_msgs, shape_msgs, smach, smach_msgs, smach_ros, smclib, std_msgs, std_srvs, stereo_msgs, tf, tf_conversions, tf2, tf2_kdl, tf2_msgs, tf2_py, tf2_ros, topic_tools, trajectory_msgs, turtlebot3_bringup, turtlebot3_msgs, urdf, urdf_parser_plugin, visualization_msgs, xacro, xmlrpcpp
{% endcapture %}
<div class="notice--info">{{ notice_03 | markdownify }}</div>


[appendix_raspi_cam]: /docs/en/platform/turtlebot3/appendix_raspi_cam/#raspberry-pi-camera
[pc_network_configuration]: /docs/en/platform/turtlebot3/pc_setup/#network-configuration
[network_configuration]: #5-network-configuration
[enable_ssh_server_in_raspberry_pi]: /docs/en/platform/turtlebot3/faq/#enable-ssh-server-in-raspberry-pi
