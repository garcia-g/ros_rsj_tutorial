---
title: Ubuntu LinuxとROSのインストール
date: 2021-01-12
---

- Table of contents
{:toc}

本セミナーで使用する開発環境として Ubuntu Linux とその上で動作する ROS を利用します。<br>
本ページでは Ubuntu Linux と ROS のインストール方法を紹介します。

## 用意するもの

- ノート型パソコンあるいはデスクトップPC
-- *本手順によりパソコンの既存のOS（Windows等）及び保存されているデータやソフトウェアは完全に削除されます。<br>
予めにバックアップを行ってください。*{: style="color: red"}
- 容量 4GB 以上の空 USB メモリ
- インターネット接続
- モニター１台
- キーボード
- HDMIケーブル
- マウス
- 容量 4GB 以上の空 SDカード

## 手順（リモートPCセットアップ）

### Ubuntu Linux のダウンロード

1. 下記URLから Ubuntu Linux のインストールイメージをダウンロードします

   本セミナーでは Ubuntu 16.04 Xenial 64bit Desktop 版を推奨します。<br>
   中級者以上であれば、Ubuntu 18.04 Bionic を使用することもできますが、セミナーの内容を適宜読み替えて進めて頂く必要があります。

   [最新バージョン](https://www.ubuntu.com/download/desktopo)

   [Ubuntu 18.04 Bionic](http://releases.ubuntu.com/bionic/)

   [Ubuntu 16.04 Xenial](http://releases.ubuntu.com/xenial/)

   ![Ubuntu ダウンロード](/images/ubuntu_download_1.png)

1. Ubuntu をメンテナンスしている Canonical に寄付することも可能ですが、必須ではありません

   ![Ubuntu ダウンロード](/images/ubuntu_download_2.png)

1. 以下の画面になったら、ダウンロードが自動的に始まります

   ![Ubuntu ダウンロード](/images/ubuntu_download_3.png)

### Live USB の作成

1. 下記 URL から、Live USB 作成ソフトをダウンロードします。
   - Windows、Mac OS Xの場合：<https://unetbootin.github.io/>

     ページ中の、Live USB 作成に使用しているPCのOSを選択してください。

     ![UNetbootinダウンロード](/images/unetbootin_download.png)

   - Linux (Debian/Ubuntu) の場合: 下記コマンドを実行

     ```shell
     $ sudo apt-get install unetbootin
     ```

1. 誤って必要なデータを削除してしまうのを防ぐため、使用しない USB メモリや、メモリーカードを取り外し、使用する USB メモリのみを接続します。<br>
使用する USB メモリは、ファイルが入っていない空の状態にして下さい。

1. Live USB を作成する PC と Ubuntu をインストール する PC はそれぞれ別でも構いません。

1. ダウンロードした unetbootin-windows-???.exe（Windows の場合）を実行します。<br>
   下記、「Windows によって PC が保護されました」画面が現れた場合は、「実行」ボタンをクリックしてください。

   ![Windows UAC](/images/windows_idiot_screen.png)

   また、下記のユーザアカウント制御画面が現れた場合、「はい」をクリックしてください。

   ![Windows UAC](/images/windows_uac.png)

1. UNetbootin の画面で、「ディスクイメージ」を選択し、「…」ボタンをクリックして先ほどダウンロードした`ubuntu-xxxx.iso` ファイルを選択します。<br>
(xxxx はダウンロードしたファイル名に合わせて変更ください)

  また、「スペースは、リブートしてもファイルを維持するために使用」欄に「4096」と入力し、「ドライブ」欄で、使用する USB メモリのドライブ名を選択します。<br>
  内容を確認後、「OK」をクリックしてください。

   ![UNetbootin process 1](/images/unetbootin_setting.png)

   書き込み完了までしばらく待機します。<br>
   USB2.0 の場合10分以上、書き込み速度の遅いメモリだと30分程度かかる場合があります。<br>
   下記の「永続性を設定する」画面で「応答なし」と表示される場合がありますが、正常に動作していますので、そのまま待機してください。

   ![UNetbootin process 2](/images/unetbootin_freeze.png)

   下記画面が表示されれば、「Live USB」の作成は完了です。終了をクリックして下さい。

   ![UNetbootin process 3](/images/unetbootin_end.png)

1. Live USBから起動するためのBIOSの設定を行います。

   セミナーで使用する PC の電源を切り、下記の手順で作成した Live USB を接続した状態で起動します。<br>
   起動時に、BIOS 設定画面に入ります。*PC のメーカー毎に BIOS への入り方が異なります*{: style="color: red"}ので、マニュアル等で確認してください。<br>
   下記の図は Acer での BIOS 設定画面の例です。

   ![BIOS 1](/images/acer_boot.png)

   BIOS 設定画面に入ったら、起動順（Boot order, Boot priority）の設定で、USB メモリが最優先になるように設定します。 （表示は使用している PC および USB メモリのメーカーによって異なります）

   ![BIOS 2](/images/acer_boot_order1.png)

   ![BIOS 3](/images/acer_boot_order2.png)

   設定を保存して再起動します。

   ![BIOS 4](/images/acer_boot_save.png)

### Ubuntu Linux のインストール

1. Live USB をパソコンに接続し、パソコンの電源を入れます。

1. 以下の画面が表示されます。言語を選択してください。

   ![Ubuntu install 1](/images/ubuntu_install_1.png)

1. 「Ubuntu をインストール」を選択しインストール手順を開始します。<br>
画面に出る説明に従ってインストール手順を続いてください。

   ![Ubuntu install 2](/images/ubuntu_install_2.png)

1. 以下の画面に届いたら、「ディスクを削除して Ubuntu をインストールする」を選択してください。

   ![Ubuntu install 3](/images/ubuntu_install_3.png)

1. インストール後、LiveUSB を外してパソコンを再起動すると以下の画面が現れます。<br>
これで Ubuntu Linux のインストールが完了です。

   ![Ubuntu install 4](/images/ubuntu_install_4.png)


### ROS ベースパッケージのインストール

![](/images/turtlebot3/remote_pc_and_turtlebot.png)

**警告**: この章の内容は、TurtleBot3を制御する`リモートPC`(デスクトップまたは、ラップトップPC)に対応しています。 この手順は、TurtleBot3で実施しないでください。
{: .notice--warning}

下記のスクリプトを使用すると、ROS1のインストール手順を簡略化できます。
ターミナルウィンドウでこのスクリプトを実行します。ターミナルアプリケーションは、画面の左上隅にあるUbuntu検索アイコンから起動できます。もしくは、ターミナルのショートカットキー(`Ctrl`-`Alt`-`t`)を使用して起動できます。 ROS1をインストールした後、リモートPCを再起動してください。

```shell
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh
$ chmod 755 ./install_ros_kinetic.sh 
$ bash ./install_ros_kinetic.sh
```

**注釈**: インストールされるパッケージを確認するには、このリンクを確認してください。[install_ros_kinetic.sh](https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh)
{: .notice--info}

{% capture info_01 %}
**注釈**:  
 - ROBOTISのROSパッケージはMelodic Moreniaをサポートしていますが、TurtleBot3にはROS Kinetic Kameを使用することを推奨します。
 - ROSをMelodic Moreniaにアップグレードする場合は、サードパーティのROSパッケージが完全にサポートされていることを確認してください。
{% endcapture %}
<div class ="notice--info">{{info_01 | markdownify}}</div>

### ROS1 依存パッケージのインストール

リモートPCにROS1依存パッケージをインストールする手順です。

```shell
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy \
  ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch \
  ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python \
  ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs \
  ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro \
  ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping \
  ros-kinetic-navigation ros-kinetic-interactive-markers
```

リモートPCにTurtleBot3を制御するための依存パッケージをインストールする手順です。

```shell
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

`catkin_make`コマンドがエラー無しで完了した場合、TurtleBot3の準備は完了です。

### ネットワーク構成

![](/images/turtlebot3/network_configuration.png)

ROS1では、TurtleBot PCとリモートPCの間で通信をするためにIPアドレスが必要です。 リモートPCとTurtleBot PCは、同じwifiルーターに接続する必要があります。

リモートPCのターミナルウィンドウで次のコマンドを入力し、リモートPCのIPアドレスを確認します。

```shell
$ ifconfig
```

長方形の赤い枠で囲っている文字列が、`リモートPC`のIPアドレスです。

![](/images/turtlebot3/network_configuration2.png)

以下のコマンドを入力します。

```shell
$ nano ~/.bashrc
```

`Alt + /`を入力するとファイルの最終行へ移動します。

`ROS_MASTER_URI`と`ROS_HOSTNAME`の`localhost`のIPアドレスを、上記のターミナルウィンドウから取得したIPアドレスに変更します。


![](/images/turtlebot3/network_configuration3.png)

次に、以下のコマンドでbashrcを実行します。

```shell
$ source ~/.bashrc
```

以上で、開発環境の構築が完了しました。

## 手順（TurtleBotメインコンピューター：Raspberry Pi 3 セットアップ）

準備中

以上で、開発環境の構築が完了しました。
