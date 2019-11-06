---
title: カメラの動作確認
date: 2019-11-06
---

既存のROSパッケージを使用してカメラの基本動作を確認します。

- Table of contents
{:toc}


## 準備

1. 新しいワークスペースを作成します。

   ```shell
   $ mkdir -p ~/block_finder_ws/src/
   $ cd ~/block_finder_ws/src/
   $ catkin_init_workspace
   $ ls
   CMakeLists.txt
   ```

1. ROSパッケージ`v4l-utils`をインストールします。

   ```shell
   $ sudo apt-get install v4l-utils
   ```

1. ROSパッケージ`usb_cam`をインストールします。

   ```shell
   $ sudo apt-get install ros-kinetic-usb-cam
   ```

## 実行

はじめに Web カメラを接続せずに開始してください。

1. PC 内蔵のカメラ有無を確認します。<br>
ノートパソコンなどで、内蔵カメラがある場合はデバイス番号が表示されます。<br>
存在しない場合は「そのようなファイルやディレクトリはありません」などと表示されます。

   ```shell
   $ ls /dev/video*
   ```

1. WebカメラをUSBでパソコンに接続します。

1. Webカメラのデバイス番号を確認します。

   ```shell
   $ ls /dev/video*
   /dev/video0
   ```

   __各自の環境（ハードウェア）により、デバイス番号が変化します。例えば、ノートパソコンなどで内蔵カメラがある場合は/dev/video1などになります。__

1. Web カメラが対応している解像度などを確認します。<br>
（※下記ではデバイス番号が0の場合の例を示します。デバイス番号が0以外の場合は、オプション「d」の値を変更してください。）

   ```shell
   $ v4l2-ctl -d 0 --list-formats-ext
   ```

1. Web カメラが取得している画像を表示します。

   1. デバイス番号が0の場合

      ```shell
      $ roslaunch usb_cam usb_cam-test.launch
      ```

   1. デバイス番号が0以外の場合

      ```shell
      $ roscd usb_cam/launch
      $ cp usb_cam-test.launch ~/block_finder_ws/usb_cam-test_rsj.launch
      $ cd ~/block_finder_ws
      $ gedit usb_cam-test_rsj.launch
      # video_deviceを/dev/video1などに変更し、上書き保存する。
      $ roslaunch usb_cam-test_rsj.launch
      ```

1. 次のようなユーザーインターフェースが表示されたら、正しく動作しています。<br>
このユーザーインターフェースのボタンを押すと画像を保存することができます。

   ![usb_cam](images/usb_cam.png)

1. エラーが表示されずに画像が保存されていることが確認できたら、『Ctrl』キー＋『c』キーで終了します。
