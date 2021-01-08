---
title: 日本ロボット学会　ロボットの作り方 ～移動ロボットの基本とROSによるナビゲーション実習～
---

# ロボットの作り方 ～移動ロボットの基本とROSによるナビゲーション実習～

[日本ロボット学会 セミナー申し込みページ](https://www.rsj.or.jp/event/seminar/news/2020/s131.html)

- Table of contents
{:toc}

## 事前準備

下記を*必ず*{: style="color: red"}ご用意ください。

- 実習に利用するノート PC
  - Ubuntu Linux と ROS を事前にインストールしてください
  - インストール方法は以下を参考にしてください
    - [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)
  - ノート PC は USB type A ポートが2ポート以上あるもの、もしくは USB ハブをご用意ください
<--- - プログラムの事前ダウンロード
  - 下記のスクリプトをお使いいただければ、お使いの PC 環境にセミナーで作成するプログラム一式がダウンロードされます
  - ネットワーク接続ができない場合に備えて、下記手順で事前にダウンロードしておいてください
    1. `$ wget https://raw.githubusercontent.com/takahasi/ros_moveit_rsj_tutorial/gh-pages/download_contents.sh`
    1. `$ chmod +x download_contents.sh`
    1. `$ ./download_contents.sh`
- セミナーテキストの事前ダウンロード
  - 下記の URL にセミナーで使用するテキスト一式が PDF 形式で格納されています
  - ネットワーク接続ができない場合に備えて、事前にダウンロードしておいてください（紙でも配布する予定です）
    1. <https://github.com/takahasi/ros_moveit_rsj_tutorial/tree/gh-pages/pdf> 
--->

下記は*必要な方のみ*{: style="color: red"}ご用意ください。

- 移動ロボット（組立済み）
  - Robotis社が提供している[TurtleBot3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)を利用します
  - _注意_{: style="color: red"}：他のTurtleBotを使う場合はセミナーの内容を適宜読み替えて進めて頂く必要があります。
- ロボットが移動できるスペース

不明な点や、事前準備がうまく行かない点については、オーガナイザまでお問い合わせ下さい。

## スケジュール

### １日目（1/23(土) 10:00-16:30）

<--- |09:45-|受付開始| --->
|10:00-11:00|[講習環境の確認とROSの基本操作](ros_basics.html)|
講義１ 「ROS概論」<br>講師：Geoffrey BIGGS （TierⅣ）|
|11:00-12:00|移動ロボットの動作確認|
|12:00-13:00|昼休み|
|13:00-14:00|講義1 「ROSを用いた自律移動ロボットのシステム構築」<br>講師：原　祥尭 (千葉工業大学)|
|14:00-15:00|ROSを用いたマップ取得|
|15:00-16:30|ROS Navigationの利用|

### ２日目（1/24(日) 10:00-15:00）

<--- |09:00-|受付開始| --->
|10:00-11:00|講義2 「移動ロボットナビゲーション概論」<br>講師：上田　隆一 (千葉工業大学)|
|11:00-12:00|マップを利用したナビゲージョン操作|
|12:00-13:00|昼休み|
|13:00-14:30|他のセンサーの情報とロボットナビゲーションの統合|
|14:30-15:00|課題と質疑|

スケジュールは、演習の進行等に応じて変更する場合がありますのでご了承ください。

## 会場

オンラインでの実習（zoomを使用予定です）


## セミナーテキスト（準備中）
順次にテキストをアップロードする予定です。

1. [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)|

1. [Linux の基本操作](linux_basics.html)|
<---
1. [移動ロボットの基本動作](manipulator_check.html)|

1. [ROS の基本操作](ros_basics.html)|

1. [マニピュレータ制御](manipulators_and_moveit.html)|

1. [カメラの動作確認](camera_check.html)|

1. [画像処理](image_processing_and_opencv.html)|

1. [ROS の便利機能](ros_useful_stuff.html)|

1. [画像処理とマニピュレーションの組み合わせ](full_application.html)|
--->

## 参考情報

- [日本ロボット学会 第121回 ロボットの作り方 申し込みページ](https://www.rsj.or.jp/event/seminar/news/2020/s131.html)
- [ROS Japan UG （日本ユーザ会）](https://rosjp.connpass.com/)
- [ROS メッセージボード](https://discourse.ros.org/)
- [ROS Answers](http://answers.ros.org/)（日本語でも大丈夫です）
- [Programming Robots with ROS](http://shop.oreilly.com/product/0636920024736.do)
- [TurtleBot Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
