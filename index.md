---
title: 日本ロボット学会　ロボットの作り方 ～移動ロボットの基本とROSによるナビゲーション実習～
---

# ロボットの作り方 ～移動ロボットの基本とROSによるナビゲーション実習～

[日本ロボット学会 セミナー申し込みページ](https://www.rsj.or.jp/event/seminar/news/2020/s131.html)

- Table of contents
{:toc}

## 事前準備

下記を<span style="color:red">*必ず*</span>ご用意ください。

- 実習に利用するノート PC
  - Ubuntu Linux と ROS を事前にインストールしてください
  - インストール方法は以下を参考にしてください
    - [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)

下記は<span style="color:red">*必要な方のみ*</span>ご用意ください。

- 移動ロボット（組立済み）
  - Robotis社が提供している[TurtleBot3 burger](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)を利用します
  - <span style="color:red">_注意_</span>：他のTurtleBotを使う場合はセミナーの内容を適宜読み替えて進めて頂く必要があります。
- ロボットが移動できるスペース

不明な点や、事前準備がうまく行かない点については、オーガナイザまでお問い合わせ下さい。

## スケジュール

### １日目（1/23(土) 10:00-16:30）

|10:00-10:30|セミナーの進行につきまして|
|10:30-12:00|[講習環境の確認](linux_basics.html)と[ROSの基本操作](ros_basics.html)|
|12:00-13:00|昼休み|
|13:00-14:00|講義1 [「ROSを用いた自律移動ロボットのシステム構築」](/Hara-sensei_210123.pdf)<br>講師：原　祥尭 (千葉工業大学)|
|14:00-15:00|[移動ロボットの動作確認](turtlebot-basics.html)|
|15:00-16:30|[ROSを用いたマップ取得](slam-basics.html)|


<!--|15:00-16:30|[ROS Navigationの利用](ros-navigation.html)|-->

### ２日目（1/24(日) 10:00-15:30）

|10:00-11:00|講義2 「移動ロボットナビゲーション概論」<br>講師：上田　隆一 (千葉工業大学)|
|11:00-12:00|マップを利用したナビゲージョン操作１|
|12:00-13:00|昼休み|
|13:00-13:30|マップを利用したナビゲージョン操作２|
|13:30-15:00|障害物認識と回避|
|15:00-15:30|課題と質疑|

スケジュールは、演習の進行等に応じて変更する場合がありますのでご了承ください。

## 会場

オンラインでの実習（zoomを使用予定です）


## セミナーテキスト（準備中）
順次にテキストをアップロードする予定です。

1. [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)

1. [Linux の基本操作](linux_basics.html)

1. [ROSの基本操作](ros_basics.html)

1. [Turtlebot3の基本操作](turtlebot-basics.html)

1. [ROSを用いたマップ取得](slam-basics.html)

1. [マップを利用したナビゲージョン操作](map-navigation.html)

1. [障害物認識と回避](obstacle-detection.html)


## 参考情報

- [日本ロボット学会 第121回 ロボットの作り方 申し込みページ](https://www.rsj.or.jp/event/seminar/news/2020/s131.html)
- [ROS Japan UG （日本ユーザ会）](https://rosjp.connpass.com/)
- [ROS メッセージボード](https://discourse.ros.org/)
- [ROS Answers](http://answers.ros.org/)（日本語でも大丈夫です）
- [Programming Robots with ROS](http://shop.oreilly.com/product/0636920024736.do)
- [TurtleBot Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)