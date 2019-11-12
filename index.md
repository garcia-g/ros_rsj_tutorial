---
title: 日本ロボット学会　ロボットの作り方 ～ROSを使用した画像処理とマニピュレータ制御～
---

# ロボットの作り方 ～ROSを使用した画像処理とマニピュレータ制御～

[日本ロボット学会 セミナー申し込みページ](https://www.rsj.or.jp/event/seminar/news/2019/s121.html)

- Table of contents
{:toc}

## 事前準備

下記を*必ず*{: style="color: red"}ご用意ください。

- 実習に利用するノート PC
  - Ubuntu Linux と ROS を事前にインストールしてください
  - インストール方法は以下を参考にしてください
    - [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)
  - ノート PC は USB type A ポートが2ポート以上あるもの、もしくは USB ハブをご用意ください
- プログラムの事前ダウンロード
  - 下記のスクリプトをお使いいただければ、お使いの PC 環境にセミナーで作成するプログラム一式がダウンロードされます
  - ネットワーク接続ができない場合に備えて、下記手順で事前にダウンロードしておいてください
    1. `$ wget https://raw.githubusercontent.com/takahasi/ros_moveit_rsj_tutorial/gh-pages/download_contents.sh`
    1. `$ chmod +x download_contents.sh`
    1. `$ ./download_contents.sh`

下記は*必要な方のみ*{: style="color: red"}ご用意ください。マニピュレータに関しては、セミナー申込時にロボット購入希望としていただいた方に関しては主催者側で準備します。

- マニピュレータ
  - RT Robot Shopが提供している[ロボットアーム CRANE+](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1324&products_id=3626)を利用します
  - _注意_{: style="color: red"}：CRANE+ の電源は別売りです。[12Vおよび５Aが可能な電源](http://www.rt-shop.jp/index.php?main_page=product_info&cPath=1000_1012_1131&products_id=595)が必要です
- USBハブ
  - 本セミナーでは USB type-A のポートを2系統（マニピュレータ、カメラ）使用する予定です。
  - 使用予定のノート PC のポートが空いていないもしくは付いていない場合は USB ハブをご持参ください。
- モバイルルーターなどのインターネット接続機器
  - 会場でもインターネット接続環境を用意しますが、同時接続時は速度が遅くなる可能性があるため、可能であれば持参をお願いします。
  - 提供する無線LANのSSID、パスワードについては当日会場で連絡いたします。

下記は*主催者側で準備*{: style="color: red"}しますので、事前準備は不要です。セミナー受講料に含まれますので、セミナー後は各自お持ち帰りしていただく予定です。

- マニピュレータとPCをつなげる[USB延長ケーブル](https://www.amazon.co.jp/gp/product/B007STDLM0)
- カメラ
  - 一般的な USB 接続の Web カメラをご用意ください
  - 接続して PC が認識できれば基本的に利用可能です
  - 例：[サンワサプライ Web カメラ](https://www.sanwa.co.jp/product/syohin.asp?code=CMS-V41BK&cate=1)
- カメラ用三脚
  - 卓上であれば、60cm以上の高さ調節が可能な三脚
  - ウェブカメラを利用する場合、三脚の上部にウェブカメラが取り付けられること（ネジ、テープ等で）を確認してください
  - 例：[Velbon EX-Macro](https://www.amazon.co.jp/gp/product/B00DL5RP5Y)
- マニピュレータの把持対象物
  - カメラ認識のために単純な形状の明るい色の物体がおすすめです
  - 把持時にハンドの損傷等を防ぐために、柔らかい物体が望ましいです
  - 例：[激落ちくんスポンジ](https://www.amazon.co.jp/dp/B07J6534TN)
- カメラキャリブレーション用のチェッカーボード
  - カメラキャリブレーション用の白黒チェッカーボード
  - 下記の URL を印刷したものが利用できます（8x6、1マス2.45cm）
  - <http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=get&target=check-108.pdf>

不明な点や、事前準備がうまく行かない点については、オーガナイザまでお問い合わせ下さい。

## スケジュール

### １日目（11/16(土) 10:00-17:30）

|09:30-|開場・受付開始|
|10:00-11:00|講義１ 「ROS概論」<br>講師：Geoffrey BIGGS （TierⅣ）|
|11:00-11:30|[Linuxの基本操作](linux_basics.html)|
|11:30-12:00|[マニピュレータの動作確認](manipulator_check.html)|
|12:00-13:00|昼休み|
|13:00-14:00|講義２ 「MoveIt構造とアプリケーションの例」<br>講師：Felix von Drigalski (OMRON SINIC X)|
|14:00-15:00|[ROSの基本操作](ros_basics.html)|
|15:00-17:30|[マニピュレータ制御](manipulators_and_moveit.html)|

### ２日目（11/17(日) 10:00-17:00）

|09:00-|開場|
|10:00-10:15|[カメラの動作確認](camera_check.html)|
|10:15-12:00|[画像処理](image_processing_and_opencv.html)|
|12:00-13:00|昼休み|
|13:00-13:30|[ROSの便利機能](ros_useful_stuff.html)|
|13:30-16:30|[画像処理とマニピュレーションの組み合わせ](full_application.html)|
|16:30-17:00|クロージング|
|17:00-|片付け・解散|

スケジュールは、演習の進行等に応じて変更する場合がありますのでご了承ください。

## 会場

中央大学 後楽園キャンパス 2号館 製図室 2215・2221号室（東京都文京区春日1-13-27）

### 交通案内

- [アクセスマップ](/files/access.pdf)

### 昼食案内

- [ランチマップ](/files/lunch.pdf)


## セミナーテキスト

1. [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)|

1. [Linux の基本操作](linux_basics.html)|

1. [マニピュレータの動作確認](manipulator_check.html)|

1. [ROS の基本操作](ros_basics.html)|

1. [マニピュレータ制御](manipulators_and_moveit.html)|

1. [カメラの動作確認](camera_check.html)|

1. [画像処理](image_processing_and_opencv.html)|

1. [ROS の便利機能](ros_useful_stuff.html)|

1. [画像処理とマニピュレーションの組み合わせ](full_application.html)|

## 参考情報

- [日本ロボット学会 第121回 ロボットの作り方 申し込みページ](https://www.rsj.or.jp/event/seminar/news/2019/s121.html)
- [(過去開催)2017年度 RSJロボット工学セミナー第106回@中央大 実施時のトップページ](index_20171021.html)
- [(過去開催)2017年度 RSJロボット工学セミナー第106回＠つくば 実施時のトップページ](index_20170617.html)
- [ROS Japan UG （日本ユーザ会）](https://rosjp.connpass.com/)
- [ROS メッセージボード](https://discourse.ros.org/)
- [ROS Answers](http://answers.ros.org/)（日本語でも大丈夫です）
- [Programming Robots with ROS](http://shop.oreilly.com/product/0636920024736.do)
