---
title: 日本ロボット学会　ロボットの作り方 ～ROSを使用した画像処理とマニピュレータ制御～
---

# ロボットの作り方 ～ROSを使用した画像処理とマニピュレータ制御～

## 事前準備

下記を*必ず*{: style="color: red"}ご用意ください。

- 実習に利用するノート PC
  - Ubuntu Linux と ROS を事前にインストールしてください
  - インストール方法は以下の[事前準備](#事前準備)をご参考ください
  - ノート PC は USB ポートが2ポート以上あるもの、もしくは USB ハブをご用意ください


下記は*必要な方のみ*{: style="color: red"}ご用意ください。マニピュレータに関しては、セミナー申込時にロボット購入希望としていただいた方に関しては主催者側で準備します。

- マニピュレータ
  - RT Robot Shopが提供している[ロボットアーム CRANE+](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1324&products_id=3626)を利用します
  - _注意_{: style="color: red"}：CRANE+ の電源は別売りです。[12Vおよび５Aが可能な電源](http://www.rt-shop.jp/index.php?main_page=product_info&cPath=1000_1012_1131&products_id=595)が必要です

- USBハブ
  - 本セミナーでは USB type-A のポートを2系統（マニピュレータ、カメラ）使用する予定です。使用予定のノート PC のポートが空いていないもしくは付いていない場合は USB ハブをご持参ください。

下記は*主催者側で準備*{: style="color: red"}しますので、事前準備は不要です。セミナー受講料に含まれますので、セミナー後は各自お持ち帰りしていただく予定です。

- マニピュレータとPCをつなげる[USB延長ケーブル](https://www.amazon.co.jp/gp/product/B007STDLM0/ref=oh_aui_detailpage_o02_s00)
- カメラ
  - 一般的な USB 接続の Web カメラをご用意ください
  - 接続して PC が認識できれば基本的に利用可能です
  - 例：[エレコム Web カメラ](https://www.amazon.co.jp/gp/product/B00UZNLIBW/ref=oh_aui_detailpage_o03_s00)
- カメラ用三脚
  - 卓上であれば、60cm以上の高さ調節が可能な三脚
  - ウェブカメラを利用する場合、三脚の上部にウェブカメラが取り付けられること（ネジ、テープ等で）を確認してください
  - 例：[Velbon EX-Macro](https://www.amazon.co.jp/gp/product/B00DL5RP5Y/ref=oh_aui_detailpage_o03_s00)
- マニピュレータが持つ物品
  - カメラ認識のために単純な明るい色がおすすめです
  - 把持時にハンドの損傷等を防ぐために、柔らかい物体が望ましいです
  - 例：[激落ちのスポンジ](https://www.amazon.co.jp/gp/product/B005ZETITK/ref=oh_aui_detailpage_o00_s00)

## PC 環境の構築

- [Ubuntu LinuxとROSのインストール](linux_and_ros_install.html)

## セミナーテキスト

1. [Linux の基本操作](linux_basics.html)|

1. [講習環境の整備とマニピュレータの動作確認](manipulator_check.html)|

1. [ROS の基本操作](ros_basics.html)|

1. [マニピュレータ制御](manipulators_and_moveit.html)|

1. [カメラの動作確認](camera_check.html)|

1. [画像処理](image_processing_and_opencv.html)|

1. [ROS の便利機能](ros_useful_stuff.html)|

1. [画像処理とマニピュレーションの組み合わせ](full_application.html)|

## 参考情報

- [(過去開催)2017年度 RSJロボット工学セミナー第106回@中央大 実施時のトップページ](index_20171021.html)
- [(過去開催)2017年度 RSJロボット工学セミナー第106回＠つくば 実施時のトップページ](index_20170617.html)
- [ROS Japan UG （日本ユーザ会）](https://rosjp.connpass.com/)
- [ROS メッセージボード](https://discourse.ros.org/)
- [ROS Answers](http://answers.ros.org/)（日本語でも大丈夫です）
- [Programming Robots with ROS](http://shop.oreilly.com/product/0636920024736.do)
