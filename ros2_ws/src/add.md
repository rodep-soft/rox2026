# 追加実装メモ

## L2 + Options中の操作制限

- L2 + Optionsで最大開放へ移動している間、および最大開放状態が解除されるまで、以下以外の操作を無効にする。
  - belt
  - positionの最大開放位置に関する操作
  - mecanumのangular.z

## ドリブル操作topicの整理

- `/dribble/mode`は使用しない。
- 以下の`std_msgs/msg/Bool` topicで扱う。
  - `/game2_command`
  - `/intake_and_shoot`
