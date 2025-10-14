# robomas_driver

## 実行コマンド
```bash
ros2 run robomas_driver robomaster_ros2_ctrl
```



## 動作確認用

### モードの切り替え
- 0 : 電流制御モード　　
- 1 : 速度制御モード　　
- 2 : 位置制御モード


```bash
ros2 topic pub -1 /rm_cmd_array robomas_driver/MotorCmdArray "
cmds:
- {id: 1, type: 'M3508', mode: 2, value: 50.0}      # 位置 50.0 rev
- {id: 2, type: 'M3508', mode: 1, value: 1200.0}    # 速度 1200 rpm
- {id: 3, type: 'M3508', mode: 0, value: 1.5}       # 電流 1.5 A
- {id: 6, type: 'M2006', mode: 2, value: 100.0}     # 位置 50.0 rev
- {id: 8, type: 'M2006', mode: 1, value: 500.0}     # 速度 500 rpm
"
```
