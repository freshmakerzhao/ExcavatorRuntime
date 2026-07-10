orin负责接受stm32传感器数据,整理后对外发送本体数据 machine_state_v1,代表“真机本体状态”

必需字段如下。

**1. header**

```
{
  "type": "machine_state_v1",
  "schema_version": "1.0",
  "seq": 12345, 
  "stamp_ms": 1780000000000,
  "source": "orin",
  "machine_id": "scale_excavator_v1"
}
```

说明：

```
seq			数据包序号,从0计数
stamp_ms	时间戳,单位ms
machine_id	防止接错机器
```

**2. safety**

```
"safety": {
  "estop": false,
  "stm32_alive": true,
  "sensor_valid": true,
  "control_enabled": false,
  "fault_flags": []
}
```

说明：

```
estop				急停状态,默认false,物理急停或严重危险设置为true,不再产生动作,第一优先级
stm32_alive=true   	STM32 通信状态,默认true,存活可用
sensor_valid=true  	这一帧本体传感器数据是否可信,true为可信,可以用来推理
control_enabled     是否允许PC动作真正进入控制链路,第一版默认false,人工确认进入自动控制后才设置为true
fault_flags         故障码,方便诊断,可设置一些内部定义的故障码方便实时观测:

stm32_timeout              	STM32 通信超时
sensor_invalid             	传感器整体无效
boom_sensor_invalid         boom 缸传感器异常
stick_sensor_invalid        stick/arm 缸传感器异常
bucket_sensor_invalid       bucket 缸传感器异常
swing_encoder_invalid       回转编码器异常
orin_over_temp              Orin 温度过高
low_voltage                 电压过低
action_timeout              PC 动作超时
estop_pressed               急停按下
calibration_missing         标定参数缺失
limit_reached               到达软件/硬件限位,应关闭control_enabled
```

**3. actuator_state：策略 observation **
ONNX 38 维 observation 需要的执行器状态。

```
"actuator_state": {
  "boom": {
    "position_m": 0.012,
    "velocity_mps": 0.001
  },
  "stick": {
    "position_m": -0.018,
    "velocity_mps": 0.000
  },
  "bucket": {
    "position_m": 0.006,
    "velocity_mps": -0.002
  },
  "swing": {
    "position_rad": 0.25,
    "velocity_rad_s": 0.01
  }
}
```

说明：

```
boom / stick / bucket 液压缸位置单位 m
boom / stick / bucket 液压缸速度单位 m/s
swing 回转角单位 rad 
swing 角速度单位 rad/s
不要归一化
stick 液压缸对应后续 joint_state 里的 arm 关节
```

**4. joint_state**

```
"joint_state": {
  "position_rad": {
    "swing": 0.25,
    "boom": 0.42,
    "arm": -0.75,
    "bucket": 0.36
  }
}
```

说明：

```
position_rad单位		rad 
第一阶段不发送 joint_state.velocity_rad_s, PC侧自动补0
```

**5. raw_sensor：第一阶段暂不发送**

```
第一阶段不发送 raw_sensor。
```

说明：

```
后续需要诊断时再扩展 raw_sensor, 例如 ADC 原始值、编码器 tick、STM32 本地时间等。
原始油缸电压
滤波前数据
传感器状态位
```


**6. 通信通道**

使用 UDP JSON通信, Orin 是 Ubuntu20/ROS1,PC 是 Ubuntu24/ROS2,直接用 UDP 可以避免 ROS1/ROS2 网络兼容问题。

推荐端口:

```
Orin -> PC   machine_state_v1   UDP 18081
PC   -> Orin policy_action      UDP 18082
```

当前固定配置:

```
PC_IP   = 192.168.2.127
ORIN_IP = 192.168.2.88

Orin 发送状态到: 192.168.2.127:18081
PC   发送动作到: 192.168.2.88:18082
```

说明:

```
192.168.2.127 是当前开发PC在局域网中的地址。
192.168.2.88 是当前 Orin 在局域网中的地址
```

Orin 侧需要支持配置:

```
ORIN_STATIC_IP=192.168.2.88   # 如果Orin已经固定为192.168.2.88,保持当前配置即可
ORIN_NETMASK=255.255.254.0
ORIN_GATEWAY=192.168.2.1
PC_STATE_HOST=192.168.2.127
PC_STATE_PORT=18081
ORIN_ACTION_BIND_PORT=18082
```

PC 侧需要支持配置:

```
PC_STATE_BIND_HOST=0.0.0.0
PC_STATE_BIND_PORT=18081
ORIN_ACTION_HOST=192.168.2.88
ORIN_ACTION_PORT=18082
```

**7. 10Hz 传感器频率下的约定**

当前 STM32 传感器频率是 10Hz,也就是每 100ms 一帧新本体数据。第一版建议 Orin 按“有新数据就发”的方式发送 `machine_state_v1`:

```
STM32 10Hz -> Orin 整理 -> Orin 10Hz UDP -> PC
```

关键约定:

```
seq                 每发一帧 machine_state_v1 加1
stamp_ms            Orin 整理并发出这一帧时的系统时间,单位ms
stm32_stamp_ms      STM32 本地tick或采样时间,只用于调试,不用和PC/Orin绝对对齐
sensor_valid=true   这一帧是新的、范围合理的、可用于FK和ONNX
sensor_valid=false  数据过旧、缺字段、超范围、跳变异常或STM32超时
```

PC 侧建议的超时阈值:

```
正常周期:       100ms
warn阈值:       超过200ms没有新状态包
fault阈值:      超过300ms没有新状态包
action有效期:   100ms,最多不超过150ms
```

**8. PC -> Orin 动作包**

PC 返回给 Orin 的动作包如下:

```
{
  "type": "policy_action",
  "schema_version": "1.0",
  "seq": 456,
  "stamp_ms": 1780000000100,
  "action_order": ["boom", "stick", "bucket", "swing"],
  "action": [0.0, 0.1, -0.1, 0.0],
  "action_type": "normalized_velocity_command",
  "valid_for_ms": 100
}
```

说明:

```
action_order    明确四个动作值的顺序,避免理解错位
action          归一化动作,范围[-1,1]
valid_for_ms    动作有效期,超过这个时间Orin必须丢弃并置零
```

Orin 执行动作前必须检查:

```
estop == false
control_enabled == true
sensor_valid == true
stm32_alive == true
policy_action 未超时
action 每一维都在 [-1,1]
```

不满足任一条件时:

```
不发送真实控制动作,或者发送全零安全动作。
```

**完整 Orin -> PC 包**

```
{
  "type": "machine_state_v1",
  "schema_version": "1.0",
  "seq": 12345,
  "stamp_ms": 1780000000000,
  "source": "orin",
  "machine_id": "scale_excavator_v1",

  "safety": {
    "estop": false,
    "stm32_alive": true,
    "sensor_valid": true,
    "control_enabled": false,
    "fault_flags": []
  },

  "actuator_state": {
    "boom": {"position_m": 0.012, "velocity_mps": 0.001},
    "stick": {"position_m": -0.018, "velocity_mps": 0.0},
    "bucket": {"position_m": 0.006, "velocity_mps": -0.002},
    "swing": {"position_rad": 0.25, "velocity_rad_s": 0.01}
  },

  "joint_state": {
    "position_rad": {
      "swing": 0.25,
      "boom": 0.42,
      "arm": -0.75,
      "bucket": 0.36
    }
  }
}
```
