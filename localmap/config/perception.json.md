# `perception.json` 配置说明

这是 AiryLidar 感知栈的唯一生产 profile。所有空间量使用右手
`machine_root_ros`：`+X` 挖掘机前方、`+Y` 左侧、`+Z` 向上，单位均为米。

标准 JSON 不支持注释，因此说明放在与配置同目录的本文件；不要向 JSON 添加
`//`、`/* ... */` 或 `_comment` 字段，严格加载器会拒绝它们。

## 顶层

| 字段 | 含义 |
| --- | --- |
| `schema` | 配置格式版本，加载器据此拒绝不兼容文件。 |
| `profile_id` | 这份生产 profile 的稳定标识。 |
| `expected_frame` | 所有感知、LocalMap、规划输入必须使用的唯一 ROS 根，固定为 `machine_root_ros`。 |
| `inputs` | 启动感知链所需的外部配置。 |
| `outputs` | 实时 JSON 产物和日志目录。 |
| `topics` | ROS 输入、输出 topic 契约。 |
| `local_map` | 从点云生成 LocalMap JSON 的范围与节流频率。 |
| `octomap` | `octomap_server` 的体素建图参数与输入裁剪。 |

## `inputs`

| 字段 | 含义 |
| --- | --- |
| `rslidar_config` | RoboSense 驱动配置。网络、雷达型号等不在本 profile 重复。 |
| `extrinsics` | `rslidar -> machine_root_ros` 外参。当前文件为由历史 Unity 外参重表达的版本，不能当作新的现场测量。 |
| `targets` | dig/dump 目标；其 `frame_id` 必须为 `machine_root_ros`。 |
| `bucket_tip_bridge` | FK Bucket Tip 到规划 Bucket Tip 的同手性 bridge。当前为 identity，仅统一 topic 与 JSON 生命周期，不做镜像转换。 |

## `outputs`

| 字段 | 含义 |
| --- | --- |
| `live_local_map` | 实时 LocalMap JSON，供一次规划读取。 |
| `live_bucket_tip` | 实时 Bucket Tip JSON，来自 FK bridge，带 FK 源时间。 |
| `log_dir` | 感知子进程日志目录。 |

## `topics`

| 字段 | 含义 |
| --- | --- |
| `raw_cloud` | 雷达驱动发布的原始 `rslidar` 点云。 |
| `machine_cloud` | 已变换到 `machine_root_ros` 的点云；RViz 和 LocalMap 使用此话题。 |
| `octomap_cells` | OctoMap 占据方块的 `MarkerArray` topic。 |
| `bucket_tip_fk` | FK 直接发布的、带关节源时间的右手 Bucket Tip Pose。 |
| `bucket_tip_machine_root` | bridge 后供规划/RViz 使用的 Bucket Tip Pose。 |

## `local_map`

`bounds` 顺序固定为 `[x_min, x_max, y_min, y_max, z_min, z_max]`。
它是生成 LocalMap JSON 前的点云裁剪盒，也是规划器可见地图的外边界。

当前值 `[-0.5, 4.0, -3.0, 1.5, -0.7, 1.2]` 表示：

```text
前后 X：-0.5 .. 4.0 m
左右 Y：-3.0 .. 1.5 m
高度 Z：-0.7 .. 1.2 m
```

`write_every` 是每多少帧写一次 `live_local_map`；`publish_every` 是每多少帧发布一次
LocalMap JSON topic。增大可减少 I/O，代价是规划输入更新变慢。

## `octomap`

| 字段 | 含义 |
| --- | --- |
| `resolution_m` | 体素边长；`0.05` 代表 5 cm。更小更细，但内存和计算量更高。 |
| `max_range_m` | 雷达点到传感器超过此距离时不积分进 OctoMap。 |
| `filter_ground_plane` | 是否让 OctoMap 尝试移除地面。当前为 `false`，便于先验证坐标与点云一致性。 |
| `reset_interval_s` | 周期清空 OctoMap 的秒数；调试时避免历史残影，设为 `0` 则不自动清空。 |
| `crop_bounds` | 输入 OctoMap 前的裁剪盒，顺序同 `local_map.bounds`。应始终位于 `local_map.bounds` 内。 |

当前 `crop_bounds` 与 `local_map.bounds` 相同，意味着 LocalMap 与 OctoMap 观察同一工作空间。
若以后为了性能缩小 OctoMap 范围，必须保留 `crop_bounds ⊆ local_map.bounds`，并重新确认
规划不会把轨迹穿过未建图区域。

## 修改规则

1. 先在 RViz 与 `inspect_live_cloud_geometry.py` 验证轴方向和高度轴，再改范围。
2. 一次只改一个参数组；改完重启感知栈并保存对应日志/截图到 `EvaluationReport/`。
3. 外参、Bucket Tip bridge、几何和关节零位不是 OctoMap 调参，不能同时修改。
4. 不要用 shell 环境变量覆盖本文件的坐标语义；生产启动脚本会从本 profile 加载它们。
