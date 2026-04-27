# AMCL Fusion 调参手册

这份文档只讨论 `amcl_fusion` 节点本身的定位参数：粒子数、观测模型、运动模型、odometry 噪声、丢定位判定、全局搜索和在线调参。  
不讨论相机、HSV、ridge、白线提取、曝光、mask 生成等视觉处理链路。

当前主用 launch：

```bash
ros2 launch rcj_localization imx477_hsv_dt_fastmap_amcl.launch.py
```

## 1. AMCL Fusion 的工作流程

`amcl_fusion` 每个滤波周期大致做这几步：

1. 根据 yaw 和 odometry 做运动预测。
2. 把当前粒子与观测输入进行匹配，计算每个粒子的权重。
3. 根据权重发布 `/amcl_pose`、`/particlecloud` 等结果。
4. 按权重重采样，并按需要随机注入新粒子。
5. 如果开启 global search，根据粒子云集中程度和权重变化判断是否进入或退出全局搜索。

调参时要分清三类问题：

- 观测模型问题：粒子权重算得不合理。
- 运动模型问题：粒子预测走得太散、太紧或方向不对。
- 恢复策略问题：丢定位后不能恢复，或太容易误判丢定位。

## 2. 当前 AMCL 参数总览

### 2.1 位姿输入与基础开关

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `enable_localization` | `true` | 是否运行定位主循环 |
| `map_topic` | `/map` | 地图 topic |
| `mask_topic` | `/white_line_dt_ridge_filter_node/white_final_mask` | AMCL 使用的观测输入 topic |
| `use_weighted_mean_pose` | `true` | 发布加权平均位姿，而不是最高权重粒子 |
| `publish_debug_pointcloud` | `true` | 发布 AMCL 观测点调试点云 |
| `debug_pointcloud_topic` | `/field_line_observations_debug` | AMCL 观测点调试 topic |
| `publish_particle_weight_markers` | `true` | 发布按权重着色的粒子 Marker |
| `particle_weight_marker_topic` | `/particle_weights` | 粒子权重 Marker topic |
| `particle_weight_marker_scale` | `0.035` | Marker 点大小 |

### 2.2 Yaw 与 odometry

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `use_fake_yaw` | `true` | 是否使用固定 yaw |
| `fake_yaw_degrees` | `0.0` | 固定 yaw 值 |
| `yaw_zero_map_degrees` | `0.0` | yaw=0 对应地图方向的偏移 |
| `yaw_topic` | `/robot/yaw` | 真实 yaw topic |
| `use_stm32_gateway_odometry` | `true` | 是否通过 STM32 gateway 请求 odom 增量 |
| `stm32_command_service` | `/stm32/send_command` | STM32 service |
| `stm32_request_timeout_ms` | `50` | AMCL 等待一次 STM32 odom 响应的本地超时 |
| `stm32_enable_odometry_log` | `true` | 是否打印 STM32 odom 日志 |
| `odom_topic` | `/wheel_odometry` | 不使用 gateway 时的普通 odom topic |

如果机器人会转向，`use_fake_yaw=true` 通常只适合短时间静态或纯平移测试。真实 yaw 可用时，优先启动为：

```bash
use_fake_yaw:=false
```

### 2.3 观测模型

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `meters_per_pixel` | `0.0036` | AMCL 将局部观测转换成米制坐标的比例 |
| `forward_axis` | `v-` | 局部观测坐标中哪个轴是机器人前向 |
| `left_axis` | `u-` | 局部观测坐标中哪个轴是机器人左向 |
| `max_points` | `3000` | 每轮最多使用的观测点数 |
| `sigma_hit` | `0.10` | 观测点到地图特征的高斯宽度，单位米 |
| `off_map_penalty` | `1.0` | 观测点投到地图外时使用的惩罚距离，单位米 |
| `occupancy_threshold` | `50` | 地图栅格值大于该阈值时被当作占用特征 |
| `distance_transform_mask_size` | `5` | 距离变换 mask size，只支持 `3` 或 `5` |

`sigma_hit` 是观测模型最重要的参数。它越小，权重区分越强，但越怕噪声和比例误差；它越大，系统越宽容，但错误位姿也更容易拿到不错权重。

### 2.4 Fallback 随机扩散

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `noise_xy` | `0.05` | 没有可用 odom 时的位置扩散标准差，单位米 |
| `noise_theta` | `0.10` | 没有可用 odom 时的角度扩散标准差，单位弧度 |

在 `use_stm32_gateway_odometry=true` 且请求成功时，主运动噪声不是 `noise_xy` / `noise_theta`，而是 `odom_noise_*`。
这两个参数主要影响以下情况：

- STM32 service 未 ready。
- STM32 请求失败。
- STM32 请求超时。
- global search 状态下的随机扩散。
- 没有普通 odom 可用。

### 2.5 STM32 odometry 各向异性噪声

成功收到 STM32 odom 后，AMCL 会把运动增量转成机体系：

```text
dx_b, dy_b, dtheta
```

然后按下面的方差模型采样：

```text
sigma_x^2     = x_from_x * dx_b^2 + x_from_y * dy_b^2 + x_from_theta * dtheta^2 + x_bias
sigma_y^2     = y_from_x * dx_b^2 + y_from_y * dy_b^2 + y_from_theta * dtheta^2 + y_bias
sigma_theta^2 = theta_from_x * dx_b^2 + theta_from_y * dy_b^2 + theta_from_theta * dtheta^2 + theta_bias
```

| 参数 | 当前默认 | 主要影响 |
| --- | --- | --- |
| `odom_noise_x_from_x` | `0.08` | 前后运动造成的 x 方向不确定性 |
| `odom_noise_x_from_y` | `0.02` | 侧向运动造成的 x 方向不确定性 |
| `odom_noise_x_from_theta` | `0.0025` | 转向造成的 x 方向不确定性 |
| `odom_noise_x_bias` | `0.000064` | x 方向基础底噪 |
| `odom_noise_y_from_x` | `0.02` | 前后运动造成的 y 方向不确定性 |
| `odom_noise_y_from_y` | `0.16` | 侧向运动造成的 y 方向不确定性 |
| `odom_noise_y_from_theta` | `0.0049` | 转向造成的 y 方向不确定性 |
| `odom_noise_y_bias` | `0.000144` | y 方向基础底噪 |
| `odom_noise_theta_from_x` | `0.30` | 前后运动造成的角度不确定性 |
| `odom_noise_theta_from_y` | `0.60` | 侧向运动造成的角度不确定性 |
| `odom_noise_theta_from_theta` | `0.09` | 转向造成的角度不确定性 |
| `odom_noise_theta_bias` | `0.000304617` | 角度基础底噪 |

这些值越大，粒子云越散，越不相信 odom；越小，粒子云越紧，越相信 odom。

### 2.6 Lost 判定与 global search

| 参数 | 当前默认 | 作用 |
| --- | --- | --- |
| `use_random_search_when_unlocalized` | `false` | launch 层开关，传给 `enable_global_search` |
| `enable_global_search` | 同上 | 是否允许自动进入全局搜索 |
| `global_search_random_ratio` | `0.50` | global search 时强制随机注入比例 |
| `global_search_noise_xy` | `0.12` | global search 时位置扩散 |
| `global_search_noise_theta` | `0.10` | global search 时角度扩散 |
| `localized_xy_std_threshold` | `0.20` | 粒子云位置标准差低于该值才可能退出 global search |
| `localized_theta_std_threshold` | `0.35` | 粒子云角度标准差低于该值才可能退出 global search |
| `localized_min_updates` | `5` | 连续多少次收敛后退出 global search |
| `lost_alpha_ratio_threshold` | `0.45` | `alpha_fast / alpha_slow` 低于该值时可能判丢 |
| `lost_min_updates` | `3` | 连续多少次低 ratio 后进入 global search |
| `alpha_fast_rate` | `0.1` | 快速平均权重更新率 |
| `alpha_slow_rate` | `0.001` | 慢速平均权重更新率 |
| `random_injection_max_ratio` | `0.25` | 非强制随机注入比例上限 |

当前默认没有开启自动 global search。需要自动恢复时，启动时加：

```bash
use_random_search_when_unlocalized:=true
```

## 3. 推荐调参顺序

### 3.1 先确认 AMCL 输入坐标关系

先只看 AMCL 的地图、粒子云、AMCL 观测调试点。不要先动恢复参数。

优先调：

1. `meters_per_pixel`
2. `forward_axis`
3. `left_axis`
4. `yaw_zero_map_degrees`

判断方式：

| 现象 | 优先调 |
| --- | --- |
| 观测整体比地图大 | 减小 `meters_per_pixel` |
| 观测整体比地图小 | 增大 `meters_per_pixel` |
| 前后反 | 改 `forward_axis` 符号 |
| 左右反 | 改 `left_axis` 符号 |
| 像旋转了 90 度 | 交换 `forward_axis` / `left_axis` 使用的轴 |
| 机器人转向后整体角度错 | 调 `yaw_zero_map_degrees` 或检查 yaw 方向 |

如果这些关系不对，后面的 `sigma_hit`、`odom_noise_*`、`lost_*` 都会被误导。

### 3.2 再确认 yaw 与 odometry 是否可靠

如果机器人有转向，优先使用真实 yaw：

```bash
use_fake_yaw:=false
```

观察 STM32 odom 日志。如果经常 timeout 或失败，先提高 AMCL 本地等待时间：

```bash
ros2 param set /amcl_fusion stm32_request_timeout_ms 100
```

如果 STM32 odom 经常失败，`odom_noise_*` 不会解决问题，因为 AMCL 会走 fallback 扩散。

### 3.3 再调 `sigma_hit`

建议从当前 `0.10` 附近试：

```bash
ros2 param set /amcl_fusion sigma_hit 0.08
ros2 param set /amcl_fusion sigma_hit 0.10
ros2 param set /amcl_fusion sigma_hit 0.12
```

调参方向：

| 现象 | 调法 |
| --- | --- |
| 粒子一直很散、错误位置也能活 | 降低 `sigma_hit` |
| 轻微运动后权重崩、容易判丢 | 增大 `sigma_hit` |
| 位姿很抖、粒子来回跳 | 适当增大 `sigma_hit` |
| 收敛很慢、区分度不够 | 适当降低 `sigma_hit` |

常用范围：

```text
0.06 ~ 0.09：观测和地图很准
0.09 ~ 0.12：普通起手范围
0.12 ~ 0.18：观测噪声较大或比例略不准
```

### 3.4 再调 odometry 噪声

按动作分开测，不要混在一起：

1. 静止 10 秒
2. 前进 20cm
3. 后退 20cm
4. 侧移 20cm
5. 原地转 90 度

调参规则：

| 现象 | 优先调 |
| --- | --- |
| 静止也散 | 降低 `odom_noise_x_bias`、`odom_noise_y_bias`、`odom_noise_theta_bias` |
| 前进后沿前进方向散 | 降低 `odom_noise_x_from_x` |
| 前进后横向散 | 降低 `odom_noise_y_from_x` |
| 前进后角度散 | 降低 `odom_noise_theta_from_x` |
| 侧移后横向散 | 降低 `odom_noise_y_from_y` |
| 侧移后角度散 | 降低 `odom_noise_theta_from_y` |
| 原地转后角度散 | 降低 `odom_noise_theta_from_theta` |
| 原地转后位置散 | 降低 `odom_noise_x_from_theta`、`odom_noise_y_from_theta` |

如果粒子云中心也明显走错，不要只降低噪声。中心错通常是 odom 增量、yaw、坐标方向或比例的问题。降低噪声只会让错误更自信。

### 3.5 最后调 lost 与恢复

开启 global search：

```bash
ros2 param set /amcl_fusion enable_global_search true
```

如果一两帧坏观测就误判丢定位：

```bash
ros2 param set /amcl_fusion lost_alpha_ratio_threshold 0.30
ros2 param set /amcl_fusion lost_min_updates 5
ros2 param set /amcl_fusion alpha_fast_rate 0.05
ros2 param set /amcl_fusion random_injection_max_ratio 0.10
```

如果明显丢定位但很久不进入 global search：

```bash
ros2 param set /amcl_fusion lost_alpha_ratio_threshold 0.55
ros2 param set /amcl_fusion lost_min_updates 2
ros2 param set /amcl_fusion alpha_fast_rate 0.10
```

如果进入 global search 后恢复慢：

```bash
ros2 param set /amcl_fusion global_search_random_ratio 0.60
ros2 param set /amcl_fusion global_search_noise_xy 0.15
ros2 param set /amcl_fusion num_particles 1500
```

如果 global search 太乱：

```bash
ros2 param set /amcl_fusion global_search_random_ratio 0.30
ros2 param set /amcl_fusion global_search_noise_xy 0.08
```

## 4. 按问题现象调参

### 4.1 粒子云能跟着动，但越来越散

优先看：

1. STM32 odom 是否成功返回。
2. 是否在 fallback 扩散。
3. `odom_noise_*` 是否过大。
4. `sigma_hit` 是否过大。
5. `random_injection_max_ratio` 是否过大。

起手尝试：

```bash
ros2 param set /amcl_fusion random_injection_max_ratio 0.10
ros2 param set /amcl_fusion odom_noise_x_from_x 0.04
ros2 param set /amcl_fusion odom_noise_y_from_y 0.08
ros2 param set /amcl_fusion odom_noise_theta_from_x 0.15
ros2 param set /amcl_fusion odom_noise_theta_from_y 0.30
ros2 param set /amcl_fusion odom_noise_theta_from_theta 0.04
```

### 4.2 移动一段后像丢定位

优先看：

1. `use_fake_yaw` 是否仍为 `true`。
2. `stm32_request_timeout_ms` 是否太小。
3. `sigma_hit` 是否太小。
4. `enable_global_search` 是否关闭。
5. `lost_alpha_ratio_threshold` 是否太高。

起手尝试：

```bash
use_fake_yaw:=false
use_random_search_when_unlocalized:=true
stm32_request_timeout_ms:=100
```

如果是误判丢定位：

```bash
ros2 param set /amcl_fusion lost_alpha_ratio_threshold 0.30
ros2 param set /amcl_fusion lost_min_updates 5
ros2 param set /amcl_fusion alpha_fast_rate 0.05
```

如果是真的丢了但不恢复：

```bash
ros2 param set /amcl_fusion global_search_random_ratio 0.60
ros2 param set /amcl_fusion global_search_noise_xy 0.15
```

### 4.3 静止时粒子也扩散

优先看：

- STM32 odom 是否失败导致 fallback。
- `noise_xy` / `noise_theta` 是否太大。
- `odom_noise_*_bias` 是否太大。
- `sigma_hit` 是否太大，导致观测约束弱。

起手尝试：

```bash
ros2 param set /amcl_fusion noise_xy 0.03
ros2 param set /amcl_fusion noise_theta 0.05
ros2 param set /amcl_fusion odom_noise_x_bias 0.000025
ros2 param set /amcl_fusion odom_noise_y_bias 0.000049
ros2 param set /amcl_fusion odom_noise_theta_bias 0.000076
```

### 4.4 位姿跳动，但粒子云能收

优先看：

- `use_weighted_mean_pose`
- `random_injection_max_ratio`
- `sigma_hit`
- `odom_noise_*`

当前 `use_weighted_mean_pose=true`，已经倾向于平滑输出。继续可试：

```bash
ros2 param set /amcl_fusion random_injection_max_ratio 0.10
ros2 param set /amcl_fusion sigma_hit 0.12
```

### 4.5 丢定位后很久回不来

优先看：

- `enable_global_search`
- `global_search_random_ratio`
- `global_search_noise_xy`
- `num_particles`
- `sigma_hit`

起手尝试：

```bash
ros2 param set /amcl_fusion enable_global_search true
ros2 param set /amcl_fusion global_search_random_ratio 0.60
ros2 param set /amcl_fusion global_search_noise_xy 0.15
ros2 param set /amcl_fusion num_particles 1500
```

### 4.6 一两帧坏观测就大面积撒粒子

优先看：

- `lost_alpha_ratio_threshold`
- `lost_min_updates`
- `alpha_fast_rate`
- `random_injection_max_ratio`
- `global_search_random_ratio`

起手尝试：

```bash
ros2 param set /amcl_fusion lost_alpha_ratio_threshold 0.30
ros2 param set /amcl_fusion lost_min_updates 5
ros2 param set /amcl_fusion alpha_fast_rate 0.05
ros2 param set /amcl_fusion random_injection_max_ratio 0.10
ros2 param set /amcl_fusion global_search_random_ratio 0.30
```

### 4.7 CPU 占用高

优先看：

- `max_points`
- `num_particles`
- `filter_period_ms`

起手尝试：

```bash
ros2 param set /amcl_fusion max_points 2000
ros2 param set /amcl_fusion num_particles 700
ros2 param set /amcl_fusion filter_period_ms 100
```

## 5. Odom 噪声定量调法

如果你希望某个动作后的粒子扩散标准差是已知的，可以反推系数。

例子：前进 `0.10m`，希望 x 方向标准差约 `0.015m`，且 `odom_noise_x_bias=0.000064`。

```text
odom_noise_x_from_x = (0.015^2 - 0.000064) / 0.10^2
                    = 0.0161
```

当前 `odom_noise_x_from_x=0.08`，10cm 前进时 x 方向标准差大约是 `0.029m`。
如果你觉得 10cm 前进后 3cm 的预测扩散太大，可以把它降到 `0.02 ~ 0.04`。

每次只改 `20% ~ 50%`，并且一次只测一个动作。

## 6. 参数生效方式

在线可改：

```bash
ros2 param set /amcl_fusion sigma_hit 0.10
ros2 param set /amcl_fusion odom_noise_x_from_x 0.04
ros2 param set /amcl_fusion lost_alpha_ratio_threshold 0.30
```

立即生效：

- `meters_per_pixel`
- `forward_axis`
- `left_axis`
- `max_points`
- `use_weighted_mean_pose`
- `sigma_hit`
- `noise_xy`
- `noise_theta`
- `alpha_fast_rate`
- `alpha_slow_rate`
- `random_injection_max_ratio`
- `off_map_penalty`
- `odom_noise_*`
- `stm32_request_timeout_ms`
- `stm32_enable_odometry_log`
- `enable_global_search`
- `global_search_*`
- `localized_*`
- `lost_*`
- `filter_period_ms`

会重初始化粒子：

- `num_particles`
- `init_field_width`
- `init_field_height`

需要重启节点：

- `mask_topic`
- `map_topic`
- `yaw_topic`
- `use_fake_yaw`
- `fake_yaw_degrees`
- `yaw_zero_map_degrees`
- `odom_topic`
- `use_stm32_gateway_odometry`
- `stm32_command_service`
- `publish_debug_pointcloud`
- `publish_particle_weight_markers`
- `processing_time_topic`

地图距离场相关，通常需要重新收到地图后才完整体现：

- `occupancy_threshold`
- `distance_transform_mask_size`

## 7. 建议记录格式

每次调参都记录：

```text
启动参数：
在线修改参数：
测试动作：
粒子中心是否正确：
粒子云是否过宽：
是否误触发 global search：
丢定位后恢复时间：
CPU 是否可接受：
结论：
```

没有记录时，不要连续改三四个参数。AMCL 调参最容易出问题的地方不是参数不知道怎么改，而是改完以后不知道哪个参数造成了现象变化。
