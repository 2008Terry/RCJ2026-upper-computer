# AMCL 调参手册（Topdown PF V2）

这份手册主要讨论 `topdown_pf_localization_node_v2` 和 `particle_filter_v2` 里**真实存在**的 PF 参数，不扩展到标准激光 AMCL 的参数体系。

补充说明：

- `amcl_fusion` 现在支持通过 STM32 gateway 的 `cmd_request` 获取异步里程计增量。
- 当 `amcl_fusion` 开启 `use_stm32_gateway_odometry=true` 时，`noise_xy` / `noise_theta` 只用于“没有可用 odom 时的随机扩散 fallback”。
- 此时成功 odom 分支使用的是 `odom_noise_*` 12 参数各向异性噪声模型，见下面新增的小节。

## 1. 调参顺序

建议按下面顺序调，不要一上来就同时改很多参数：

1. 先调几何映射参数：`meters_per_pixel`、`forward_axis`、`left_axis`
2. 再调观测模型：`max_points`、`sigma_hit`、`off_map_penalty`
3. 再调运动模型：`noise_xy`、`noise_theta`
4. 再调恢复参数：`alpha_fast_rate`、`alpha_slow_rate`、`random_injection_max_ratio`
5. 最后调性能和稳定性：`num_particles`、`filter_period_ms`

如果 `/field_line_observations_debug` 和地图本身都对不齐，先不要碰 PF 参数，先把几何映射调对。

## 2. 参数分组与作用

### 2.1 几何映射参数

- `meters_per_pixel`
  - 顶视图每个像素对应多少米
  - 太大：点云整体放大，观测会落到地图线外
  - 太小：点云整体缩小，观测会挤在地图中心附近

- `forward_axis`
  - 图像哪个轴对应机器人前向
  - 取值：`u+`、`u-`、`v+`、`v-`

- `left_axis`
  - 图像哪个轴对应机器人左向
  - 取值：`u+`、`u-`、`v+`、`v-`

### 2.2 观测模型参数

- `max_points`
  - 每帧最多使用多少个观测点
  - 更大：匹配信息更多，但更耗时
  - 更小：更快，但观测信息变少
  - V2 已改成平均 log-likelihood，不会像旧版那样因为点数大而特别容易整体下溢，但点数过大仍然会拖慢运行

- `sigma_hit`
  - 观测高斯宽度，单位米
  - 更大：更宽容，点和地图线偏一点也还能接受
  - 更小：更挑剔，区分度更强，但更怕噪声和映射误差

- `off_map_penalty`
  - 点投影到地图外时使用的默认距离惩罚，单位米
  - 更大：强烈惩罚跑出地图边界的粒子
  - 更小：对出界粒子更宽容

- `occupancy_threshold`
  - 地图栅格值大于这个阈值时，视为“线”
  - 范围通常在 `0~100`
  - 调大：只有更“黑”的格子才算线
  - 调小：线更宽，但地图辨识度可能下降

- `distance_transform_mask_size`
  - OpenCV 距离变换的 mask size，当前建议 `3` 或 `5`
  - `5` 通常更平滑一些，也是当前默认值

### 2.3 运动模型参数

- `noise_xy`
  - 每轮预测时位置随机游走标准差，单位米
  - 更大：探索性更强，但粒子更抖
  - 更小：更稳，但更容易卡在局部错误位置

- `noise_theta`
  - 每轮预测时朝向随机游走标准差，单位弧度
  - 更大：允许对 yaw 的误差更宽容
  - 更小：更信任 yaw 输入

对 `amcl_fusion + STM32 gateway odometry` 这条链路：

- `noise_xy`
  - 只在 service 未就绪、callback 未返回、回包失败、或本地 watchdog 超时时生效
  - 它不再描述“成功 odom 更新时的主运动噪声”

- `noise_theta`
  - 同样只作用于 fallback 扩散分支
  - 它不再是成功 odom 分支里的主 heading 方差

### 2.3.1 Gateway Odom 各向异性噪声参数（`amcl_fusion`）

`amcl_fusion` 的成功 odom 分支把 STM32 返回的全局增量先转成机体系增量：

- `dx_b`
- `dy_b`
- `dtheta`

然后按下面的方差模型采样：

- `σx² = ax1*dx_b² + ax2*dy_b² + ax3*dtheta² + ax0`
- `σy² = ay1*dx_b² + ay2*dy_b² + ay3*dtheta² + ay0`
- `σθ² = aθ1*dx_b² + aθ2*dy_b² + aθ3*dtheta² + aθ0`

参数映射如下：

- `odom_noise_x_from_x` = `ax1`
- `odom_noise_x_from_y` = `ax2`
- `odom_noise_x_from_theta` = `ax3`
- `odom_noise_x_bias` = `ax0`
- `odom_noise_y_from_x` = `ay1`
- `odom_noise_y_from_y` = `ay2`
- `odom_noise_y_from_theta` = `ay3`
- `odom_noise_y_bias` = `ay0`
- `odom_noise_theta_from_x` = `aθ1`
- `odom_noise_theta_from_y` = `aθ2`
- `odom_noise_theta_from_theta` = `aθ3`
- `odom_noise_theta_bias` = `aθ0`

STM32 `cmd_request` 返回值的约定：

- `dx`、`dy`：厘米，map 全局系
- `dtheta`：角度，节点内部会转成弧度并归一化

推荐起手值：

- `odom_noise_x_from_x=0.08`
- `odom_noise_x_from_y=0.02`
- `odom_noise_x_from_theta=0.0025`
- `odom_noise_x_bias=0.000064`
- `odom_noise_y_from_x=0.02`
- `odom_noise_y_from_y=0.16`
- `odom_noise_y_from_theta=0.0049`
- `odom_noise_y_bias=0.000144`
- `odom_noise_theta_from_x=0.30`
- `odom_noise_theta_from_y=0.60`
- `odom_noise_theta_from_theta=0.09`
- `odom_noise_theta_bias=0.000304617`

调参直觉：

- 前进时位置太乐观，增大 `odom_noise_x_from_x`
- 侧移时明显更飘，优先增大 `odom_noise_y_from_y`
- 原地转会把位置带乱，增大 `odom_noise_x_from_theta` / `odom_noise_y_from_theta`
- 平移时朝向也容易歪，增大 `odom_noise_theta_from_x` / `odom_noise_theta_from_y`
- 静止也要有少量底噪，调 `*_bias`

### 2.4 恢复参数

- `alpha_fast_rate`
  - 快速平均权重更新率
  - 越大：对“最近几帧突然变差”更敏感
  - 越小：不容易因为几帧坏观测就触发恢复

- `alpha_slow_rate`
  - 慢速平均权重更新率
  - 越小：长期基线更稳
  - 越大：长期基线更快跟着变化

- `random_injection_max_ratio`
  - 随机重注入粒子比例上限
  - 越大：恢复能力更强，但稳定时更容易抖
  - 越小：更稳，但丢定位后恢复更慢

### 2.5 初始化与性能参数

- `num_particles`
  - 粒子数量
  - 更多：更稳、更抗噪，但更耗 CPU
  - 更少：更快，但容易跳或不稳

- `init_field_width`
  - 初始撒粒子宽度，单位米

- `init_field_height`
  - 初始撒粒子高度，单位米

- `filter_period_ms`
  - 滤波循环周期，单位毫秒
  - 更小：更新更快，CPU 更高
  - 更大：更省资源，但位姿刷新更慢

## 3. 哪些参数最重要

对这套 topdown PF 来说，影响效果最大的通常是下面这些：

1. `meters_per_pixel`
2. `forward_axis`
3. `left_axis`
4. `sigma_hit`
5. `max_points`
6. `noise_xy`
7. `num_particles`

如果前 3 个没对，后面 PF 参数基本都很难调好。

## 4. 常见现象与优先排查参数

### 现象：观测点云整体比地图大一圈或小一圈

优先看：

- `meters_per_pixel`

### 现象：观测点云左右反了、前后反了、或者像旋转了 90 度

优先看：

- `forward_axis`
- `left_axis`

### 现象：粒子云一直散不开

优先看：

- `sigma_hit`
- `noise_xy`
- `num_particles`
- `meters_per_pixel`
- `forward_axis`
- `left_axis`

### 现象：粒子能收，但位置经常抖

优先看：

- `noise_xy`
- `noise_theta`
- `sigma_hit`
- `random_injection_max_ratio`

### 现象：明显丢定位后，很久都回不来

优先看：

- `alpha_fast_rate`
- `alpha_slow_rate`
- `random_injection_max_ratio`
- `num_particles`

### 现象：一两帧坏数据就触发大面积随机重注入

优先看：

- `alpha_fast_rate`
- `alpha_slow_rate`
- `random_injection_max_ratio`

### 现象：CPU 占用太高

优先看：

- `max_points`
- `num_particles`
- `filter_period_ms`

## 5. 参数联动建议

- `sigma_hit` 和 `meters_per_pixel` 是强相关的
  - 如果比例尺不准，再怎么调 `sigma_hit` 都容易表现奇怪

- `noise_xy` 和 `num_particles` 也常常要一起看
  - `noise_xy` 大时，如果 `num_particles` 太少，粒子会又散又乱

- `alpha_fast_rate`、`alpha_slow_rate`、`random_injection_max_ratio` 最好成组调
  - 单独把恢复比例开很大，通常只会让系统更抖

## 6. 推荐的起手调法

如果你刚开始接手一套新场地或新相机标定，可以用这套顺序：

1. 在 RViz 里只看地图和 `/field_line_observations_debug`
2. 调 `meters_per_pixel`
3. 调 `forward_axis` / `left_axis`
4. 让点云尽量压到地图线附近
5. 再把 `sigma_hit` 调到“既不过宽，也不过紧”
6. 如果粒子太飘，再降一点 `noise_xy`
7. 如果恢复太慢，再调恢复参数

## 7. 动态调参建议

V2 支持运行时改大部分 PF 相关参数，但不是所有参数都会立即产生同样类型的效果：

- 立即生效：
  - `sigma_hit`
  - `noise_xy`
  - `noise_theta`
  - `odom_noise_x_from_x`
  - `odom_noise_x_from_y`
  - `odom_noise_x_from_theta`
  - `odom_noise_x_bias`
  - `odom_noise_y_from_x`
  - `odom_noise_y_from_y`
  - `odom_noise_y_from_theta`
  - `odom_noise_y_bias`
  - `odom_noise_theta_from_x`
  - `odom_noise_theta_from_y`
  - `odom_noise_theta_from_theta`
  - `odom_noise_theta_bias`
  - `alpha_fast_rate`
  - `alpha_slow_rate`
  - `random_injection_max_ratio`
  - `off_map_penalty`
  - `max_points`
  - `meters_per_pixel`
  - `forward_axis`
  - `left_axis`

- 会触发粒子重初始化：
  - `num_particles`
  - `init_field_width`
  - `init_field_height`

- 会重建定时器：
  - `filter_period_ms`

- 会在下次收到地图时生效：
  - `occupancy_threshold`
  - `distance_transform_mask_size`

如果你在现场快速调参，建议先调“立即生效”的那组参数。
