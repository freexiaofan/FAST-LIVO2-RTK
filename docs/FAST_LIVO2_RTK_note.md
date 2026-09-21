# FAST-LIVO2-RTK 原理与代码笔记（总览）

> 代码路径：`src/FAST-LIVO2-RTK`（基于 FAST-LIVO2 论文 [2408.14035v2](Supplementary/2408.14035v2.pdf)，T-RO 2024，本仓库在其基础上做了大量工程定制：文件夹式标定读取、RTK 双路融合、离线批量位姿图优化、Global-LVBA keyframe 导出等）
> 核心思想一句话：**LiDAR-视觉-IMU 三源融合的紧耦合里程计（在线，迭代误差状态卡尔曼滤波），外挂一套 RTK 绝对位置约束（在线+离线两条路），前端产出的关键帧还能导出给 [Global-LVBA](../Global-LVBA) 做二次全局 BA。**

> **本文定位**：这是"进门"总览文档。项目里已经有 4 篇非常详细、逐行核实过的深挖文档，本文**不重复**它们已经讲透的数学推导，只给出全局地图和交叉引用；本文新增的是这 4 篇没有覆盖的部分——**主循环/模式调度、RTK 的三条融合路径、离线批优化模块、体素地图结构、以及和 LIO-SAM / FAST-LIO2 的横向对比**。

---

## 0. 已有的 4 篇深挖文档（先认识它们，避免重复造轮子）

| 文档 | 覆盖内容 | 何时看 |
|---|---|---|
| [docs/EXECUTION_FLOW_AND_ALGORITHM.md](docs/EXECUTION_FLOW_AND_ALGORITHM.md) | 整体调用链总览图 | 想先有个全局印象 |
| [docs/PREDICT_UPDATE_SUMMARY.md](docs/PREDICT_UPDATE_SUMMARY.md) | IMU 预测（`UndistortPcl`）vs LIO 更新（`StateEstimation`）的公式-代码对照 | 想搞懂"预测"和"更新"具体各自算了什么 |
| [docs/UNCERTAINTY_COMPUTE_AND_USAGE.md](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md) | 点噪声/状态协方差/平面协方差三层不确定性怎么构造、传播、进权重 | 想搞懂"为什么这个点权重是这个数" |
| [docs/VOXELMAPMANAGER_ITERATION_DATAFLOW.md](docs/VOXELMAPMANAGER_ITERATION_DATAFLOW.md) | `StateEstimation` 每轮迭代的矩阵数据流 + ESKF 从零讲起 | 想逐行看懂 IEKF 怎么解出来的，或完全没学过 ESKF |
| [docs/VISUAL_POINT.md](docs/VISUAL_POINT.md) | `VisualPoint` 全生命周期（生成/检索/光度更新/维护/销毁）+ 论文逐节对照 | 想搞懂 VIO 那一半怎么工作、和论文哪里不一样 |

> ⚠️ **行号漂移提醒**：`PREDICT_UPDATE_SUMMARY.md` 明确标注了"按 2026-07-28 状态核实"；`LIVMapper.cpp` 在那之后又新增了大量与核心算法无关的定制代码（文件夹式标定加载、Global-LVBA 导出、大量调试 `ROS_INFO`），**`LIVMapper.cpp` 里的具体行号可能已经和文档不完全一致**，但 `voxel_map.cpp`/`IMU_Processing.cpp` 的核心算法函数改动较少，公式和相对位置关系仍然可信。本文所有 `LIVMapper.cpp` 行号引用均为本次重新 `grep`/通读后的当前值。

---

## 1. 和 LIO-SAM / FAST-LIO2 的横向对比

（另两篇笔记：[LIO-SAM-原理与代码笔记.md](../LIO-SAM/LIO-SAM-原理与代码笔记.md)、[FAST_LIO-原理与代码笔记.md](../FAST_LIO/FAST_LIO-原理与代码笔记.md)）

| | LIO-SAM | FAST-LIO2 | **FAST-LIVO2-RTK** |
|---|---|---|---|
| 传感器 | LiDAR+IMU(+GPS) | LiDAR+IMU | **LiDAR+IMU+相机(+RTK)** |
| 状态求解 | GTSAM 因子图 ISAM2 | IKFoM 流形 IESKF | **手写 ESKF（`StatesGroup`，19 维，非 IKFoM）** |
| 地图结构 | 关键帧点云拼图 | ikd-Tree | **体素哈希 + 多层八叉树**（`VoxelOctoTree`） |
| 特征提取 | LOAM 角点/面点 | 不分类（全面点） | **不分类**（面点残差为主，另加 VIO 光度残差） |
| 回环 | 有（ICP+距离触发） | 无 | **无**（在线部分）；离线批优化部分靠 RTK 绝对约束"拉直"轨迹，效果类似弱回环 |
| 视觉 | 无 | 无 | **有**：LiDAR 平面先验 + patch 光度直接法，ESIKF 顺序更新（先 LIO 后 VIO） |
| 绝对定位 | GPS 因子（在线因子图） | 无 | **RTK 三条路**：①在线联合 IEKF、②在线独立线性 KF、③离线批量 GTSAM 位姿图（见 §5） |
| 外参在线估计 | 无 | 支持（lidar-IMU） | 不在线估计（外参从标定文件夹静态读取） |
| 曝光时间估计 | 无 | 无 | **有**（VIO 状态里带 `inv_expo_time`，见 §4.3） |

一句话总览：**如果说 FAST-LIO2 是"IESKF + ikd-Tree 的纯 LiDAR 里程计"，LIO-SAM 是"因子图 SLAM"，那 FAST-LIVO2-RTK 就是在 FAST-LIO2 的手写 ESKF 骨架上，加了一路视觉 ESIKF 顺序更新、一路体素八叉树地图、再外挂了 RTK 的双在线路径 + 一套独立于主里程计之外的离线批量位姿图（这部分才真正像 LIO-SAM 的 GTSAM 后端）。**

---

## 2. 整体架构与数据流

系统由**两个独立可执行程序/模块**组成：

```
┌─────────────────────────── 在线实时前端（LIVMapper::run，src/LIVMapper.cpp:988）───────────────────────────┐
│                                                                                                            │
│  传感器回调（imu_cbk / standard_pcl_cbk|livox_pcl_cbk / img_cbk / rtk_cbk|localization_odom_cbk）           │
│         └─ 各自塞进 imu_buffer / lid_raw_data_buffer / img_buffer / (rtk 挂在 measures.back().rtk 里)        │
│                              │                                                                             │
│                     sync_packages()  ── 按 slam_mode_ 分 ONLY_LIO / LIVO 两态机，决定这一包是 LIO 还是 VIO   │
│                              │                                                                             │
│                     handleFirstFrame() → processImu()（IMU 预测+去畸变，见 §4）                              │
│                              │                                                                             │
│                    stateEstimationAndMapping()  按 lio_vio_flg 分发：                                       │
│                       ├─ LIO/LO → handleLIO()：体素地图 IEKF 更新（见 §5）→(可选)handleRTK()（见 §6.2）      │
│                       └─ VIO    → handleVIO()：视觉光度 ESIKF 更新（见 §7）                                  │
│                              │                                                                             │
│              （可选）imu_prop_callback：从最新 EKF 状态出发做高频前向传播，发布低延迟里程计（见 §8）           │
│                              │                                                                             │
│              （可选）saveLvbaKeyframeLidar / saveLvbaKeyframeImageOdom：导出 Global-LVBA keyframe（见 §9）   │
└────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
                                             │ (bag 播完/建图停了 或 手动 /run_offline_opt)
                                             ▼
┌────────────────────────── 离线批量后端（optimization 模块，见 §6.3）──────────────────────────────────────────┐
│  订阅 LIVO2 关键帧位姿+点云（message_filters 同步）+ RTK/GNSS 轨迹                                            │
│  initialAlign（SVD 刚体对齐 + DTW 时间偏移估计）→ buildBatchGraph（GTSAM BetweenFactor+GPSFactor）             │
│  → LevenbergMarquardtOptimizer 批量求解 → 输出优化后全局地图 + TUM 轨迹                                       │
└────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**关键点**：在线前端**没有**闭环检测、**没有**全局位姿图——它只是一条不断被 IMU 预测和"LIO 或 VIO"观测交替修正的滤波状态（`_state`，`StatesGroup` 类型）。真正意义上"让整条轨迹全局一致"的能力来自离线批量后端（§6.3），这是它区别于 LIO-SAM 在线因子图的关键设计取舍：**在线追求实时性和简单的滤波结构，全局一致性放到离线单独解决**。

---

## 3. 状态表示：19 维 `StatesGroup`（手写 ESKF，不是 IKFoM）

定义在 `include/common_lib.h`（`DIM_STATE=19`）：

| 索引 | 分量 | 说明 |
|---|---|---|
| 0–2 | 姿态误差 $\delta\boldsymbol\theta$ | so(3) 右扰动，`R ← R·Exp(δθ)` |
| 3–5 | 位置误差 $\delta\mathbf p$ | |
| 6 | 曝光时间误差 $\delta e$ | **相机专属**，`inv_expo_time`，纯 FAST-LIO2 没有这一维 |
| 7–9 | 速度误差 $\delta\mathbf v$ | |
| 10–12 | 陀螺零偏误差 $\delta\mathbf b_g$ | |
| 13–15 | 加计零偏误差 $\delta\mathbf b_a$ | |
| 16–18 | 重力误差 $\delta\mathbf g$ | |

对比 FAST-LIO2 的 `state_ikfom`（23 维，流形+IKFoM 模板库，含在线外参）：这里**没有把 lidar-IMU 外参放进状态**（外参从标定文件夹静态读入，`extR`/`extT` 全局变量），**多了曝光时间这一维**（VIO 用），且实现上是**手写的 `operator+=`/`operator-`**（`common_lib.h`），不是 MTK/IKFoM 那套流形工具箱模板——这意味着这个仓库的 ESKF 数学和 FAST-LIO2 的 IKFoM 版本形式上很像（都是"姿态右扰动+其余误差态直接相加"），但工程实现是两条独立代码路径，不能直接照搬 FAST-LIO2 笔记里对 `state_ikfom`/`esekfom.hpp` 的代码级细节。

`_state`（当前滤波状态）和 `state_propagat`（本轮 IMU 预测出的先验，供 `voxel_map.cpp::StateEstimation` 内部"先验偏移补偿项"使用）是两个不同变量——详见 [PREDICT_UPDATE_SUMMARY.md §5](docs/PREDICT_UPDATE_SUMMARY.md)。

---

## 4. IMU 预测与去畸变（`processImu` → `ImuProcess::Process2` → `UndistortPcl`）

`LIVMapper::processImu()`（`src/LIVMapper.cpp:610`）只是薄薄一层封装：

```cpp
p_imu->Process2(LidarMeasures, _state, feats_undistort, T_G_to_W);
if (gravity_align_en) gravityAlignment();
state_propagat = _state;
voxelmap_manager->state_ = _state;
voxelmap_manager->feats_undistort_ = feats_undistort;
```

真正的预测数学（离散运动学传播 `R←R·Exp(ωdt)`、`P = F_x·P·F_x^T + Q`、逐点反向去畸变补偿）已经在 [PREDICT_UPDATE_SUMMARY.md §2-3](docs/PREDICT_UPDATE_SUMMARY.md) 逐行核实过，此处不重复。

**首帧重力对齐**（`gravityAlignment()`，`src/LIVMapper.cpp:592`）：只在 `gravity_align_en=true`（默认 `false`，见 `uav.gravity_align_en`）且 IMU 静止初始化完成后跑一次——用初始化阶段估到的重力方向 `_state.gravity` 算一个"把 IMU 系 z 轴掰正到 -z（世界系重力方向）"的旋转 `G_q_I0`，同时旋转 `pos_end/rot_end/vel_end/gravity`。这一步只影响**世界系的定义**（把 IMU 静止时的重力方向定为世界系竖直向下），不影响相对位姿精度；无人机等对初始姿态敏感的场景才需要打开，车载/地面场景通常关闭（本仓库车载配置都是 `false`）。

---

## 5. LIO 更新：`handleLIO` → `VoxelMapManager::StateEstimation`

`handleLIO()`（`src/LIVMapper.cpp:773`）的流程：

```
空点云检查 → VoxelGrid 降采样(feats_down_body) → Body→World 变换
  → 首帧: BuildVoxelMap()（只建一次，之后靠 UpdateVoxelMap 增量维护）
  → 读 rtk_good（rtk_ini 且这一帧 rtk 时间戳有效）
  → voxelmap_manager->StateEstimation(state_propagat, rtk_good, rtk_data)   ◀── 核心，见下
  → _state = voxelmap_manager->state_   （把更新结果写回主状态）
  → saveLvbaKeyframeLidar()（可选，见 §9）
  → 重算每点世界协方差 pv_list_[i].var，UpdateVoxelMap（增量插入/更新八叉树叶子平面）
  → mapSliding()（可选，地图滑窗）
  → 发布 odometry / path / 可视化点云
```

`StateEstimation` 内部"点噪声构造→迭代 IEKF→加权最小二乘求解"的完整数学，[VOXELMAPMANAGER_ITERATION_DATAFLOW.md](docs/VOXELMAPMANAGER_ITERATION_DATAFLOW.md) 和 [UNCERTAINTY_COMPUTE_AND_USAGE.md](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md) 已经逐行核实，核心结论：

- 每个点到平面残差的方差 $\sigma_l = \underbrace{J_{nq}\Sigma_\pi J_{nq}^T}_{\text{平面参数不确定性}} + \underbrace{n^T\Sigma_{p,w}n}_{\text{点世界系不确定性}}$，权重 $R^{-1}=1/(0.001+\sigma_l)$；
- 数据关联不是纯"最近距离"，而是先过 $|d|<\sigma_{num}\sqrt{\sigma_l}$ 门限，再在候选平面里选似然 $\propto\sigma_l^{-1/2}\exp(-d^2/2\sigma_l)$ 最大的；
- 用矩阵求逆引理把"对 `effct_feat_num_×effct_feat_num_` 的 `R` 求逆"等价变形成"对 `DIM_STATE×DIM_STATE` 的小矩阵求逆"，求解式 `solution = K1·H^Tz + vec - G·vec_pose`，其中 `vec = state_propagat - state_` 是"迭代过程中相对 IMU 先验的偏移量"——这是 IEKF（而非一次性 EKF）的关键项。

**体素地图结构**（`include/voxel_map.h`）：地图不是单层 kd-tree（像 FAST-LIO2 的 ikd-Tree），而是 **`unordered_map<VOXEL_LOCATION, VoxelOctoTree*>` 哈希表 + 每个体素内部一棵多层八叉树**（`max_layer_`/`layer_init_num_` 控制，`HH-LVGO.yaml` 里 `max_layer: 2`）：一个体素内点数够多且足够平面（`min_eigen_value`/`planer_threshold` 判据）就直接拟合成一个 `VoxelPlane`（含 6×6 参数协方差 `plane_var_`）；不够平面就往下细分八叉树子节点继续拟合，直到 `max_layer_` 或点数不够为止。`build_single_residual` 对每个待匹配点就是"先查其所在体素，若该层不是平面就递归下探子节点"。

> ⚠️ **已知问题（PREDICT_UPDATE_SUMMARY.md 已记录，代码里也自己标了 `// todo 可能有越界？？？`）**：`rtk_good=true` 时 `effct_feat_num_ += 3`，但填充 LIO 残差的循环 `for (i=0;i<effct_feat_num_;i++)` 直接读 `ptpl_list_[i]`，而 `ptpl_list_` 并没有跟着多分配 3 个元素——最后 3 次循环存在**越界读取**。如果要动 `voxel_map.cpp::StateEstimation` 里这段代码，务必先确认这个问题是否已经修复（比如把循环上界改成 `ptpl_list_.size()`，RTK 那 3 行单独处理）。

---

## 6. RTK 融合：三条完全独立的路径（容易搞混，务必分清楚）

这是本仓库相对官方 FAST-LIVO2 最大的定制点（仓库名里的"RTK"就来自这里），一共有 **3 条独立的融合路径**，触发时机、观测模型、噪声设置都不一样：

### 6.1 路径一：`StateEstimation` 内部联合加权（在线，随 LIO 每帧一起解）

`rtk_good=true` 时，在 `voxel_map.cpp::StateEstimation` 同一个加权最小二乘方程里**追加 3 行 RTK 位置观测**（`Hsub`/`R_inv`/`meas_vec` 各补 3 行），和 LIO 的点到面残差**一起**解出这一轮的状态增量——见 [PREDICT_UPDATE_SUMMARY.md §4](docs/PREDICT_UPDATE_SUMMARY.md)、[UNCERTAINTY_COMPUTE_AND_USAGE.md §6](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md)。噪声来自配置 `gps.rtk_std`（`[x,y,z]` 标准差，例如 `HH-LVGO.yaml` 里 `[0.05, 0.05, 0.10]`）。

> **配置注释里记录的真实教训**（`HH-LVGO.yaml:101-103`）：`rtk_std` 原来在 `voxel_map.cpp` 里硬编码成 `[0.003,0.003,0.1]`（3mm/3mm/10cm），相对实测残差（xy 十几厘米、z 数米）**过于乐观**——RTK 位置几乎被"钉死"，导致激光残差只能靠旋转去消化，把姿态带偏。现在改成从配置读取且给了更保守的默认值。**这是一个非常典型的"协方差设得太自信导致状态被观测硬拽跑偏"的案例**，调 RTK 融合参数时值得记住。

### 6.2 路径二：`handleRTK()` 独立线性 KF（在线，LIO 更新完之后单独再修一次）

`handleLIO()` 跑完、且 `rtk_en && rtk_ini` 时，`stateEstimationAndMapping()`（`src/LIVMapper.cpp:629`）会再调一次 `handleRTK()`（`src/LIVMapper.cpp:650`）——这是一个**完全独立、非迭代、标准形式**的线性卡尔曼更新，不走体素地图、不参与 LIO 的加权最小二乘：

```cpp
Eigen::Vector3d z_k = rtk_data.p;
z_k[2] = _state.pos_end(2);                          // ⚠️ z 直接用当前估计值覆盖，RTK 高度不参与这次更新
H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();    // 只对"位置误差"这 3 列有雅可比
Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * 1e-4 * 5;
S = H*P*H^T + R_cov;  K = P*H^T*S^{-1};
y = z_k - (_state.pos_end + _state.rot_end * T_I_to_R);   // T_I_to_R: IMU->RTK天线杆臂
dx = K*y;  _state += dx;  _state.cov = (I-K*H)*P;
```

几个值得注意的细节：
- **`z_k[2] = _state.pos_end(2)`**——RTK 的高度分量被直接替换成当前估计值，等价于这条路径**只用 RTK 的水平位置**（和 `voxel_map.cpp` 里 `rtk_std` z 轴给一个很宽松/近乎关闭的思路是一致的，配置注释里也提到"与 handleRTK 里 z_k[2]=state.z 的做法一致"）。
- **雅可比 `H` 没有姿态列**：创新量 `y` 的计算里包含 `_state.rot_end * T_I_to_R`（杆臂旋转到世界系），理论上对姿态误差 `δθ` 也有一个 $-R\cdot(T_{I\to R})^\wedge$ 的雅可比项（推导方式同 [FAST_LIO 笔记](../FAST_LIO/FAST_LIO-原理与代码笔记.md) 里姿态雅可比那一节），但代码里 `H.block<3,3>(0,0)` 保持为 0——**这是一个简化（略去了杆臂-姿态耦合项），杆臂 `T_I_to_R` 越大、姿态不确定性越大时这个简化的误差越明显**，在小杆臂场景下影响有限。
- `R_cov` 是硬编码的 `5e-4·I`（约 2.2cm 标准差），和路径一从配置读取的 `rtk_std` 是**两套完全不同、互不关联的噪声参数**——调参时容易忘记两处都要改。

### 6.3 路径三：离线批量位姿图优化（`optimization` 模块，`include/optimization.h` + `src/optimization.cpp`）

这是一套**独立于在线里程计之外**、类似 LIO-SAM 后端的 GTSAM 批量位姿图优化，在线建图过程中只是**收集数据**，真正求解发生在建图**结束之后**（自动触发：关键帧数量不再增长超过 `finish_timeout` 秒；或手动 `rostopic pub /run_offline_opt std_msgs/Empty -1`）：

1. **数据收集**：`syncedCallback` 用 `message_filters::ApproximateTime` 同步 LIVO2 里程计（关键帧位姿）与点云；`gpsHandler`/`odomRtkHandler` 收集 RTK/GNSS 轨迹（`odom_rtk_en=true` 时改用 `/localization/odom` 顶替原始 GNSS PVT 消息）。
2. **`initialAlign()`**：先用 `calculateDtwTimeOffset()`（**动态时间规整 DTW**，`FastDTW/example.hpp`）估计 LIVO2 轨迹和 RTK 轨迹之间的**时间偏移**（两路数据时钟可能不同源、有固定延迟），再用 `computeSVD()`（Umeyama 型刚体配准：去质心→叉积矩阵 SVD→`R=U·V^T`，行列式为负时翻转最后一维保证是纯旋转不含镜像）算出 LIVO2 世界系到 RTK/UTM 世界系的刚体变换 `T_imu_rtk`。`align_window_` 控制用多长轨迹做这次配准。
   > **配置注释记录的教训**（`HH-LVGO.yaml:114-116`）：`align_window` 原来写死 50 秒，**短轨迹对 6 自由度刚体 SVD 是退化构型**（比如一段近似直线的轨迹配准不出正确的俯仰角），会拟出虚假倾角——现在默认 `-1`（用全部关键帧轨迹）。
3. **`buildBatchGraph()`**：construct GTSAM 图——LIVO2 相邻关键帧相对位姿作 `BetweenFactor`（噪声 `livo2_RPY_cov`/`livo2_XYZ_cov`），RTK 绝对位置作 `GPSFactor`（噪声 `rtk_cov`，**按行驶距离每隔 `rtk_factor_interval` 米加一个**，而不是每个关键帧都加）。
   > **配置注释记录的教训**（`HH-LVGO.yaml:117-119`）：关键帧是按时间产生的，车在红灯前静止不动时也在不断攒关键帧，如果每个关键帧都加一个 RTK 因子，**同一个物理位置会被塞进几十个几乎重复的绝对约束**，把优化结果拽偏（相当于给这一个点的权重不成比例地放大）——因此改成按里程抽稀。
4. **`LevenbergMarquardtOptimizer` 批量求解**（不是 ISAM2 增量式），输出优化后的轨迹（`writeOptimizedTumTrajectory`）和全局地图（`saveOptimizedGlobalMap`）。

**这条路径本质上是把"在线滤波里程计的局部漂移"用"离线一次性全局位姿图优化"来修正**，思路和 LIO-SAM 的 GTSAM 后端很像，但**不在线跑**（LIO-SAM 是边建图边用 ISAM2 增量优化），也**不含回环因子**（只有里程计 BetweenFactor + RTK GPSFactor，没有距离触发的回环检测），是一种更轻量、专为"有 RTK 参考轨迹"场景设计的离线后处理。

---

## 7. VIO 更新（视觉部分）

`handleVIO()`（`src/LIVMapper.cpp:710`）核心只有一行：`vio_manager->processFrame(...)`。`VisualPoint` 的完整生命周期（`generateVisualMapPoints` 新建 → `retrieveFromVisualSparseMap` 检索+仿射 warp → `computeJacobianAndUpdateEKF` 光度残差迭代更新状态 → `updateVisualMapPoints`/`updateReferencePatch` 维护）、光度雅可比链式推导、以及和论文 Section V/VII 的逐条对照（含代码相对论文的 6 处简化，比如未实现的法向光度优化、被注释掉的 80° 视角剔除门），[VISUAL_POINT.md](docs/VISUAL_POINT.md) 已经讲得非常完整，此处不重复，只提两个容易漏看的点：

- **VIO 状态更新和 LIO 是同一个 `_state`，但走的是完全不同的一套雅可比/求解代码**（`vio.cpp::updateState`，19 维状态里只有姿态/位置/曝光 7 维有观测列），两者靠 `sync_packages` 的 LIO/VIO 状态机**串行交替**执行（论文里叫"Sequential Update"：同一个 ESIKF 框架，先用这一帧的 LiDAR 点收敛，再用图像光度收敛），不是同时联合求解。
- **曝光时间 `inv_expo_time`** 作为状态一部分被一起估计，专门应对相机自动曝光变化对光度残差的干扰；这是 FAST-LIVO2 相对纯 LiDAR 系统（LIO-SAM/FAST-LIO2）多出来的、且和视觉强相关的一个状态维度。

---

## 8. 高频里程计支路：`imu_prop_callback`

`imu_prop_callback()`（`src/LIVMapper.cpp:1036`）不参与状态估计，只是从**最近一次 LIO/VIO 更新完的 `latest_ekf_state`** 出发，用还没被用掉的高频 IMU 数据做纯运动学前向积分（`prop_imu_once`，公式和 §4 的预测公式一样：`R←R·Exp(ωdt)`、`v←v+a_world·dt`、`p←p+v·dt+½·a_world·dt²`），发布一路**低延迟、但会在下一次 LIO/VIO 更新时被"拉回"**的里程计（`/LIVO2/imu_propagate`），纯粹是给需要高频反馈的下游（比如控制回路）用的展示/参考支路，不影响主状态估计精度。

---

## 9. Global-LVBA Keyframe 导出（本仓库定制功能）

`save_lvba_en_` 开关打开时，`handleLIO()`/`handleVIO()` 结尾分别调 `saveLvbaKeyframeLidar()`/`saveLvbaKeyframeImageOdom()`（`src/LIVMapper.cpp:338/382`），把关键帧的**激光系点云 + 世界位姿(`T_utm_ins`) + 去畸变全分辨率图像 + 标定文件夹拷贝**按 [Global-LVBA](../Global-LVBA/) 的 `custom`/`keyframe` 数据格式写盘（`pointclouds/odoms/images/calib` 四个子目录，`{idx}_{ts}` 命名严格配对）。同一帧 LIO 先写点云+LIO 位姿，随后 VIO 用**更优**的视觉修正位姿覆写同一个 odom 文件——这是"LIO 先给一个可用位姿，VIO 精修后再覆盖"的写入顺序，写入前用点云是否成功写入来门控 `idx` 递增，避免产生孤儿文件。这部分与本仓库另一个 package [Global-LVBA](../Global-LVBA/) 的输入契约是强耦合的（细节见我的记忆 `global-lvba-cbd-lidar-only` / `fast-livo2-lvba-export`），改动前建议先确认 Global-LVBA 那边的 `DatasetIO::loadFramesKeyframe` 读取逻辑没变。

---

## 10. 代码里的坑与细节 Checklist

- [ ] **RTK+`ptpl_list_` 越界读取**（§5 末尾）：`rtk_good` 时 `effct_feat_num_+=3` 但 `ptpl_list_` 没跟着扩容，代码里自己标了 `// todo 可能有越界？？？`，改这段代码前先确认是否已修。
- [ ] **RTK 有三条完全独立的融合路径**（§6.1/6.2/6.3），噪声参数互不相关（`voxel_map.cpp` 内部读配置 `rtk_std`；`handleRTK` 硬编码 `5e-4`；离线批优化读配置 `rtk_cov`）——排查"RTK 好像没生效"类问题时，先搞清楚是哪一条路径的参数没调对。
- [ ] `handleRTK()` 里 `z_k[2]` 直接替换成当前估计值，这条路径**实际只用了 RTK 的水平位置**；雅可比矩阵没有姿态-杆臂耦合项（`H.block<3,3>(0,0)` 恒为 0），杆臂大时是一个已知简化。
- [ ] 离线批优化的 `align_window`（SVD 配准用的轨迹长度）、`rtk_factor_interval`（RTK 因子按里程抽稀）都是从真实踩坑里改出来的默认值（原来分别是硬编码 50 秒/每关键帧都加），改动这两个参数前建议看一下 `HH-LVGO.yaml` 里对应注释的完整背景。
- [ ] `StatesGroup` 是**手写**ESKF（19 维，含曝光时间），和 FAST-LIO2 的 `state_ikfom`（IKFoM 流形模板，23 维，含在线外参）是两套完全不同的实现，即使数学思路相似（都是右扰动+误差态相加），**代码/雅可比细节不能跨仓库照搬**。
- [ ] 地图结构是"体素哈希 + 每格一棵多层八叉树"（`VoxelOctoTree`），不是单一全局 kd-tree/ikd-Tree；`max_layer`/`layer_init_num`/`min_eigen_value` 等参数控制何时继续细分八叉树 vs 直接拟合平面。
- [ ] VIO 和 LIO 更新**串行交替**（同一个 `_state`，`sync_packages` 靠 `lio_vio_flg` 状态机决定这一包数据走哪条路），不是同时联合求解——这是论文里的"Sequential Update"设计，不要以为两者是同一个观测方程里的两组残差。
- [ ] `img_en=0`（纯 LIO 模式，`slam_mode_=ONLY_LIO`）时不存在 VIO 分支，`keyframe_time` 直接取 `lidar_frame_end_time`；这和 `LIVO` 模式下 `keyframe_time` 取图像曝光时刻是两套不同的关键帧时间戳定义，混用会导致 LVBA 导出的时间戳含义不一致。
- [ ] 本仓库的外参不是硬编码/单一 yaml 参数，而是从 `extrin_calib/calib_path` 指向的标定文件夹按传感器名读取（`intrinsics/`+`extrinsics/`），且明确区分了 `imu_frame`（IMU/ins 系）和 `vehicle_frame`（车体系）——`LIVMapper.cpp` 里有醒目的历史踩坑注释：某数据集的 ins 相对 vehicle 有 180° 翻转+1.1m 平移，误用 vehicle 系当 lidar→IMU 外参会导致点云相对 IMU 重力方向整体翻转，排查"重力方向不对/建图整个翻转"类问题时先看这里。

---

## 11. 重点知识速记（复习用）

1. **两大模块**：在线实时前端（`LIVMapper`，纯滤波，无回环/无全局图）+ 离线批量后端（`optimization`，GTSAM 位姿图，有 RTK 绝对约束但无回环因子）——全局一致性由离线部分负责，在线只管局部精度和实时性。
2. **状态是 19 维手写 ESKF**（`StatesGroup`：姿态/位置/**曝光时间**/速度/陀螺零偏/加计零偏/重力），不是 FAST-LIO2 那套 IKFoM 流形模板，外参也不在线估计（静态标定文件夹读入）。
3. **地图是体素哈希+多层八叉树**（不是 ikd-Tree/kd-tree），每个体素够平面就直接拟合成一个带 6×6 参数协方差的 `VoxelPlane`，不够就递归细分。
4. **LIO 更新是加权最小二乘 IEKF**：残差方差 = 平面参数不确定性投影 + 点世界系不确定性投影，数据关联先过 σ 门限再选最大似然平面，用矩阵求逆引理把大矩阵求逆变成 `DIM_STATE×DIM_STATE` 小矩阵求逆。
5. **VIO 是 LiDAR 先验深度+法向的平面 warp 光度直接法**：`VisualPoint` 位置/法向来自 LiDAR 而非视觉三角化，参考 patch 经仿射/单应 warp 到当前视角后逐像素比光度，曝光时间作为状态一起估计。
6. **VIO 和 LIO 串行交替**（Sequential Update），共享同一个 `_state`，但各自有独立的观测方程/雅可比/求解代码。
7. **RTK 有三条独立路径**：①`StateEstimation` 内联合加权（在线，和 LIO 同一方程）；②`handleRTK` 独立线性 KF（在线，LIO 之后单独再修一次，实际只用水平位置）；③离线批量 GTSAM 位姿图（SVD 刚体对齐+DTW 时间同步+按里程抽稀的 GPSFactor）。三条路噪声参数互不相通。
8. **已知的一个未确认修复的越界 bug**：RTK 观测让 `effct_feat_num_` 多算 3，但 `ptpl_list_` 容量没跟着变。
9. **配置文件里留了大量真实踩坑记录**（RTK 协方差过于自信导致姿态被拽偏、SVD 配准窗口太短导致虚假俯仰角、RTK 因子未按里程抽稀导致红灯堆积因子拽偏轨迹、lidar→IMU 外参系搞混导致点云整体翻转）——这些注释比代码本身更值得读,能少踩很多坑。
10. **详细数学推导看已有 4 篇深挖文档**（§0），本文件只负责把它们和新增的主循环/RTK/离线后端/体素地图/LVBA 导出串成一张完整地图。
