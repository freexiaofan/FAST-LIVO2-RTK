# VisualPoint：视觉地图点是怎么得到、怎么用的

本文聚焦 FAST-LIVO2 里的 **视觉地图点 `VisualPoint`**：它从哪来、在一帧里怎么被检索、怎么进
光度 EKF 更新、又怎么被维护和销毁。所有公式都尽量落到具体代码行（行号按当前仓库状态核对）。

核心文件：
- 类定义：[include/visual_point.h](../include/visual_point.h)、[src/visual_point.cpp](../src/visual_point.cpp)
- 观测 patch：[include/feature.h](../include/feature.h)
- 全部逻辑：[src/vio.cpp](../src/vio.cpp)（`VIOManager`）
- 数据结构：[include/vio.h](../include/vio.h)、[include/common_lib.h](../include/common_lib.h)

相关文档：
- [VOXELMAPMANAGER_ITERATION_DATAFLOW.md](VOXELMAPMANAGER_ITERATION_DATAFLOW.md)：LiDAR 侧 voxel 地图与 ESIKF
- [PREDICT_UPDATE_SUMMARY.md](PREDICT_UPDATE_SUMMARY.md)：预测/更新的公式
- **论文**：FAST-LIVO2（[Supplementary/2408.14035v2.pdf](../Supplementary/2408.14035v2.pdf)，T-RO 2024）——本文的公式/章节号均按此论文标注。

---

## 论文对照速查（章节 · 公式 · 代码）

`VisualPoint` 的机制分散在论文 **Section V（Local Mapping）** 和 **Section VII（Visual Measurement
Model）**，ESIKF 融合在 **Section IV-D**。对照如下：

| 论文章节 / 公式 | 干什么 | 本文 | 代码 |
|---|---|---|---|
| **V-C** Visual Map Point Generation | 生成新视觉点、加 patch 金字塔 | §3 | `generateVisualMapPoints` |
| **V-D** Reference Patch Update，eq (12) | 打分选参考 patch | §6.2 | `updateReferencePatch` |
| **V-E** Normal Refine，eq (13)-(16) | 仿射 warp 的法向 + 法向优化 | §4.3(c) / §6.2 | `getWarpMatrixAffineHomography` / `updateReferencePatch` |
| **VII-A.1** Visible Voxel Query | 可见 voxel 查询 | §4.1-4.2 | `retrieveFromVisualSparseMap` 前半 |
| **VII-A.2** Raycasting on Demand | 按需光线投射补点 | §4.2 | `raycast_en` 分支 |
| **VII-A.3** Outlier Rejection | 遮挡/深度/视角外点剔除 | §4.3(a) | 深度连续性检查等 |
| **VII-B** Sparse-Direct，eq (21)-(23) | 光度残差 + 逆向组合 | §5.1 | `updateState` / `updateStateInverse` |
| **IV-D** Sequential Update，eq (11) | ESIKF 顺序更新（先 LiDAR 后视觉） | §5.3 | `computeJacobianAndUpdateEKF` |

> **⚠️ 本代码相对论文的两处简化（详见 §10）**：(1) 法向只用 LiDAR 平面先验（论文 "Plane prior"），
> **未实现** eq (14)-(16) 的光度法向优化（论文 "Plane normal refined"，独立线程）；(2) 参考 patch
> 打分把 eq (12) 的 $\omega_1$ 权重简化成了等权 `NCC + cos`。

---

## 0. 一句话总览

`VisualPoint` 是**挂在场景表面上的一个 3D 点**，它记住自己在若干历史帧里的图像小块（patch，即
`Feature`）。新的一帧来时，系统把可见的 `VisualPoint` 从地图里捞出来，用它们的历史 patch 和当前
图像做**逐像素光度对齐**，反解出相机（进而 IMU）的位姿——这就是 VIO 更新。位姿更新完，再用当前
LiDAR 点**新建**一批 `VisualPoint`、给老点**补观测**、**精化法向和参考 patch**。

一句话：**`VisualPoint` = 3D 位置 + 法向 + 一串历史 patch 观测**，是"直接法"光度残差的载体。

---

## 1. 数据结构

### 1.1 `VisualPoint`（[visual_point.h:23-46](../include/visual_point.h#L23-L46)）

| 字段 | 含义 | 关键用途 |
|---|---|---|
| `pos_` | 点在**世界系**的 3D 坐标 | 投影、残差、warp 都基于它 |
| `normal_` | 该点处的**表面法向**（世界系） | 单应仿射 warp（`normal_en`）、参考 patch 打分 |
| `previous_normal_` | 上一次的法向 | 判断法向是否收敛 |
| `normal_information_` | 法向估计的信息矩阵（逆协方差） | 预留 |
| `covariance_` | 点的 3D 协方差 | 平面一致性 3σ 检验 |
| `obs_` | `list<Feature*>`：**观测到该点的所有历史 patch** | 直接法的原料 |
| `ref_patch` | 当前选定的**参考 patch** | warp 的源、残差的基准 |
| `has_ref_patch_` | 是否已选出 ref_patch | |
| `is_normal_initialized_` | 法向是否已初始化 | 没初始化的点不参与 warp |
| `is_converged_` | 法向是否已收敛 | 收敛后只留 ref_patch、不再更新 |

> `boost::noncopyable`：`VisualPoint` 持有裸指针 `Feature*` 且自己负责 `delete`，禁止拷贝避免
> 二次释放。

### 1.2 `Feature`（一个 patch 观测，[feature.h:19-54](../include/feature.h#L19-L54)）

| 字段 | 含义 |
|---|---|
| `point_` | 反向指回它所属的 `VisualPoint` |
| `patch_` | `float*`，多金字塔层的图像小块像素（真正被对齐的数据） |
| `px_` | 该 patch 在**第 0 层图像**的像素坐标 |
| `f_` | 该像素的**单位方向向量**（bearing，相机系） |
| `T_f_w_` | 提取该 patch 时**相机的位姿** $T_{cw}$（world→cam） |
| `level_` | 提取时所在金字塔层 |
| `img_` | 提取时的整幅灰度图（warp 时要从原图重采样） |
| `mean_` | patch 均值（NCC/去均值用，惰性计算） |
| `score_` | patch 质量得分（NCC+视角），选 ref_patch 用 |
| `inv_expo_time_` | 提取时的**逆曝光时间**，做光度归一化 |
| `id_` | 提取时的帧号 |

`Feature::pos()` = `T_f_w_.inverse().translation()`，即"拍到这个 patch 时相机在世界系的位置"。

**一对多关系**：一个 `VisualPoint` 有多个 `Feature`（不同帧、不同视角的观测），其中一个被选为
`ref_patch`。

### 1.3 全局视觉地图 `feat_map`（[vio.h](../include/vio.h)）

```
feat_map : unordered_map<VOXEL_LOCATION, VOXEL_POINTS*>
```

按 **0.5 m 的哈希 voxel** 组织：每个 voxel 存一串落在其中的 `VisualPoint*`
（`VOXEL_POINTS::voxel_points`）。这是**跨帧持久**的视觉地图。

### 1.4 当前帧子图 `visual_submap`（`SubSparseMap`，[vio.h:26-56](../include/vio.h#L26-L56)）

`retrieveFromVisualSparseMap` 每帧重建，是喂给 EKF 的"这一帧要用的点+数据"：

| 数组 | 含义 |
|---|---|
| `voxel_points` | 本帧选中的 `VisualPoint*` 列表 |
| `warp_patch` | 每个点的**参考 patch 经仿射 warp 后**的像素（多层） |
| `inv_expo_list` | 每个点参考 patch 的逆曝光时间 |
| `search_levels` | 每个点的最佳搜索金字塔层 |
| `errors` / `propa_errors` | 每个点的光度误差 |
| `add_from_voxel_map` | 供 `generateVisualMapPoints` 补点用的候选 |

### 1.5 `pointWithVar`（LiDAR 点，[common_lib.h:113-134](../include/common_lib.h#L113-L134)）

VIO 的输入 `pg` 是一串 `pointWithVar`（来自 LIO 的 `pv_list_`）：带**世界坐标 `point_w`**、
**法向 `normal`**、**协方差 `var`**。`VisualPoint` 的位置和法向就来自它。

---

## 2. 生命周期总览：`processFrame`

一帧图像进来，`VIOManager::processFrame`（[vio.cpp:1786-1834](../src/vio.cpp#L1786-L1834)）按固定 7 步走：

```text
processFrame（每帧固定 7 步，│ = 顺序执行）

 ① updateFrameState
      └─ 用当前 EKF 状态算相机位姿 T_cw
 │
 ② retrieveFromVisualSparseMap                        ◀── 怎么用(上)
      └─ 检索可见 VisualPoint → 选 ref_patch → 仿射 warp → 填 visual_submap
 │
 ③ computeJacobianAndUpdateEKF                        ◀── 怎么用(下)
      └─ 逐金字塔层，光度残差迭代更新 _state
 │
 ④ generateVisualMapPoints                            ◀── 怎么得到
      └─ 用当前 LiDAR 点新建 VisualPoint
 │
 ⑤ plotTrackedPoints / projectPatchFromRefToCur       （可视化）
 │
 ⑥ updateVisualMapPoints                              ◀── 维护
      └─ 给老点补新观测 Feature
 │
 ⑦ updateReferencePatch                               ◀── 维护
      └─ 用平面法向精化 normal_，NCC 选 ref_patch
```

- **第 2 步"检索"和第 3 步"更新" = 怎么用**；
- **第 4/6/7 步 = 怎么得到 / 怎么维护**。

下面按"得到 → 用 → 维护 → 销毁"展开。

> **关键参数默认值**（[LIVMapper.cpp:62-73](../src/LIVMapper.cpp#L62-L73) → [vio.cpp:150-154](../src/vio.cpp#L150-L154)）：
> `patch_size=8` ⟹ `patch_size_total=64`、`patch_size_half=4`；`patch_pyrimid_level=3` ⟹
> `warp_len=64×3=192`；`grid_size=5`；`normal_en=true`；`inverse_composition_en=false`；
> `exposure_estimate_en=true`；`max_iterations=5`；`img_point_cov=100`；`outlier_threshold=1000`。

---

## 3. VisualPoint 怎么"得到"：`generateVisualMapPoints`

[vio.cpp:804-906](../src/vio.cpp#L804-L906)。输入是当前帧的 LiDAR 世界点 `pg`。

### 3.1 网格化选点（每格留最强角点）

图像按 `grid_size` 划成网格。遍历 `pg`，把每个点投影到当前图像：

$$
\mathbf{p}_c = R_{cw}\,\mathbf{p}_w + P_{cw},\qquad
\mathbf{px} = \pi(\mathbf{p}_c)\ \ (\text{相机投影 } w2c)
$$

对落在图像内（[vio.cpp:816](../src/vio.cpp#L816)）、且所在网格**还没被已有地图点占用**
（`grid_num[index] != TYPE_MAP`，[vio.cpp:820](../src/vio.cpp#L820)）的点，算 **Shi-Tomasi 角点分数**
`vk::shiTomasiScore`（[vio.cpp:822](../src/vio.cpp#L822)），每格只保留分数最高的一个候选
（[vio.cpp:824-828](../src/vio.cpp#L824-L828)）。

> 这样做保证新点**稀疏且均匀**、且落在纹理强的地方（角点分数高 = 光度对齐更可靠），并且避开
> 已经有地图点的格子。

> **📄 论文 Section V-C**：候选点要求 (1) 当前帧可见、(2) 灰度梯度大；每个 voxel 只留投影**深度最小**
> 的候选（近处遮挡远处）。图像划分 **30×30 像素** 网格，空网格才用最高梯度候选建新点。候选来源：
> 成熟平面取**最近 50 个 LiDAR 点**、非成熟平面取全部点。⚠️ 代码里网格由 `vio/grid_size` 决定
> （默认 5，本配置未改），与论文的 30 略有出入——属可调参数。

### 3.2 建点、建首个观测、定法向

对每个入选格子（[vio.cpp:860-898](../src/vio.cpp#L860-L898)）：

1. **法向定向**：把 LiDAR 给的法向转到相机系，用观测方向点乘决定正负（法向要朝向相机）：
   $$
   \text{norm\_vec}=R_{cw}\,\mathbf{n},\quad \text{dir}=\widehat{R_{cw}\mathbf{p}_w+P_{cw}},\quad
   \cos\theta=\text{dir}\cdot\text{norm\_vec}
   $$
   $\cos\theta<0$ 就翻转法向（[vio.cpp:889-890](../src/vio.cpp#L889-L890)）。
2. `getImagePatch`（[vio.cpp:875](../src/vio.cpp#L875)）在投影像素处提取 patch。
3. `new VisualPoint(pt)`（[vio.cpp:877](../src/vio.cpp#L877)）——**这就是 VisualPoint 的诞生**。
4. `new Feature(...)`（[vio.cpp:880](../src/vio.cpp#L880)）记下 patch、像素 `px`、bearing `f`、
   当前相机位姿 `T_f_w_`、逆曝光；`addFrameRef` 挂到点的 `obs_`（[vio.cpp:885](../src/vio.cpp#L885)）。
5. `covariance_ = pt_var.var`、`is_normal_initialized_ = true`（[vio.cpp:886-887](../src/vio.cpp#L886-L887)）。
6. `insertPointIntoVoxelMap`（[vio.cpp:894](../src/vio.cpp#L894)）按 0.5 m voxel 塞进全局 `feat_map`
   （[vio.cpp:227-250](../src/vio.cpp#L227-L250)）。

> **关键点**：`VisualPoint` 的 3D 位置和法向都来自 **LiDAR**（`pg`），不是靠图像三角化。这正是
> LIVO 相比纯视觉的优势——深度直接由激光给定，视觉只负责精化位姿。

> **📄 论文 Section V-C**：每个 patch 是 **3 层金字塔**（论文示例 11×11，每层由上一层**减半采样**得到；
> 代码里 `patch_size=8`、层数 `patch_pyrimid_level=3`）。同时把**当前状态（位姿 + 曝光时间）连同法向**一并
> 记到该点上——这正是 `Feature` 里的 `T_f_w_` / `inv_expo_time_` 和 `VisualPoint::normal_`。

---

## 4. VisualPoint 怎么"用"（上）：检索 `retrieveFromVisualSparseMap`

[vio.cpp:352-782](../src/vio.cpp#L352-L782)。目标：从全局 `feat_map` 里挑出**这一帧能看到、且适合做
光度对齐**的点，连同 warp 好的参考 patch 一起放进 `visual_submap`。

### 4.1 第一步：建深度图 + 记录候选 voxel（[vio.cpp:386-428](../src/vio.cpp#L386-L428)）

把当前帧 LiDAR 点 `pg` 投影到图像，写一张 `depth_img`（每像素存深度），同时把这些点所在的 voxel
键存进 `sub_feat_map`。深度图后面用于**遮挡/深度不连续检测**。

> **📄 论文 Section VII-A.1（可见 voxel 查询）**：地图 voxel 极多，逐个判 FoV 太慢。论文的技巧是只
> 轮询两类 voxel——**当前扫描 LiDAR 点命中的 voxel**（相机/LiDAR FoV 大量重叠，视觉点多半也在这里），
> 以及**上一帧被判可见的点所在的 voxel**（相邻帧 FoV 高度重叠）。代码里"用当前 `pg` 建 `sub_feat_map`"
> 就是前者。

### 4.2 第二步：遍历候选 voxel，每格留最近的点（[vio.cpp:440-484](../src/vio.cpp#L440-L484)）

对 `sub_feat_map` 里每个 voxel，去全局 `feat_map` 找对应的 `VisualPoint` 列表，逐个：

- **朝向/正深度检查**：`dir = T_f_w_ * pos_`，要求 `dir[2] > 0`（点在相机前方，
  [vio.cpp:462](../src/vio.cpp#L462)）；注意：这里还有一个"法向-视线夹角 >80° 剔除"的门
  （`dir.dot(norm_vec)<=0.17`）但**被注释掉了**（[vio.cpp:464](../src/vio.cpp#L464)），当前不生效；
- 投影 `w2c` 落在图像内（[vio.cpp:467](../src/vio.cpp#L467)）；
- 标记该网格 `grid_num[index] = TYPE_MAP`，并在同格中**保留离相机最近的点**
  （`map_dist`，[vio.cpp:475-479](../src/vio.cpp#L475-L479)）——近点遮挡远点。

（可选 `raycast_en` 光线投射模块 [vio.cpp:487-599](../src/vio.cpp#L487-L599) 会沿射线补更多 voxel，
默认关闭。）

### 4.3 第三步：逐格构造残差数据（[vio.cpp:605-774](../src/vio.cpp#L605-L774)）

对每个 `TYPE_MAP` 网格里选中的 `pt`：

**(a) 深度连续性检查**（[vio.cpp:620-640](../src/vio.cpp#L620-L640)）：在投影点的 patch 邻域里查
`depth_img`，若邻域 LiDAR 深度与该点深度差 `> 0.5 m`，判为遮挡/边缘，**跳过**——避免把跨越深度断层
的 patch 拿去对齐。

> **📄 论文 Section VII-A.3（外点剔除，Fig. 8）**：可见点里有四类要剔除：① **被遮挡**（把子图点投影到
> 当前帧，每 30×30 网格只留深度最小的）；② **深度不连续**（与深度图 **9×9** 邻域比，即这里的邻域查
> `depth_img`）；③ **参考 patch 视角过大**；④ **当前帧视角过大**（③④ 阈值均 **>80°**）。代码里 ③④ 对应
> 那条 `dir.dot(norm_vec)<=0.17` 的 80° 门——但**当前被注释掉了**（§4.2），所以本版本只做了 ①②。

**(b) 选参考 patch `ref_ftr`**：
- `normal_en` 且只有 1 个观测：直接用它（[vio.cpp:659](../src/vio.cpp#L659)）；
- `normal_en` 且还没定 ref_patch：遍历所有 obs，选**与其它 obs 平均光度误差最小**的那个（最"典型"
  的 patch，[vio.cpp:663-691](../src/vio.cpp#L663-L691)）；
- `normal_en` 且已定：复用 `pt->ref_patch`（[vio.cpp:692](../src/vio.cpp#L692)）；
- **非** `normal_en`：`getCloseViewObs` 选**视角最接近当前帧**的观测
  （[vio.cpp:696](../src/vio.cpp#L696)，见 §7）。

**(c) 仿射 warp 矩阵 $A_{cur\_ref}$**：把参考帧的 patch 变换到当前帧视角（**完整"求 $A$ → 选层 → 重采样"见 §4.4**）。
- `normal_en`：用法向做**单应**`getWarpMatrixAffineHomography`（[vio.cpp:712](../src/vio.cpp#L712)）：
  $$
  H_{cur\_ref}=R_{cur\_ref}\big(\mathbf{n}^\top\mathbf{x}_{ref}\,I-\mathbf{t}\,\mathbf{n}^\top\big)
  $$
  （[vio.cpp:257-258](../src/vio.cpp#L257-L258)），再用它把 patch 四角映射过去、差分出 $A$。
- 非 `normal_en`：用深度做仿射 `getWarpMatrixAffine`（[vio.cpp:726](../src/vio.cpp#L726)），并把结果
  缓存进 `warp_map`。
- `getBestSearchLevel`（[vio.cpp:714](../src/vio.cpp#L714)）：按 $\det A$ 选合适的金字塔层
  （尺度变化大就用高层）。

> **📄 论文 Section V-E.1，eq (13)（仿射 warp，Fig. 5a）**：参考 patch 像素 $u_r^j$ 到目标帧像素
> $u_i^j$ 的仿射矩阵
> $$A_i^r=P\Big({}^{I_i}R_{I_r}+{}^{I_i}t_{I_r}\tfrac{{}^{I_r}n^\top}{{}^{I_r}n^\top\,{}^{I_r}p}\Big)P^{-1}$$
> 其中 $P/P^{-1}$ 是相机投影/反投影（换成 MEI/等距等模型即可**直接处理鱼眼**，无需先校正）。这与代码
> 的单应式 $H_{cur\_ref}$ 等价——都是**用平面法向**而非"常深度"来 warp。
>
> **📄 论文消融（Supplementary I-A）**：warp 用"常深度假设""平面先验""平面法向精化"三者对比——常深度
> 回环漂移 **0.22 m**，平面先验/法向精化都 **<0.01 m**，且地面文字、车道线渲染明显更清晰。这说明**用
> LiDAR 平面法向做 warp 是精度关键**（代码走的正是 `normal_en` 平面先验分支）。

**(d) 生成 warp 后的参考 patch**：`warpAffine` 逐金字塔层重采样出 `patch_wrap`
（[vio.cpp:739-742](../src/vio.cpp#L739-L742)；重采样细节见 §4.4③）。

**(e) 光度误差 + 外点剔除**：当前帧同位置取 patch `patch_buffer`，算误差
（[vio.cpp:746-751](../src/vio.cpp#L746-L751)）：
$$
\text{error}=\sum_k\big(\tfrac{1}{\tau_{ref}}\,P^{wrap}_k-\tfrac{1}{\tau_{cur}}\,I_k\big)^2
$$
（$\tau$ 为曝光时间，$1/\tau$=`inv_expo`）。可选 NCC 检查（[vio.cpp:753-761](../src/vio.cpp#L753-L761)）；
若 `error > outlier_threshold * patch_size_total` 判为外点、**跳过**（[vio.cpp:763](../src/vio.cpp#L763)）。

**(f) 入库**：通过的点把 `pt`、`patch_wrap`、`search_level`、`inv_expo`、`error` 推入 `visual_submap`
（[vio.cpp:765-770](../src/vio.cpp#L765-L770)）。最终 `total_points = visual_submap->voxel_points.size()`。

### 4.4 仿射 warp 详解：求 $A$ → 选层 → 重采样

**为什么需要它**：一个点的参考 patch 是过去某帧在**某个视角/距离**下拍的；当前帧从**不同位姿**看同一点，
投影会发生**旋转 + 缩放 + 错切（透视）**。若直接拿参考 patch 逐像素比当前图像会对不上、残差被视角差污染
（大基线、大视角、靠近/远离时尤其严重）。仿射 warp = **先把参考 patch 变换成"从当前视角看应有的样子"，
再比**，让直接法在大视角/大尺度变化下仍成立。小 patch 上平面诱导的单应可用一个 **2×2 仿射矩阵 $A$** 局部
线性近似，所以整个操作 = **① 求 $A$ → ② 选层 → ③ 重采样**。

**① 求仿射矩阵 $A_{cur\_ref}$**（`normal_en` 默认走 `getWarpMatrixAffineHomography`，[vio.cpp:252-273](../src/vio.cpp#L252-L273)）：
1. 建平面诱导单应（点在法向 $n$、过 $x_{ref}$ 的局部平面上，[vio.cpp:257-258](../src/vio.cpp#L257-L258)）：
   $$H_{cur\_ref}=R_{cur\_ref}\big((n^\top x_{ref})\,I-t\,n^\top\big)$$
2. 取参考 patch **中心** `px_ref` 和两个偏移点 `px_ref+[k,0]`、`px_ref+[0,k]`，各自 反投影(`cam2world`)
   → 过 $H$ → 投回当前图像(`world2cam`)；
3. 两条像素差分即 $A$ 的两列（[vio.cpp:271-272](../src/vio.cpp#L271-L272)）：
   $$A.\text{col}(0)=\frac{px^{du}_{cur}-px_{cur}}{k},\qquad A.\text{col}(1)=\frac{px^{dv}_{cur}-px_{cur}}{k}$$
   即 $A$ = "参考像素微小位移 → 当前像素位移"的**雅可比**，一次抓住缩放+旋转+错切。

> 非 `normal_en` 走 `getWarpMatrixAffine`（[vio.cpp:275-290](../src/vio.cpp#L275-L290)）：用**点的深度**
> 代替法向来算 offset，即"常深度假设"，斜面上不准（消融里被打败的那个）；结果按 `ref_ftr->id_` 缓存进
> `warp_map` 复用。

**② 选金字塔层**（`getBestSearchLevel`，[vio.cpp:320-331](../src/vio.cpp#L320-L331)）：`D = det(A)` 是
面积缩放比，`while (D > 3) { level++; D *= 0.25; }`。视角带来的尺度变化大，就去**更粗的金字塔层**采样，
使 warp 后 patch ≈ 单位面积，**防走样/混叠**。

**③ 重采样**（`warpAffine`，[vio.cpp:292-318](../src/vio.cpp#L292-L318)）：用**逆向 warp**（遍历输出网格、
反查输入，避免空洞），逐金字塔层生成 `patch_wrap`：
```text
A_ref_cur = A_cur_ref.inverse()               // 当前 patch 网格位移 → 参考图像位移
for 每个 patch 像素 (x, y):
    px_patch = (x - half, y - half) * 2^search_level * 2^pyramid_level
    px = A_ref_cur * px_patch + px_ref          // 该去参考图像哪里取值
    patch_wrap[...] = 双线性插值(ref_img, px)    // vk::interpolateMat_8u
```

**去向**：`patch_wrap` 存进 `visual_submap->warp_patch`（§4.3f），随后在 §5.1 的光度残差里当参考
$P^{wrap}$ 用。**一句话：把"不同视角拍的参考 patch"对齐到当前视角，使逐像素光度比较成立；用 LiDAR
平面法向算 $A$ 是精度关键（消融 0.22 m → <0.01 m，见 §4.3c 论文注）。**

---

## 5. VisualPoint 怎么"用"（下）：光度 EKF 更新

`computeJacobianAndUpdateEKF`（[vio.cpp:784-802](../src/vio.cpp#L784-L802)）**从高到低**遍历金字塔层，
每层调 `updateState`（[vio.cpp:1520-1688](../src/vio.cpp#L1520-L1688)）做**迭代**误差状态卡尔曼更新
（由粗到精，避免陷入局部极小）。

### 5.1 光度残差

用当前 EKF 位姿把点投到当前帧，取 patch，与 warp 好的参考 patch 逐像素比（曝光归一化）：

$$
r_k=\frac{1}{\tau_{cur}}\,I\big(\pi(R_{cw}\mathbf{p}_w+P_{cw})\big)_k-\frac{1}{\tau_{ref}}\,P^{wrap}_k
$$

代码 [vio.cpp:1621](../src/vio.cpp#L1621)：`res = state->inv_expo_time * cur_value - inv_ref_expo * P[...]`。

其中相机位姿由 IMU 状态和外参得到（每次迭代重算，[vio.cpp:1542-1543](../src/vio.cpp#L1542-L1543)）：
$$
R_{cw}=R_{ci}R_{wi}^\top,\qquad P_{cw}=-R_{ci}R_{wi}^\top P_{wi}+P_{ci}
$$

> **📄 论文 Section VII-B，eq (21)-(22)**：理想情况下把地图点 ${}^Gp_i$ 投到当前帧，参考 patch 与当前
> patch 的光度差应为零：
> $$0=\tau_k I_k\big(\pi({}^C T_I({}^G T_I)^{-1}\,{}^Gp_i)+\Delta u\big)-\tau_r I_r\big(\pi({}^{C_r}T_G\,{}^Gp_i)+A_i^r\Delta u\big)$$
> $\Delta u$ 是 patch 内相对中心的像素偏移，$A_i^r$ 是把当前 patch 映射到参考 patch 的仿射矩阵（§4.3c）。
> $\tau_k,\tau_r$ 是当前帧/参考帧的**逆曝光时间**——正是这一项让系统能跨曝光变化做光度对齐，代码里就是
> `state->inv_expo_time` 与 `inv_ref_expo`。曝光时间 $\tau$ 作为**状态量**一起被 EKF 估计，并固定首帧
> $\tau_0=1$ 以消除"全为零"的退化。

### 5.2 雅可比链

对每个像素，残差对状态的雅可比按链式法则拆三段（[vio.cpp:1600-1617](../src/vio.cpp#L1600-L1617)）：

| 段 | 公式 | 代码 |
|---|---|---|
| 图像梯度（含曝光/尺度） | $J_{img}=\frac{1}{\tau}\,[\,du,\ dv\,]$ | [vio.cpp:1611-1613](../src/vio.cpp#L1611-L1613) |
| 像素对相机点 $\partial\pi/\partial\mathbf{p}_c$ | $J_{dpi}=\begin{bmatrix}f_x/z&0&-f_xx/z^2\\0&f_y/z&-f_yy/z^2\end{bmatrix}$ | `computeProjectionJacobian`, [vio.cpp:189-201](../src/vio.cpp#L189-L201) |
| 相机点对相机旋转 | $J_{d\phi}=J_{img}J_{dpi}\,[\mathbf{p}_c]_\times$ | [vio.cpp:1614](../src/vio.cpp#L1614) |
| 相机点对相机平移 | $J_{dp}=-J_{img}J_{dpi}$ | [vio.cpp:1615](../src/vio.cpp#L1615) |

再由**相机误差态 → IMU 误差态**（通过外参 $T_{ci}$）：
$$
J_{dR}=J_{d\phi}\,\underbrace{R_{ci}}_{J_{d\phi\_dR}}+J_{dp}\,\underbrace{(-R_{ci}[P_{ic}]_\times)}_{J_{dp\_dR}},\qquad
J_{dt}=J_{dp}\,\underbrace{R_{ci}R_{wi}^\top}_{J_{dp\_dt}}
$$

代码 [vio.cpp:1616-1617](../src/vio.cpp#L1616-L1617)；外参项在 `initializeVIO` 里预算
（[vio.cpp:57-65](../src/vio.cpp#L57-L65)：`Jdphi_dR=Rci`、`Jdp_dR=-Rci*skew(Pic)`）。

每个像素给 `H_sub` 贡献一行 `[JdR, Jdt, (cur_value)]`（最后一列是曝光项，
[vio.cpp:1628-1629](../src/vio.cpp#L1628-L1629)）。

### 5.3 卡尔曼更新与迭代

观测矩阵 `H_sub` 尺寸是 `(total_points·patch_size_total) × 7`（[vio.cpp:1533](../src/vio.cpp#L1533)）：
每个像素一行 `[JdR(3), Jdt(3), cur_value(1)]`，**第 7 列是曝光项**（`exposure_estimate_en` 关闭时
只用前 6 列，[vio.cpp:1628-1629](../src/vio.cpp#L1628-L1629)）。

组正规方程并更新（[vio.cpp:1657-1669](../src/vio.cpp#L1657-L1669)）：

$$
K_1=\Big(H^\top H+(\Sigma/\sigma_{img})^{-1}\Big)^{-1},\quad
\delta x=-K_1 H^\top z+\text{vec}-G\,\text{vec}_{[0:7]}
$$

其中 `vec = state_propagat - state`（回到预测值的量，迭代 EKF 的先验耦合项），
`state += δx`（[vio.cpp:1669](../src/vio.cpp#L1669)）。`δx` 是 19 维，但只有 0-6 维有观测贡献；
`operator+=`（[common_lib.h:193-203](../include/common_lib.h#L193-L203)）把各段写回状态：
**0-2→`rot_end`（`Exp`）、3-5→`pos_end`、6→`inv_expo_time`**、7-9→`vel_end`、10-12→`bias_g`、
13-15→`bias_a`、16-18→`gravity`。

- **误差不降就回退** `old_state`、结束本层（[vio.cpp:1677-1681](../src/vio.cpp#L1677-L1681)）；
- **收敛判据**：旋转增量 `<0.001°` 且平移增量 `<0.001`（[vio.cpp:1675](../src/vio.cpp#L1675)）；
- 全部层做完，`state->cov -= G*state->cov`（[vio.cpp:800](../src/vio.cpp#L800)），并
  `updateFrameState` 把最终相机位姿写回 `new_frame_`（[vio.cpp:801](../src/vio.cpp#L801)）。

> **前向 vs 逆向组合**：`inverse_composition_en` 打开时走 `updateStateInverse`
> （[vio.cpp:1398-1518](../src/vio.cpp#L1398-L1518)）：雅可比在**参考帧**一次性预算
> （`precomputeReferencePatches`，[vio.cpp:1327-1396](../src/vio.cpp#L1327-L1396)，梯度取自 ref patch），
> 迭代中只用当前 `Rwi` 旋转一下即可，更快；代价是 `H_sub` 只有 6 列（**不估曝光**）、残差也不做曝光归一化。
> 前向法（默认）每次迭代都从**当前图像**重算梯度、且带曝光项，更准。

> **📄 论文 Section IV-D，eq (11)（ESIKF 顺序更新）**：卡尔曼增益与迭代更新为
> $$K=\big(H_\kappa^\top R^{-1}H_\kappa+\hat P^{-1}\big)^{-1}H_\kappa^\top R^{-1},\quad
> \hat x_{\kappa+1}=\hat x_\kappa\boxplus\big(-Kz_\kappa-(I-KH_\kappa)(\hat x_\kappa\boxminus\hat x)\big)$$
> 代码里的 $K_1=(H^\top H+(\Sigma/\sigma_{img})^{-1})^{-1}$ 是它的等价形式（$R=\sigma_{img}I$，
> $\hat P=\Sigma$），`vec = state_propagat - state` 就是 $(\hat x_\kappa\boxminus\hat x)$ 那一项。
> **"顺序更新"** = 一次 ESIKF 里**先用 LiDAR 点到面残差迭代收敛，再用视觉光度残差在其结果上继续迭代**
> （论文 Algorithm 1：LiDAR 更新 Line 6-10，视觉更新 Line 13-23，逐金字塔层），对应代码 `handleLIO`→
> `handleVIO` 的先后顺序。
>
> **📄 逆向组合，eq (23)**：把位姿增量参数化为 ${}^GT_I={}^G\hat T_I\,\mathrm{Exp}(\delta T)$ 并从 $u_i$
> 移到 $u_i'$，使雅可比只需**在参考帧算一次**、迭代中不再重算——即 `updateStateInverse` 的做法。

---

## 6. VisualPoint 怎么"维护"

### 6.1 补观测：`updateVisualMapPoints`（[vio.cpp:908-967](../src/vio.cpp#L908-L967)）

位姿更新后，决定要不要给这一帧再给点**加一个新的 `Feature` 观测**：

- 已收敛点（`is_converged_`）：`deleteNonRefPatchFeatures` 只留 ref_patch，省内存
  （[vio.cpp:920](../src/vio.cpp#L920)）；
- **加观测判据**（满足其一，[vio.cpp:934-944](../src/vio.cpp#L934-L944)）：相对上次观测的
  平移 `Δp>0.5 m` 或旋转 `Δθ>0.3 rad`，或像素移动 `>40 px`——即视角/位置变化够大，新 patch 才有信息量；
- **限制观测数**：`obs_.size() >= 30` 时用 `findMinScoreFeature` 删掉分数最低的观测
  （[vio.cpp:947-953](../src/vio.cpp#L947-L953)）；
- 满足条件就 `new Feature + addFrameRef`（[vio.cpp:958-963](../src/vio.cpp#L958-L963)）。

> **📄 论文 Section V-C（加 patch 判据）**：论文写的是"距上次加 patch **超过 20 帧**，或像素位置偏移
> **>40 px**"就加新 patch；目的是让每个点积累**视角均匀分布**的观测。代码把"20 帧"换成了**位姿增量**
> 判据（$\Delta p>0.5\,\mathrm{m}$ 或 $\Delta\theta>0.3\,\mathrm{rad}$），"40 px"一致——效果等价（都在
> 视角/位置变化够大时才加），只是触发量从帧数改成了实际运动量。

### 6.2 精化法向 + 选 ref_patch：`updateReferencePatch`（[vio.cpp:969-1100](../src/vio.cpp#L969-L1100)）

只处理**法向已初始化、未收敛、观测数 >5、本帧刚补过观测**的点
（[vio.cpp:977-980](../src/vio.cpp#L977-L980)）。

**法向精化**（[vio.cpp:990-1032](../src/vio.cpp#L990-L1032)）：到 LiDAR 的 `plane_map` 找该点所在
voxel 的平面，做**平面一致性 3σ 检验**（点到平面距离 vs $\sqrt{\sigma_l}$，
[vio.cpp:1011](../src/vio.cpp#L1011)），通过就把点的法向对齐到平面法向（保持与 `previous_normal_`
同向，[vio.cpp:1019-1020](../src/vio.cpp#L1019-L1020)）。若法向变化 `<0.0001` 且观测 `>10`，标记
`is_converged_ = true`（[vio.cpp:1026-1028](../src/vio.cpp#L1026-L1028)）。

> **📄 论文 Section V-E.2，eq (14)-(16)（法向优化）**：论文在 LiDAR 平面先验之上**再做一步光度优化**——
> 通过最小化参考 patch 与其它 patch 的光度误差来精修法向：
> $$ {}^{I_r}n^*=\arg\min_{{}^{I_r}n\in S^2}\sum_{i\in S}\sum_j\big\|\tau_i I_i(A_i^r u_r^j)-\tau_r I_r(u_r^j)\big\|^2$$
> 并把带约束的 ${}^{I_r}n$ 重参数化为无约束的 $m\in\mathbb R^2$（eq 15-16，`M=Bm+b`），放在**独立线程**里
> 解、不阻塞里程计。收敛后**固定**参考 patch 和法向、删除其它 patch。
> **⚠️ 本代码未实现这一步**：`updateReferencePatch` 只把 `normal_` **对齐到 LiDAR 平面法向**（即论文的
> "Plane prior"），"收敛"指的是 LiDAR 平面法向随点云成熟而**稳定不再变**，不是 eq(14-16) 的光度最小化。
> 消融里 Plane prior 与 Plane normal refined 漂移都 <0.01 m，所以缺这步影响有限。

**选 ref_patch**（[vio.cpp:1036-1097](../src/vio.cpp#L1036-L1097)）：对每个 obs 打分
$$
\text{score}=\text{NCC}+\cos\text{angle}
$$
（NCC=与其它 obs 的平均归一化互相关，cos_angle=观测视角与法向夹角；[vio.cpp:1087](../src/vio.cpp#L1087)），
选 score 最大的 obs 作 `ref_patch`（[vio.cpp:1091-1096](../src/vio.cpp#L1091-L1096)）。

> 直觉：ref_patch 要**和多数观测都像**（NCC 高）且**正对该点表面**（cos_angle 大），这样后续 warp
> 和残差最稳。

> **📄 论文 Section V-D，eq (12)（参考 patch 打分）**：完整评分是加权和
> $$ S=(1-\omega_1)\cdot\tfrac1n\sum_{i=1}^n \mathrm{NCC}(f,g_i)+\omega_1\cdot c,\quad
> c=\tfrac{n\cdot p}{\|p\|},\quad \omega_1=\tfrac{1}{1+e^{\mathrm{tr}(\Sigma_n)}}$$
> $c$ 是 patch 视线与法向的夹角余弦（正对平面时 $c=1$），$\omega_1$ 由**法向协方差的迹** $\mathrm{tr}(\Sigma_n)$
> 自适应决定（法向越不确定，越少信 $c$）。选 $S$ 最大的 patch 作参考——这套机制刻意选**与多数 patch 相似
> （抗动态物体，借鉴 MVS）且正对平面（保留高分辨率纹理）**的 patch。⚠️ 代码把它简化成**等权** `NCC + c`
> （[vio.cpp:1087](../src/vio.cpp#L1087)），没有 $\omega_1$ 自适应权重。
>
> 论文也强调：这比 FAST-LIVO(v1) 和前人"直接选与当前帧视角最接近的 patch"更好——后者会让参考帧太贴近
> 当前帧、对位姿约束很弱（正是本文非 `normal_en` 分支 `getCloseViewObs` 的老做法，见 §8）。

---

## 7. VisualPoint 的销毁与内存管理

| 操作 | 位置 | 作用 |
|---|---|---|
| `~VisualPoint` | [visual_point.cpp:24-32](../src/visual_point.cpp#L24-L32) | `delete` 掉 `obs_` 里所有 `Feature` |
| `~Feature` | [feature.h:49-53](../include/feature.h#L49-L53) | `delete[] patch_` |
| `deleteFeatureRef` | [visual_point.cpp:39-55](../src/visual_point.cpp#L39-L55) | 删单个观测；若是 ref_patch 则清空 ref |
| `deleteNonRefPatchFeatures` | [visual_point.cpp:113-126](../src/visual_point.cpp#L113-L126) | 收敛点只留 ref_patch |
| `addFrameRef` | [visual_point.cpp:34-37](../src/visual_point.cpp#L34-L37) | `obs_.push_front(ftr)` 加观测 |

`VisualPoint` 本身进 `feat_map` 后**长期存活**（跨帧复用），地图滑窗/清理由 LiDAR 侧的
voxel 管理触发；`Feature` 观测则按 §6.1 的上限（30 个）滚动淘汰。

---

## 8. 两个关键辅助函数

### `getCloseViewObs`（[visual_point.cpp:57-95](../src/visual_point.cpp#L57-L95)）
非 `normal_en` 时选参考 patch：在所有 obs 里找**观测方向与当前帧最接近**的（方向余弦最大）：
$$
\text{obs\_dir}=\widehat{\text{framepos}-\mathbf{pos}},\quad \max_i\ \text{obs\_dir}\cdot\widehat{(\mathbf{c}_i-\mathbf{pos})}
$$
夹角 `>60°`（cos<0.5）就放弃（[visual_point.cpp:88](../src/visual_point.cpp#L88)）——视角差太大 patch 不可比。

### `findMinScoreFeature`（[visual_point.cpp:97-111](../src/visual_point.cpp#L97-L111)）
线性扫 `obs_` 找 `score_` 最小的观测，供 §6.1 超限淘汰。

---

## 9. 速查：VisualPoint 全生命周期

| 阶段 | 函数 | 对 VisualPoint 做了什么 |
|---|---|---|
| **生** | `generateVisualMapPoints` | LiDAR 点→`new VisualPoint`，定位置/法向，建首个 `Feature`，入 `feat_map` |
| **检索** | `retrieveFromVisualSparseMap` | 挑可见点、选 ref_patch、仿射 warp、外点剔除 → `visual_submap` |
| **用** | `updateState`（EKF） | 用 `pos_`+warp 后 ref patch 做光度残差，反解并更新 `_state` |
| **补观测** | `updateVisualMapPoints` | 视角变化够大就 `addFrameRef` 新 `Feature`；超 30 个淘汰最低分 |
| **精化** | `updateReferencePatch` | 用平面精化 `normal_`、判 `is_converged_`、NCC 选 `ref_patch` |
| **灭** | `~VisualPoint` / `deleteFeatureRef` | 释放观测与 patch 内存 |

---

## 10. 代码 vs 论文：一致点与差异

对照论文 [2408.14035v2.pdf](../Supplementary/2408.14035v2.pdf) 通读后，本代码的实现要点：

**完全一致的部分**
- 视觉点由 **LiDAR 点 + 高梯度**生成，带 3 层 patch 金字塔（V-C）；
- warp 用**平面法向做单应/仿射**（V-E.1，eq 13），而非常深度假设；
- 光度残差**逐像素、带逆曝光归一化**，逐金字塔层由粗到精（VII-B，eq 21-22）；
- ESIKF **顺序更新**（先 LiDAR 后视觉），增益即 eq (11) 的等价形式（IV-D）；
- 外点剔除的**遮挡 + 深度不连续**（VII-A.3 的 ①②，9×9 深度邻域）；
- 逆向组合可选项（eq 23，`updateStateInverse`）。

**代码相对论文的简化 / 差异**

| # | 论文 | 本代码 | 影响 |
|---|---|---|---|
| 1 | **法向光度优化** eq (14)-(16)，独立线程（"Plane normal refined"） | **未实现**；`updateReferencePatch` 只对齐到 LiDAR 平面法向（"Plane prior"） | 小：消融里两者漂移都 <0.01 m |
| 2 | 参考 patch 打分 eq (12)：$\omega_1=\frac{1}{1+e^{\mathrm{tr}(\Sigma_n)}}$ 自适应加权 | 等权 `NCC + cos`（[vio.cpp:1087](../src/vio.cpp#L1087)） | 小：仍选"相似且正对平面"的 patch |
| 3 | 外点剔除 ③④：参考/当前**视角 >80° 剔除**（VII-A.3） | 该 80° 门**被注释掉**（[vio.cpp:464](../src/vio.cpp#L464)） | 中：大视角 patch 未被剔除，可能引入弱约束点 |
| 4 | 加 patch 判据："超 20 帧 或 >40 px"（V-C） | 改为位姿增量 $\Delta p>0.5$ 或 $\Delta\theta>0.3$ 或 >40 px | 等价：触发量从帧数换成运动量 |
| 5 | 生成网格 **30×30 px**（V-C） | `vio/grid_size`（默认 5，本配置未改） | 需注意：网格更细，视觉点更密 |
| 6 | 鱼眼直接投影（MEI/等距，eq 13 的 $P/P^{-1}$） | 本工程走**整幅去畸变 + 纯针孔**（见 LIVMapper 去畸变）；鱼眼分支未走 | 取决于相机：针孔场景无影响 |

> 一句话：**核心 pipeline 与论文一致**（LiDAR 先验深度/法向 + 平面 warp + 逆曝光光度直接法 + 顺序 ESIKF），
> 精化环节（法向光度优化、自适应打分权重、视角门）做了工程简化。要进一步提精度，最值得补的是 **#3 视角门**
> 和 **#1 法向光度优化**。

---

## 附：数据流一图

```text
   LiDAR pg (point_w, normal, var)
        │  generateVisualMapPoints（新建点）
        ▼
   feat_map ────────── 全局持久 VisualPoint（0.5m voxel）
        │  retrieveFromVisualSparseMap        ◀── 当前图像
        ▼
   visual_submap ───── 本帧点 + warp 后的参考 patch
        │  updateState（光度残差迭代）
        ▼
   _state (IMU 位姿) ── updateFrameState ──▶ new_frame_ T_cw
        ▲                                         │
        └──────── 反过来用于投影 / warp ◀─────────┘

   维护回写 feat_map：
     updateVisualMapPoints  ◀─ 当前图像        （补观测 Feature）
     updateReferencePatch   ◀─ LiDAR plane_map （精化法向、选 ref_patch）
```
