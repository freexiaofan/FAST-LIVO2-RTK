# FAST-LIVO2-RTK 的 VoxelMap 数据结构详解

> 代码位置：`include/voxel_map.h` + `src/voxel_map.cpp`（类 `VoxelOctoTree`、`VoxelMapManager`、`VoxelPlane`）
> 本文只讲**地图数据结构本身**——怎么建、怎么增量更新、怎么查询、怎么滑窗删除；`StateEstimation()` 里"怎么用这些平面构建 IEKF 残差/权重"的完整数学推导，已经在 [docs/VOXELMAPMANAGER_ITERATION_DATAFLOW.md](docs/VOXELMAPMANAGER_ITERATION_DATAFLOW.md) 和 [docs/UNCERTAINTY_COMPUTE_AND_USAGE.md](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md) 里逐行核实过，本文不重复，只做必要的交叉引用。整体项目笔记见 [FAST-LIVO2-RTK-原理与代码笔记.md](FAST-LIVO2-RTK-原理与代码笔记.md) §5。

---

## 0. 一句话总览

**地图 = 一个哈希表（体素坐标 → 一棵八叉树），每棵八叉树的叶子节点要么是一个拟合好的局部平面，要么因为不够平面而继续往下细分。** 和 FAST_LIO 用的 ikd-Tree（单一一棵扁平 kd 树，惰性删除+按需重建）是完全不同的两套地图组织思路。

```
voxel_map_ : unordered_map<VOXEL_LOCATION, VoxelOctoTree*>
                     │
        每个 key 是一个"根体素"（边长 max_voxel_size_，比如 0.5m）
                     │
              VoxelOctoTree（可能还没建好/是叶子/继续细分）
                     ├─ octo_state_==0（叶子）：plane_ptr_ 指向一个 VoxelPlane（可能 is_plane_=false，表示"不是平面但也不再细分了"）
                     └─ octo_state_==1（内部节点）：leaves_[0..7] 八个子节点（八叉树细分）
```

---

## 1. 体素定位：`VOXEL_LOCATION` + 哈希（`voxel_map.h:102-124`）

```cpp
class VOXEL_LOCATION { public: int64_t x, y, z; ... };
```

任意世界坐标点，除以体素边长 `max_voxel_size_` 再取整（负数额外减 1，避免 `(int)(-0.3)==0` 导致原点两侧的体素错误重合），就得到它所在的整数体素坐标 `(x,y,z)`：

```cpp
loc_xyz[j] = p_w[j] / voxel_size;
if (loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
VOXEL_LOCATION position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
```

配自定义哈希函数（`voxel_map.h:113-124`，经典的"逐维乘质数取模再叠加"哈希组合方式），塞进 `std::unordered_map`，就能把"世界坐标属于哪个体素"这个查询降到平均 O(1)。

---

## 2. 单个体素内部：`VoxelOctoTree`（多层八叉树）

### 2.1 字段含义（`voxel_map.h:135-172`）

| 字段 | 含义 |
|---|---|
| `temp_points_` | 还没被"消耗掉"的缓存点（等着凑够数拟合/重新拟合平面用） |
| `plane_ptr_` | 指向这个节点自己的 `VoxelPlane`（不管是不是叶子节点都有一个，方便统一判断） |
| `layer_` / `max_layer_` | 当前在第几层 / 整棵树最多细分到第几层（config `lio/max_layer`，比如 2） |
| `octo_state_` | 0=已经是叶子（不再细分）；1=内部节点（还有子节点） |
| `leaves_[8]` | 八个子节点指针（八叉树，按 x/y/z 各自"大于中心"与否编码成 0~7） |
| `points_size_threshold_` | 这一层要攒够多少个点才**第一次**尝试拟合平面（对应 config `layer_init_num_`，各层可以给不同值，比如 `[5,5,5,5,5]`） |
| `update_size_threshold_` | 硬编码 **5**——平面拟合好之后，每新来 5 个点就重新拟合一次平面（更新法向/协方差） |
| `max_points_num_` | 平面缓存点数上限（config `lio/max_points_num`，比如 50），超过就冻结这个平面不再更新 |
| `update_enable_` | 是否还允许继续用新点更新这个平面（冻结后置 false） |

### 2.2 判断"是不是平面"：`init_plane()`（`voxel_map.cpp:68`）

对缓存的点做 **PCA**（和之前聊过的"点云协方差矩阵特征值分解"是同一套数学）：

1. 算这批点的**质心** `center_` 和**协方差矩阵** `covariance_`。
2. 特征值分解，取**最小特征值**对应的特征向量当**法向** `normal_`，最大特征值对应的当 `x_normal_`（平面内的"长轴"方向）。
3. **平面判据**：`min_eigen_value < planer_threshold_`（config `lio/min_eigen_value`，比如 0.0025）——最小特征值足够小，说明这批点在法向方向"够扁"，判定为平面（`is_plane_=true`）；否则判定"不是平面"（`is_plane_=false`）。
4. **`radius_ = sqrt(max_eigen_value_)`**——平面patch 的"半径"，用最大特征值（平面内最分散的方向）的平方根来近似这批点在平面内摊开的范围，后面查询候选平面时用来判断"这个点离这个平面patch 的中心是不是足够近，值不值得去比对"。
5. 顺带算 `plane_var_`（6×6 平面参数协方差，前3维法向+后3维中心点），这部分的雅可比推导已经在 [UNCERTAINTY_COMPUTE_AND_USAGE.md §2.3](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md) 逐行核实过，不重复。

**只有 4 个字符的差别决定了整套地图的行为**：`is_plane_` 这个布尔值——建树/查询/更新的所有分支逻辑都是在问"这个节点现在是不是一个平面"。

### 2.3 建树期的细分：`init_octo_tree()` + `cut_octo_tree()`（`:150-230`）

```cpp
void init_octo_tree() {
  if (temp_points_.size() > points_size_threshold_) {
    init_plane(temp_points_, plane_ptr_);
    if (plane_ptr_->is_plane_) { octo_state_ = 0; /* 点数超上限就冻结，见下 */ }
    else { octo_state_ = 1; cut_octo_tree(); }   // 不是平面 → 细分成8个子节点
  }
}
```

`cut_octo_tree()` 把缓存点按"是否大于体素中心"分到 8 个子体素（每维 0/1，`4*x+2*y+z` 编码成 0~7），每个非空子节点**递归**再判断一次"够点数了就拟合，是平面就停、不是就继续细分"——直到判定为平面，或者已经细分到 `max_layer_` 层（强制停止，不管是不是平面）。

**含义**：地图不是"处处都用同样精度的体素"，而是**局部自适应**——平坦区域（比如地面、墙面）用大体素就能拟合出平面，很快停止细分；复杂/带边缘的区域会一层层细分成更小的子体素，直到局部足够平坦或者细分到头。

> **细分到 `max_layer_` 还不是平面，这些点会怎样？——建图阶段"留着但闲置"**：`cut_octo_tree()` 命中 `layer_ >= max_layer_` 时直接 `return`（`voxel_map.cpp:178-182`），**没有清空 `temp_points_`**——这批点原样留在这个叶子节点里，只是 `octo_state_` 被标成"不再细分"、`plane_ptr_->is_plane_` 保持 `false`。因为 `find_correspond()`/`build_single_residual()` 只认 `is_plane_==true` 的节点，这些点之后**不会被任何查询用到**，纯粹占内存、不参与残差构建。增量更新阶段（`UpdateOctoTree()`）对这种"到头仍不是平面"的节点行为不一样，见下一节末尾。

### 2.4 增量更新：`UpdateOctoTree()`（`:232-303`）——这是在线建图实际走的路径

```cpp
if (!init_octo_) {
    // 这个节点还从没拟合过 → 攒点，攒够 points_size_threshold_ 就调 init_octo_tree() 首次拟合/细分
}
else if (plane_ptr_->is_plane_) {
    if (update_enable_) {
        temp_points_.push_back(pv);  new_points_++;
        if (new_points_ > update_size_threshold_ /*5*/) { init_plane(temp_points_, plane_ptr_); new_points_=0; }  // 每5个新点重新拟合一次
        if (temp_points_.size() >= max_points_num_) { update_enable_=false; 释放temp_points_; }  // 点数够多就冻结
    }
    // update_enable_==false 时：新点直接被忽略，这个平面参数永久定格
}
else {
    // 不是平面：还没到 max_layer_ 就按 UpdateOctoTree 递归下探/新建子节点；
    // 已经到 max_layer_（细分到头还不是平面）就退化成"和叶子一样攒点、定期 init_plane"的逻辑
}
```

**三个关键设计点**：
1. **平面拟合"随点数增多定期刷新"，不是每来一个点都重算**（`update_size_threshold_=5`）——省计算量，5 个点的误差不足以让平面参数明显漂移。
2. **点数够多（`max_points_num_`）就"冻结"**——平面已经用足够多的点拟合得很稳定了，继续囤积新点收益递减，还占内存，干脆停止更新、把缓存点释放掉（`std::vector<pointWithVar>().swap(temp_points_)` 是清空+释放容量的惯用写法）。这意味着**地图里存活很久的平面，参数会在某一刻彻底定格，之后即使这片区域被反复扫到也不会再变**。
3. **"不是平面"的节点，只要没到 `max_layer_`，收到新点就继续往下递归细分**——建图和更新用的是**同一套判定逻辑**（`init_plane`/平面判据），没有"建图一套算法、更新另一套算法"的割裂。
4. **"到 `max_layer_` 了还不是平面"，增量更新阶段不是直接放弃**：只要 `update_enable_` 还是 `true`，新点会继续塞进 `temp_points_`，且**每凑够 5 个新点（`update_size_threshold_`）就重新尝试一次 `init_plane()`**——万一后续观测让这批点分布变得更平坦，这个节点仍有机会翻身变成 `is_plane_=true`。只有当 `temp_points_.size() > max_points_num_` 还是拟合不出平面时，才会 `update_enable_=false` 并立即 `swap` 清空——**这时候点才是真正被丢弃（物理释放），之后再有新点落到这个位置会被无声忽略（连 push_back 都不做）**。这和上面"建图阶段到头直接留着不管"的行为不一样：建图阶段是"一次性判定、之后不再理会"，增量更新阶段是"持续重试、直到点数上限才彻底放弃"。

### 2.5 查询：`find_correspond()`（给 VIO 用）vs `build_single_residual()`（给 LIO 用）

- **`find_correspond(pw)`**（`:305`）：只做**一次定位**——从根节点开始，是叶子就返回自己，不是就按点坐标算该往哪个子节点走，递归下探，找到"点所在的那个叶子节点"就返回，不做任何距离/概率比较。这是 VIO 侧 `updateReferencePatch()` 精化视觉点法向时用的（见 [VISUAL_POINT.md](docs/VISUAL_POINT.md) §6.2）——只需要"这个点大致在哪个平面附近"，不需要严格的统计验证。
- **`build_single_residual()`**（`:790`，LIO 侧用）：不是简单定位，而是**递归遍历所有非空子节点**（一个点可能同时落在"父节点还没来得及标记为不是平面"的边界上，或者需要在多个候选子树里比较），对每个遇到的、**已经是平面**的叶子节点，做三重检验（`range_dis <= 3×radius_` 半径粗筛 → `σ` 门限统计检验 → 概率择优保留最大似然的那个），这一套"候选筛选+概率择优"的完整数学在 [UNCERTAINTY_COMPUTE_AND_USAGE.md §4](docs/UNCERTAINTY_COMPUTE_AND_USAGE.md) 已经讲过，这里补充的是**它在哪一层代码结构里发生**：不是"查一次给一个确定答案"，而是"递归访问所有候选叶子、每个都验证一遍、留下最可信的那个"。

---

## 3. 整棵地图的生命周期：建、增量更新、滑窗删除

### 3.1 建图（`BuildVoxelMap()`，只在第一帧调用一次，`voxel_map.cpp:604`）

```
对这一帧所有降采样点：
  1. 算每个点的世界系协方差 pv.var（body系测距噪声 + 姿态/位置不确定性投影，见 UNCERTAINTY 文档 §3）
  2. 按体素坐标分组，同一体素的点先塞进同一个 VoxelOctoTree 的 temp_points_（还没建树）
  3. 每个新出现的体素，new 一个 VoxelOctoTree(max_layer, layer=0, ...)，登记进 voxel_map_ 这个哈希表
最后统一遍历一遍所有体素，调用 init_octo_tree() 做首次拟合/细分
```

**"先分组、最后统一 init_octo_tree()"**（而不是每加一个点就判断一次）是因为第一帧点数很多，边插入边判断会导致同一个体素被反复拟合很多次，不如攒齐了再拟合一次划算。

### 3.2 增量更新（`UpdateVoxelMap()`，`:681`，每帧调用）

结构和 `BuildVoxelMap()` 几乎一样（同样是先按体素坐标定位、体素不存在就新建），唯一区别是：**建图阶段直接塞进 `temp_points_`、最后统一 `init_octo_tree()`；增量阶段对每个点直接调 `UpdateOctoTree(pv)`**（上面 2.4 节那套"是平面就攒点定期刷新、不是平面就递归下探/细分"的逻辑），是逐点即时处理，不做"先分组再统一处理"的批量优化——因为增量阶段每帧点数经过降采样后已经不多，没必要再攒批。

### 3.3 地图滑窗：`mapSliding()` / `clearMemOutOfMap()`（`:1001-1046`）—— 硬删除，不是惰性删除

```cpp
void mapSliding() {
    if ((position_last_ - last_slide_position).norm() < sliding_thresh) return;  // 走得不够远，不用滑
    last_slide_position = position_last_;
    // 以当前位置为中心，算出 [x_min,x_max]×[y_min,y_max]×[z_min,z_max] 这个 half_map_size 范围的体素窗口
    clearMemOutOfMap(...);
}

void clearMemOutOfMap(...) {
    for (体素哈希表里每一项)
        if (体素坐标超出窗口范围) { delete it->second; voxel_map_.erase(it); }  // 直接物理删除
}
```

**这里没有"惰性删除+打标记+定期重新平衡"这一套**（对比 FAST_LIO 的 ikd-Tree，见下一节）——只要一个根体素（连同它下面整棵八叉树，靠 `~VoxelOctoTree()` 析构函数递归 `delete leaves_[i]`）落在窗口外，**立刻同步物理释放**，`voxel_map_.erase()` 直接把这个键从哈希表里摘掉。触发时机是"走了超过 `sliding_thresh` 距离才检查一次"（不是每帧都检查，省一点遍历整个哈希表的开销），默认 `map_sliding_en=false`（大多数配置不开这个滑窗，让地图无限增长）。

---

## 4. 和 FAST_LIO 的 ikd-Tree 地图，结构上的核心差异

（详见 [FAST_LIO-原理与代码笔记.md](../FAST_LIO/FAST_LIO-原理与代码笔记.md) 里 ikd-Tree 惰性删除那一节）

| | FAST_LIO：ikd-Tree | FAST-LIVO2-RTK：VoxelMap |
|---|---|---|
| 顶层结构 | 一整棵扁平 kd 树（按点分割超平面） | 哈希表（体素坐标→八叉树），每个体素独立一棵小八叉树 |
| 地图单元存的是什么 | 单个点 | 拟合好的**平面参数**（法向、中心、协方差），点本身用完可以不留 |
| 删除策略 | **惰性删除**：先打标记，`Criterion_Check` 命中阈值（删除比例>50% 或左右子树失衡>70/30）才真正重建子树，大子树还丢给后台线程异步重建 | **硬删除**：滑窗范围外的体素直接同步 `delete`+`erase`，没有标记、没有重建判据、没有后台线程 |
| 平面/结构信息 | 没有——只存点，每次查询时临时用 5 近邻现算平面 | **常驻**——平面参数算一次、缓存在 `VoxelPlane` 里，之后查询直接读，不用重新拟合（除非又满 5 个新点触发刷新） |
| 局部自适应 | 无（kd 树按点数二分，不管局部是不是平坦） | **有**：平坦区域大体素就够、少细分；复杂区域自动细分到 `max_layer_` |

**一句话对比**：ikd-Tree 是"存点、临时现算平面"的通用近邻结构，删除追求"树平衡"；VoxelMap 是"存拟合好的平面、按需细分/冻结"的专用平面地图结构，删除追求"简单直接、跟着车走"。两者是完全不同的设计哲学，不是同一套东西的两种实现。

---

## 5. 关键配置参数速查（`config/HH-LVGO.yaml` 的 `lio`/`local_map` 段）

| 参数 | 对应代码字段 | 含义 |
|---|---|---|
| `lio/voxel_size` | `max_voxel_size_` | 根体素边长（米），比如 0.5 |
| `lio/max_layer` | `max_layer_` | 八叉树最多细分层数，比如 2 |
| `lio/layer_init_num` | `layer_init_num_`（每层一个值） | 每一层要攒够多少点才第一次尝试拟合平面 |
| `lio/max_points_num` | `max_points_num_` | 平面缓存点数上限，超过就冻结不再更新 |
| `lio/min_eigen_value` | `planner_threshold_` | 平面判据阈值（PCA 最小特征值） |
| （硬编码 5，不可配） | `update_size_threshold_` | 已是平面时，每几个新点重新拟合一次 |
| `lio/beam_err` / `dept_err` | `calcBodyCov()` 输入 | 点测量噪声（角度/测距）建模参数 |
| `lio/sigma_num` | `sigma_num_` | 数据关联的 σ 门限倍数 |
| `local_map/map_sliding_en` | — | 是否开启地图滑窗（大多数配置默认关闭） |
| `local_map/half_map_size` | `half_map_size` | 滑窗保留范围（以体素为单位的半边长） |
| `local_map/sliding_thresh` | `sliding_thresh` | 走多远才检查一次是否需要滑窗 |

---

## 6. 代码坑与细节 Checklist

- [ ] **`VoxelOctoTree::Insert()` 是声明了但从未被调用的死代码**（全仓库搜 `->Insert(`/`.Insert(`，只有它自己的递归自调用，没有任何外部调用点）——真正的"往树里加点"走的是 `UpdateOctoTree()`（`UpdateVoxelMap()` 调用），不要把这两个搞混。
- [ ] `update_size_threshold_` 是**硬编码 5**，不是配置项——想调这个"多久刷新一次平面"的频率，得改代码而不是改 yaml。
- [ ] 平面一旦点数超过 `max_points_num_` 就**永久冻结**（`update_enable_=false` 之后无法再恢复），即使后续观测质量更好也不会再更新这个平面——如果发现某片区域的平面参数"很早就不对但一直不更新"，先看是不是撞到了这个冻结机制。
- [ ] 地图滑窗是**硬删除**（立即物理释放），不是 ikd-Tree 那种惰性删除——`map_sliding_en=true` 时被滑出窗口的地图数据**直接丢失**，不会像 FAST_LIO 里那样"删除的点可以选择性保留"（虽然 FAST_LIO 默认也没保留，但至少有 `acquire_removed_points()` 这个接口）；这里没有对应接口。
- [ ] `find_correspond()`（VIO 用）和 `build_single_residual()`（LIO 用）是**两套不同严谨程度的查询**——前者只做树的定位、不做统计检验，后者要过半径粗筛+σ门限+概率择优三道关；不要拿 VIO 侧"随便查一个大概位置"的结果去替代 LIO 侧的严格数据关联逻辑。
- [ ] `BuildVoxelMap()`（首帧建图）和 `UpdateVoxelMap()`（后续增量更新）虽然长得很像，但**内部处理策略不同**（前者攒批最后统一 `init_octo_tree()`，后者逐点 `UpdateOctoTree()`），阅读/修改代码时不要以为两个函数是同一套逻辑复制粘贴。

---

## 7. 重点知识速记

1. **地图 = 体素哈希表 + 每格一棵多层八叉树**，八叉树的每个节点非"平面叶子"即"继续细分"，判据是 PCA 最小特征值是否小于阈值。
2. **平面参数算好后常驻缓存**（`VoxelPlane`：法向、中心、6×6 协方差、radius_），不像 ikd-Tree 那样每次查询临时现算。
3. **平面刷新有节流**（每 5 个新点才重新拟合一次）、**有上限冻结**（点数超过 `max_points_num_` 后永久停止更新），是两个控制计算量/内存的关键机制。
4. **局部自适应细分**：平坦区域体素早早停止细分，复杂区域自动往下细分到 `max_layer_`，不是全图统一分辨率。
5. **删除策略是硬删除**（滑窗直接物理释放整棵子树），和 FAST_LIO ikd-Tree 的"惰性标记+按阈值触发重建+大子树异步重建"是完全不同的两套设计。
6. **`Insert()` 是未使用的死代码**，真正的增量插入路径是 `UpdateVoxelMap()→UpdateOctoTree()`。
7. **VIO 和 LIO 查询同一棵地图但走不同函数**：`find_correspond()`（简单定位）服务视觉法向精化，`build_single_residual()`（半径筛选+统计检验+概率择优）服务 LIO 残差构建。
