#include "optimization.h"
#include <filesystem>

namespace {
// 建目录（含父级）。所有输出路径写文件前都过一遍：
// PCL 写不存在的目录会抛 pcl::IOException("Error during open!")，未捕获直接 terminate 整个节点；
// ofstream 则是静默失败（TUM/vel 文本什么都不写，更难查）。
bool ensureOutDir(const std::string &dir, const char *tag)
{
  if (dir.empty()) return false;
  try
  {
    std::filesystem::create_directories(dir);
    return true;
  }
  catch (const std::exception &e)
  {
    ROS_WARN("[%s] create dir failed: %s (%s)", tag, dir.c_str(), e.what());
    return false;
  }
}
}  // namespace

optimization::optimization(ros::NodeHandle &nh)
{
    nh.param<std::string>("laserMapping/outputfilepath", outputfilepath, "");
    debug_optdata_path_ = outputfilepath + "/debug/";
    opt_tum_output_path_ = outputfilepath + "/TUM/opt_trajectory_after.txt";
    gps_tum_output_path_ = outputfilepath + "/TUM/opt_trajectory_before.txt";
    opt_vel_output_path_ = outputfilepath + "/vel/opt_vel.txt";
    gps_vel_output_path_ = outputfilepath + "/vel/gps_vel.txt";
    global_map_pcd_path_ = outputfilepath + "/global_pcd/";
    keyframe_scan_pcd_path_ = outputfilepath + "/scan_pcd/";

    // 输出目录必须先建好（原来只有 debug/ 和 vel/ 是手工存在的，
    // global_pcd/ 缺失导致 saveOptimizedGlobalMap 抛 pcl::IOException 把节点搞崩）
    ensureOutDir(debug_optdata_path_,           "opt-out");
    ensureOutDir(outputfilepath + "/TUM",       "opt-out");
    ensureOutDir(outputfilepath + "/vel",       "opt-out");
    ensureOutDir(global_map_pcd_path_,          "opt-out");
    ensureOutDir(keyframe_scan_pcd_path_,       "opt-out");

    nh.param<string>("gps/gps_topic", gps_topic, "/ublox_driver/receiver_lla");
    nh.param<double>("gps/gps_time_offset", gps_offset, 0.0);
    nh.param<bool>("gps/gps_en", gps_en, false);
    nh.param<vector<double>>("gps/extrinsic_T", gps_extrinT, vector<double>());

    // 批优化触发方式：默认等关键帧停止增长(bag 播完)后自动跑；
    // 也可以随时 `rostopic pub /run_offline_opt std_msgs/Empty -1` 手动触发
    nh.param<bool>("opt/auto_trigger_en", auto_trigger_en_, true);
    nh.param<double>("opt/finish_timeout", finish_timeout_, 5.0);
    nh.param<int>("opt/min_keyframes", min_keyframes_, 10);
    // initialAlign 里刚体对齐用多长的轨迹（秒）：<=0 = 用全部关键帧
    nh.param<double>("opt/align_window", align_window_, -1.0);
    // RTK(GPS)因子按距离抽稀的间隔，米；<=0 表示每个关键帧都加
    nh.param<double>("opt/rtk_factor_interval", rtk_factor_interval_, 10.0);
    subRunOpt = nh.subscribe<std_msgs::Empty>("/run_offline_opt", 1, &optimization::runOptTrigger_cbk, this);
    // pubGpsOdom 原来只声明没 advertise，两个 handler 里的 publish() 全是空转
    // （Release 带 -DNDEBUG，roscpp 里那句 ROS_ASSERT 被编掉，只剩静默 return）。
    // 用独立话题名，别和 LIVMapper 已占用的 /gps/odometry 撞。
    pubGpsOdom = nh.advertise<nav_msgs::Odometry>("/opt/rtk_odometry", 2000);

    nh.param<bool>("opt/debug_mode", debug_mode, false);
    nh.param<double>("opt/rtk_cov", rtk_cov, 0.02);
    nh.param<double>("opt/livo2_RPY_cov", livo2_RPY_cov, 1e-4);
    nh.param<double>("opt/livo2_XYZ_cov", livo2_XYZ_cov, 1e-4);

    // RTK 观测来源二选一，和 LIVMapper 用同一组参数：默认 GNSS PVT；
    // odom_rtk_en=true 时离线批优化的 RTK 因子也改由 /localization/odom 提供，
    // 否则 gpsQueue 一直是空的（initialAlign 只会打印 "empty buffers" 直接返回）。
    nh.param<bool>("gps/odom_rtk_en", odom_rtk_en_, false);
    nh.param<std::string>("gps/odom_rtk_topic", odom_rtk_topic_, "/localization/odom");
    // subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 2000, &optimization::gpsHandler, this);
    if (odom_rtk_en_)
    {
        subOdomRtk = nh.subscribe<nav_msgs::Odometry>(odom_rtk_topic_, 2000, &optimization::odomRtkHandler, this);
        ROS_INFO("[opt] use odometry topic as RTK source: %s", odom_rtk_topic_.c_str());
    }
    else
    {
        subGPS_pvt = nh.subscribe<gnss_comm::GnssPVTSolnMsg>(gps_topic, 2000, &optimization::gpsHandler, this);
    }

    Eigen::Vector3d gps_lever_arm_; 
    gps_lever_arm_(0) = gps_extrinT[0]; 
    gps_lever_arm_(1) = gps_extrinT[1]; 
    gps_lever_arm_(2) = gps_extrinT[2]; 
    T_imu_rtk = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(gps_lever_arm_));

    subOdom_.subscribe(nh, "/odometry/fast_livo2", 2000);
    subCloud_.subscribe(nh, "/synced_cloud", 2000);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), subOdom_, subCloud_);
    sync_->registerCallback(boost::bind(&optimization::syncedCallback, this, _1, _2));

    laserCloudSurfLastDS.reset(new PointCloudXYZRGB());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    for (int i = 0; i < 6; ++i) {
        transformTobeMapped[i] = 0;
    }

    if(debug_mode)
    {
        loadData(debug_optdata_path_);
    }

    ROS_INFO("Optimization mode: [OFFLINE]. Accumulating factors for batch optimization.");
    optimization_thread_ = std::thread(&optimization::offlineOptimizationTask, this);
}

optimization::~optimization()
{
    // offlineOptimizationTask 线程若仍 joinable（比如还阻塞在 std::cin.get()，或已提前返回但
    // 未被 join），在 std::thread 析构时会触发 std::terminate()——进程退出(Ctrl-C)时就会又崩一次。
    // 这里 detach 掉，保证退出干净。join 不合适：交互模式下线程可能一直阻塞在 std::cin.get()，
    // join 会把退出流程挂死。
    if (optimization_thread_.joinable())
        optimization_thread_.detach();
}

//Optimize threads
void optimization::runOptTrigger_cbk(const std_msgs::Empty::ConstPtr& /*msg*/)
{
    ROS_INFO("[Offline Optimization] 收到 /run_offline_opt 手动触发");
    trigger_now_ = true;
}

// 等批优化的触发条件。
// 原来这里是 std::cin.get()：roslaunch 下本节点没有交互式 stdin（终端被 rosbag play 占用），
// 它会立刻以 EOF 返回，于是批优化在一条数据都没收到时就跑一遍空 buffer；后来加的空 buffer
// 保护又让线程直接 return —— 两个一叠加，表现就是"优化线程根本没起来"。
// 现在改成轮询等待：关键帧数攒够 min_keyframes 且连续 finish_timeout 秒不再增长（说明 bag
// 播完/建图停了）就开跑，也接受 /run_offline_opt 手动触发。
// 计时用墙上时间：若 use_sim_time=true，bag 播完后 /clock 就停了，ros::Time::now() 会冻住。
bool optimization::waitForBatchTrigger()
{
    size_t last_size = 0;
    ros::WallTime last_change = ros::WallTime::now();
    ros::WallTime last_log = ros::WallTime::now();

    while (ros::ok())
    {
        size_t cur_size = 0;
        {
            std::lock_guard<std::mutex> lock(mutex);
            cur_size = cloudKeyPoses6D->size();
        }
        const ros::WallTime now = ros::WallTime::now();

        if (cur_size != last_size) { last_size = cur_size; last_change = now; }

        const double idle = (now - last_change).toSec();

        if (trigger_now_)
        {
            if (cur_size == 0) {
                ROS_WARN("[Offline Optimization] 手动触发但还没有任何关键帧，忽略");
                trigger_now_ = false;
            } else {
                ROS_INFO("[Offline Optimization] 手动触发，keyframes=%zu，开始批优化", cur_size);
                return true;
            }
        }

        if (auto_trigger_en_ && cur_size >= (size_t)min_keyframes_ && idle >= finish_timeout_)
        {
            ROS_INFO("[Offline Optimization] keyframes=%zu 已连续 %.1fs 没有新增，判定建图结束，开始批优化",
                     cur_size, idle);
            return true;
        }

        if ((now - last_log).toSec() >= 10.0)
        {
            last_log = now;
            ROS_INFO("[Offline Optimization] 等待触发中: keyframes=%zu, rtk=%zu, 空闲 %.1fs "
                     "(auto_trigger=%d, min_keyframes=%d, finish_timeout=%.1fs; 或 rostopic pub /run_offline_opt std_msgs/Empty -1)",
                     cur_size, gpsQueue.size(), idle, (int)auto_trigger_en_, min_keyframes_, finish_timeout_);
        }

        ros::WallDuration(0.5).sleep();
    }
    ROS_WARN("[Offline Optimization] 节点关闭，未执行批优化");
    return false;
}

void optimization::offlineOptimizationTask() {
    if (!waitForBatchTrigger()) return;

    // 从这里开始不再接收新关键帧：批优化会遍历/改写 cloudKeyPoses6D、surfCloudKeyFrames 和
    // initialEstimate，而 syncedCallback 在另一个线程往里 push，并发修改会直接崩。
    {
        std::lock_guard<std::mutex> lock(mutex);
        optimizing_ = true;
    }
    std::cout << "[Offline Optimization] Starting batch optimization..." << std::endl;
    initialAlign();
    std::cout << "[Offline Optimization] Initial alignment done." << std::endl;
    writeOptimizedTumTrajectory();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Initial global map and tum saved." << std::endl;
    
    gtsam::LevenbergMarquardtParams params;
    params.orderingType = gtsam::Ordering::METIS;
    params.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
    params.setMaxIterations(500);
    params.setRelativeErrorTol(1e-9);
    params.setAbsoluteErrorTol(1e-9);
    
    buildBatchGraph();
    gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate, params);
    gtsam::Values result = optimizer.optimize();
    initialEstimate = result;
    std::cout << "[Offline Optimization] First optimization pass done." <<  std::endl;

    is_optimized = true;
    int numPoses = initialEstimate.size();

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        cloudKeyPoses6D->points[i].x     = rtk_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y     = rtk_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z     = rtk_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = rtk_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = rtk_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = rtk_optimizedPose.rotation().yaw();
    }

    writeOptimizedTumTrajectory();
    std::cout << "[Offline Optimization] Optimized TUM trajectory saved "  << std::endl;
    writeRtkTumTrajectory();
    std::cout << "[Offline Optimization] RTK TUM trajectory saved " << std::endl;

    for (int i = 0; i < numPoses; ++i) {
        gtsam::Pose3 rtk_optimizedPose = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 imu_optimizedPose = rtk_optimizedPose.compose((T_imu_rtk).inverse());
        cloudKeyPoses6D->points[i].x     = imu_optimizedPose.translation().x();
        cloudKeyPoses6D->points[i].y     = imu_optimizedPose.translation().y();
        cloudKeyPoses6D->points[i].z     = imu_optimizedPose.translation().z();
        cloudKeyPoses6D->points[i].roll  = imu_optimizedPose.rotation().roll();
        cloudKeyPoses6D->points[i].pitch = imu_optimizedPose.rotation().pitch();
        cloudKeyPoses6D->points[i].yaw   = imu_optimizedPose.rotation().yaw();
    }

    std::cout << "[Offline Optimization] Saving maps and trajectories..." << std::endl;
    //savekeyframescan();
    saveOptimizedGlobalMap();
    std::cout << "[Offline Optimization] Global map saved " << std::endl;

    std::cout << "[Offline Optimization] Finished." << std::endl;
}

void optimization::saveKeyFramesAndFactor()
{
    std::lock_guard<std::mutex> lock(mutex);
    // 批优化已经开始：不再往 buffer 里加东西，否则和优化线程并发修改同一批容器
    if (optimizing_) return;
    
    // Store pose
    PointTypePose thisPose6D;
    gtsam::Pose3 currentPose = trans2gtsamPose(transformTobeMapped);
    initialEstimate.insert(cloudKeyPoses6D->size(), currentPose);

    thisPose6D.x = currentPose.translation().x();
    thisPose6D.y = currentPose.translation().y();
    thisPose6D.z = currentPose.translation().z();
    thisPose6D.intensity = cloudKeyPoses6D->size();
    thisPose6D.roll  = currentPose.rotation().roll();
    thisPose6D.pitch = currentPose.rotation().pitch();
    thisPose6D.yaw   = currentPose.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);
    
    // Storage point cloud
    PointCloudXYZRGB::Ptr thisSurfKeyFrame(new PointCloudXYZRGB());
    pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    
}

//DTW
double optimization::calculateDtwTimeOffset(
    const std::vector<std::vector<double>>& gpsdata,
    const std::vector<std::vector<double>>& slamdata)
{
    const double MIN_EFFECTIVE_VELOCITY = 0.5; // m/s
    const int IDX_TIME = 0;
    const int IDX_X = 1;
    const int IDX_Y = 2;
    const int IDX_Z = 3;

    auto process_data = [=](const std::vector<std::vector<double>>& data_in) 
        -> std::pair<std::vector<double>, std::vector<double>>
    {
        double duration = 50.0;
        const double t_start = data_in[0][IDX_TIME];
        const double t_end   = t_start + duration;

        std::vector<double> times;
        std::vector<double> norms;
        
        bool is_moving = false;
        Eigen::Vector3d origin_pos(0, 0, 0); 

        for (size_t i = 1; i < data_in.size(); ++i) {
            const auto& row_i = data_in[i];
            const auto& row_prev = data_in[i-1];

            const double current_time = row_i[IDX_TIME];
            if (current_time > t_end) {
                break; 
            }

            if (!is_moving) {
                double dt = row_i[IDX_TIME] - row_prev[IDX_TIME];
                if (dt < 1e-6) continue; 
                double dx = row_i[IDX_X] - row_prev[IDX_X];
                double dy = row_i[IDX_Y] - row_prev[IDX_Y];
                double dz = row_i[IDX_Z] - row_prev[IDX_Z];               
                double vel_norm = std::sqrt(dx*dx + dy*dy + dz*dz) / dt;

                double pos_norm = std::sqrt(row_i[IDX_X]*row_i[IDX_X] + row_i[IDX_Y]*row_i[IDX_Y] + row_i[IDX_Z]*row_i[IDX_Z]);
                if (vel_norm > MIN_EFFECTIVE_VELOCITY && pos_norm > MIN_EFFECTIVE_VELOCITY) {
                    is_moving = true;
                } else {
                    continue; 
                }
            } else {
                Eigen::Vector3d current_pos(row_i[IDX_X], row_i[IDX_Y], row_i[IDX_Z]);
                double pos_norm = current_pos.norm();

                times.push_back(row_i[IDX_TIME]);
                norms.push_back(pos_norm);
            }
        }
        
        return {times, norms};
    };

    auto [gps_pos_times, gps_pos_norms] = process_data(gpsdata);
    auto [slam_pos_times, slam_pos_norms] = process_data(slamdata);

    sample1 = &(slam_pos_norms[0]);
    sample2 = &(gps_pos_norms[0]);

    std::vector<std::pair<int, int>> dtw_path;
    DTW(slam_pos_norms, gps_pos_norms, dtw_path);

    double total_time_diff = 0.0;
    int valid_pairs = 0;

    for (const auto& pair : dtw_path) {
        if (pair.first < slam_pos_times.size() && pair.second < gps_pos_times.size()) {
            total_time_diff += (slam_pos_times[pair.first] - gps_pos_times[pair.second]);
            valid_pairs++;
        }
    }

    if (valid_pairs == 0) {
        return 0.0;
    }

    double avg_time_diff = total_time_diff / valid_pairs;

    std::ofstream slam_file(opt_vel_output_path_);
    if (slam_file.is_open()) {
        slam_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < slam_pos_norms.size(); ++i) {
            slam_file << slam_pos_times[i] << " " << slam_pos_norms[i] << std::endl;
        }
        slam_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt");
    }

    std::ofstream gps_file(gps_vel_output_path_);
    if (gps_file.is_open()) {
        gps_file << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < gps_pos_norms.size(); ++i) {
            gps_file << gps_pos_times[i] << " " << gps_pos_norms[i] << std::endl;
        }
        gps_file.close();
    } else {
        ROS_WARN("[initialAlign] could not open gps_vel.txt ");
    }
    return avg_time_diff;
}

//Spacetime synchronization
void optimization::initialAlign()
{
    //using DTW
    // std::vector<std::vector<double>> slamdata;
    // slamdata.reserve(cloudKeyPoses6D->size());
    // for (const auto& p : cloudKeyPoses6D->points) {
    //     slamdata.push_back({p.time, p.x, p.y, p.z});
    // }
    
    // std::vector<std::vector<double>> gpsdata;
    // gpsdata.reserve(gpsQueue.size());
    // for (const auto& g : gpsQueue) {
    //     gpsdata.push_back({
    //         g.header.stamp.toSec(),
    //         g.pose.pose.position.x,
    //         g.pose.pose.position.y,
    //         g.pose.pose.position.z
    //     });
    // }

    // double avg_time_diff = calculateDtwTimeOffset(gpsdata, slamdata);
    
    // ros::Duration time_offset(avg_time_diff);

    // for(auto& g : gpsQueue) {
    //     g.header.stamp += time_offset;
    // }
 
    // ROS_INFO("[initialAlign] 1. DTW calculated time offset: %.4f seconds.", avg_time_diff);

    //using spline
    std::cout << " [initialAlign] initial align (using Spline)... " << std::endl;

    if (cloudKeyPoses6D->empty() || gpsQueue.empty()) {
        ROS_WARN("[initialAlign] empty buffers.");
        return;
    }
    
    struct Sample { double t; Eigen::Vector3d p; };
    std::vector<Sample> samp; 
    samp.reserve(gpsQueue.size());
    
    for (const auto& g : gpsQueue) {
        const double t = g.header.stamp.toSec();
        const auto& p  = g.pose.pose.position; 
        samp.push_back({ t, Eigen::Vector3d(p.x, p.y, p.z) });
    }
    std::sort(samp.begin(), samp.end(),
              [](const Sample& a, const Sample& b){ return a.t < b.t; });

    std::vector<double> T; T.reserve(samp.size());
    std::vector<double> X, Y, Z;
    X.reserve(samp.size()); Y.reserve(samp.size()); Z.reserve(samp.size());
    
    for (size_t i = 0; i < samp.size(); ++i) {
        if (i > 0 && std::fabs(samp[i].t - samp[i-1].t) < 1e-12) continue; 
        T.emplace_back(samp[i].t);
        X.emplace_back(samp[i].p.x());
        Y.emplace_back(samp[i].p.y());
        Z.emplace_back(samp[i].p.z());
    }
    
    if (T.size() < 2) {
        ROS_WARN("[initialAlign] not enough RTK samples for spline.");
        return;
    }

    auto buildSpline = [](const std::vector<double>& t,
                          const std::vector<double>& y,
                          std::vector<double>& M) -> bool {
        const size_t n = t.size();
        M.assign(n, 0.0);
        if (n < 2) return false;
        if (n == 2) return true; // 两点线性
        
        std::vector<double> h(n-1);
        for (size_t i = 0; i < n - 1; ++i) {
            h[i] = t[i+1] - t[i];
            if (h[i] <= 0) return false; // 要严格递增
        }
        
        const size_t m = n - 2;
        std::vector<double> a(m), b(m), c(m), d(m);
        for (size_t i = 0; i < m; ++i) {
            a[i] = h[i] / 6.0;
            b[i] = (h[i] + h[i+1]) / 3.0;
            c[i] = h[i+1] / 6.0;
            d[i] = (y[i+2] - y[i+1]) / h[i+1] - (y[i+1] - y[i]) / h[i];
        }
        
        for (size_t i = 1; i < m; ++i) {
            double w = a[i] / b[i-1];
            b[i] -= w * c[i-1];
            d[i] -= w * d[i-1];
        }
        
        std::vector<double> Min(m, 0.0);
        if (m > 0) {
            Min[m-1] = d[m-1] / b[m-1];
            for (int i = int(m) - 2; i >= 0; --i)
                Min[i] = (d[i] - c[i] * Min[i+1]) / b[i];
        }
        for (size_t i = 0; i < m; ++i) M[i+1] = Min[i]; 
        return true;
    };

    auto evalSpline = [](const std::vector<double>& t,
                         const std::vector<double>& y,
                         const std::vector<double>& M,
                         double tq) -> double {
        const size_t n = t.size();
        if (n == 0) return 0.0;
        if (n == 1) return y[0];
        
        auto derivLeft = [&]() {
            double h = t[1] - t[0];
            return (y[1] - y[0]) / h - (h / 6.0) * (2.0 * M[0] + M[1]);
        };
        auto derivRight = [&]() {
            size_t k = n - 2; double h = t[k+1] - t[k];
            return (y[k+1] - y[k]) / h + (h / 6.0) * (2.0 * M[k+1] + M[k]);
        };
        
        if (tq <= t.front()) return y.front() + derivLeft()  * (tq - t.front());
        if (tq >= t.back())  return y.back()  + derivRight() * (tq - t.back());
        
        size_t k = std::upper_bound(t.begin(), t.end(), tq) - t.begin() - 1;
        double h = t[k+1] - t[k];
        double A = (t[k+1] - tq) / h;
        double B = (tq - t[k]) / h;
        
        return M[k] * (A * A * A) * h / 6.0
             + M[k+1] * (B * B * B) * h / 6.0
             + (y[k]   - M[k] * h * h / 6.0) * A
             + (y[k+1] - M[k+1] * h * h / 6.0) * B;
    };

    std::vector<double> Mx, My, Mz;
    if (!buildSpline(T, X, Mx) || !buildSpline(T, Y, My) || !buildSpline(T, Z, Mz)) {
        ROS_WARN("[initialAlign] spline build failed.");
        return;
    }

    auto find_nearest_gps = [&](double t_query) -> const nav_msgs::Odometry* {
        if (gpsQueue.empty()) return nullptr;

        auto it_lower = std::lower_bound(gpsQueue.begin(), gpsQueue.end(), t_query,
            [](const nav_msgs::Odometry& msg, double t) {
                return msg.header.stamp.toSec() < t;
            });

        if (it_lower == gpsQueue.begin()) {
            return &(*it_lower);
        }

        if (it_lower == gpsQueue.end()) {
            return &(*std::prev(it_lower));
        }

        const nav_msgs::Odometry& after = *it_lower;
        const nav_msgs::Odometry& before = *std::prev(it_lower);

        double dt_after = std::abs(after.header.stamp.toSec() - t_query);
        double dt_before = std::abs(t_query - before.header.stamp.toSec());

        if (dt_after < dt_before) {
            return &after;
        } else {
            return &before;
        }
    };
    
    //Aligning GPS queues and odometer spline
    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const double tk = cloudKeyPoses6D->points[i].time;

        nav_msgs::Odometry gps_odom_B;
        gps_odom_B.header.stamp = ros::Time().fromSec(tk);
        gps_odom_B.header.frame_id = "map";
        gps_odom_B.pose.pose.position.x = evalSpline(T, X, Mx, tk);
        gps_odom_B.pose.pose.position.y = evalSpline(T, Y, My, tk);
        gps_odom_B.pose.pose.position.z = evalSpline(T, Z, Mz, tk);
        gps_odom_B.pose.pose.orientation.w = 1.0; 

        const nav_msgs::Odometry* nearest_gps = find_nearest_gps(tk);
        gps_odom_B.pose.covariance[0]  = nearest_gps->pose.covariance[0]; 
        gps_odom_B.pose.covariance[7]  = nearest_gps->pose.covariance[7];  
        gps_odom_B.pose.covariance[14] = nearest_gps->pose.covariance[14];
        gpsQueue_B.push_back(gps_odom_B);
    }

    //Coordinate system transformation
    std::vector<Eigen::Vector3d> A_slam_gps; 
    std::vector<Eigen::Vector3d> B_enu_gps;  

    // 对齐用多长的轨迹：opt/align_window <= 0 表示用全部关键帧（默认）。
    // 原来固定只用前 50 s：一段近直线、近水平的轨迹对 6 自由度刚体 SVD 来说条件数很差，
    // 俯仰/横滚几乎不可观测，SVD 会把 z 噪声拟成一个虚假倾角，误差再随行驶距离线性放大。
    // 用全程数据（含转弯、坡度）几何构型丰富得多，拟合稳定；代价是 LIO 自身的漂移会被平摊
    // 到整条轨迹（起点不再严格贴合）——对"给批优化提供初值"这个用途来说反而更合适。
    const double t_start = cloudKeyPoses6D->points[0].time;
    const bool use_all = (align_window_ <= 0.0);
    const double t_end_svd = use_all ? 0.0 : (t_start + align_window_);

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) 
    {
        const auto& slam_pose_imu = cloudKeyPoses6D->points[i];
        const double tk = slam_pose_imu.time;
        
        if (!use_all && tk > t_end_svd) break; 
        if (tk < T.front() || tk > T.back()) continue; 

        Eigen::Vector3d p_enu_gps;
        p_enu_gps.x() = evalSpline(T, X, Mx, tk);
        p_enu_gps.y() = evalSpline(T, Y, My, tk);
        p_enu_gps.z() = evalSpline(T, Z, Mz, tk);
        B_enu_gps.push_back(p_enu_gps);

        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i); 
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        
        A_slam_gps.push_back(T_slam_gps.translation());
    }

    if (A_slam_gps.size() < 3) {
        ROS_WARN("[initialAlign] too few spline pairs to align.");
        return;
    }

    gtsam::Pose3 T_enu_slam = computeSVD(B_enu_gps, A_slam_gps);

    // 对齐质量诊断：
    //  - 点对数 / 轨迹里程（里程太短 → SVD 条件数差）
    //  - T_enu_slam 的 rpy：LIVO 世界系就是初始 IMU 系（ImuProcess 里 rot_end 初始化成单位阵，
    //    重力只存成向量、没把世界系转到重力对齐），所以 roll/pitch 理论上就等于开机瞬间 IMU
    //    相对水平面的倾角，只有几度。明显更大就说明 SVD 拟出了虚假倾斜。
    //  - 对齐残差 RMS/最大值：尺度或倾斜误差的特征是残差随行驶距离线性增长。
    {
        const Eigen::Vector3d rpy = T_enu_slam.rotation().rpy();
        const gtsam::Point3   t   = T_enu_slam.translation();
        double baseline = 0.0;
        for (size_t k = 1; k < A_slam_gps.size(); ++k) baseline += (A_slam_gps[k] - A_slam_gps[k-1]).norm();

        double sum_sq = 0.0, max_err = 0.0;
        for (size_t k = 0; k < A_slam_gps.size(); ++k) {
            const gtsam::Point3 pa(A_slam_gps[k]);
            const double e = (B_enu_gps[k] - T_enu_slam.transformFrom(pa)).norm();
            sum_sq += e * e;
            if (e > max_err) max_err = e;
        }
        const double rms = std::sqrt(sum_sq / A_slam_gps.size());
        const double R2D = 57.29577951308232;

        ROS_INFO("[initialAlign] SVD 用了 %zu 个点对 (%s), 轨迹里程 %.1f m",
                 A_slam_gps.size(), use_all ? "全部关键帧" : "前 align_window 秒", baseline);
        ROS_INFO("[initialAlign] T_enu_slam: rpy=[%.3f, %.3f, %.3f] deg, t=[%.2f, %.2f, %.2f] m",
                 rpy(0) * R2D, rpy(1) * R2D, rpy(2) * R2D, t.x(), t.y(), t.z());
        ROS_INFO("[initialAlign] 对齐残差: RMS=%.3f m, max=%.3f m", rms, max_err);
        if (std::fabs(rpy(0)) * R2D > 10.0 || std::fabs(rpy(1)) * R2D > 10.0)
            ROS_WARN("[initialAlign] T_enu_slam 的 roll/pitch 偏大，可能是 SVD 在退化构型上拟出了虚假倾斜");
    }

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i) {
        gtsam::Pose3 T_slam_imu = initialEstimate.at<gtsam::Pose3>(i);
        gtsam::Pose3 T_slam_gps = T_slam_imu.compose(T_imu_rtk);
        gtsam::Pose3 T_enu_gps = T_enu_slam.compose(T_slam_gps);

        initialEstimate.update(i, T_enu_gps);
        
        const auto& p = T_enu_gps.translation();
        const auto& r = T_enu_gps.rotation().rpy();
        cloudKeyPoses6D->points[i].x     = p.x();
        cloudKeyPoses6D->points[i].y     = p.y();
        cloudKeyPoses6D->points[i].z     = p.z();
        cloudKeyPoses6D->points[i].roll  = r(0);
        cloudKeyPoses6D->points[i].pitch = r(1);
        cloudKeyPoses6D->points[i].yaw   = r(2);
    }
    
    ROS_INFO("[initialAlign] Spline-based alignment complete.");
}

//build gtsam graph
void optimization::buildBatchGraph()
{
    gtSAMgraph.resize(0); 
             
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = 
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << livo2_RPY_cov, livo2_RPY_cov, livo2_RPY_cov, livo2_XYZ_cov, livo2_XYZ_cov, livo2_XYZ_cov*4.0).finished());
    // Treat `rtk_cov` as RTK standard deviation (sigma) in meters.
    // Square it to get variances for GTSAM noise model.
    const double rtk_var = rtk_cov * rtk_cov;
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
        gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << rtk_var, rtk_var, rtk_var * 4.0).finished());
    
    ROS_INFO("rtk_var, livo2_RPY_cov, livo2_XYZ_cov: %f, %f, %f", rtk_var, livo2_RPY_cov, livo2_XYZ_cov);

    size_t rtk_idx = 0; 

    ROS_INFO("[buildBatchGraph] initialEstimate.size()=%zu, gpsQueue_B.size()=%zu", initialEstimate.size(), gpsQueue_B.size());

    // RTK 因子按空间距离抽稀：每走过 rtk_factor_interval_ 米才加一个（<=0 = 每个关键帧都加）。
    // 关键帧是按时间产生的，等红灯/堵车时车不动但关键帧照样在攒，同一个位置会堆上几十个
    // GPSFactor —— 等效于把那个点的权重放大几十倍，优化结果会被静止段拽偏。
    // 按距离抽稀后，RTK 约束在空间上均匀分布。
    bool has_last_gps = false;
    gtsam::Point3 last_gps_point(0.0, 0.0, 0.0);
    size_t n_gps_factor = 0;

    for (size_t i = 0; i < initialEstimate.size(); ++i)
    {
        // addpriorfactor and betweenfactor
        if (i == 0)
        {
            gtsam::Pose3 priorPose_T_wa = initialEstimate.at<gtsam::Pose3>(0);
            gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorPose_T_wa, priorNoise));
        }
        else 
        {
            gtsam::Pose3 poseFrom_T_wa = initialEstimate.at<gtsam::Pose3>(i-1);
            gtsam::Pose3 poseTo_T_wa   = initialEstimate.at<gtsam::Pose3>(i);
            gtsam::Pose3 T_rel_ant_from_svd = poseFrom_T_wa.between(poseTo_T_wa);
            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(i-1, i, T_rel_ant_from_svd, odometryNoise));
        }

        // addrtkfactor
        // initialAlign 若提前返回（gpsQueue 空 / 样条建不起来），gpsQueue_B 会是空的或短于位姿数，
        // 这里原来直接 gpsQueue_B[i] 是越界访问。缺 RTK 就只保留里程计因子，别崩。
        if (i >= gpsQueue_B.size())
        {
            ROS_WARN_ONCE("[buildBatchGraph] gpsQueue_B(%zu) 短于位姿数(%zu)，缺失部分不加 RTK 因子",
                          gpsQueue_B.size(), initialEstimate.size());
            continue;
        }
        auto it = gpsQueue_B[i];
        double x = it.pose.pose.position.x;
        double y = it.pose.pose.position.y;
        double z = it.pose.pose.position.z;
        gtsam::Point3 gps_point(x, y, z);

        double cov_x = it.pose.covariance[0]; 
        double cov_y = it.pose.covariance[7]; 
        double cov_z = it.pose.covariance[14]; 

        // 距上一个 RTK 因子不足 rtk_factor_interval_ 米就跳过（首个必加，作为绝对位置锚点）
        if (has_last_gps && rtk_factor_interval_ > 0.0 &&
            (gps_point - last_gps_point).norm() < rtk_factor_interval_)
        {
            continue;
        }
        last_gps_point = gps_point;
        has_last_gps   = true;

        gtSAMgraph.add(gtsam::GPSFactor(i, gps_point, gps_noise));
        ++n_gps_factor;
    }

    ROS_INFO("[buildBatchGraph] 位姿 %zu 个, RTK 因子 %zu 个 (间隔 %.1f m%s)",
             initialEstimate.size(), n_gps_factor, rtk_factor_interval_,
             rtk_factor_interval_ > 0.0 ? "" : " = 每帧都加");
    if (n_gps_factor < 3)
        ROS_WARN("[buildBatchGraph] RTK 因子只有 %zu 个，绝对约束太少，优化基本等于原样输出",
                 n_gps_factor);
} 

void optimization::syncedCallback(const nav_msgs::Odometry::ConstPtr& odomMsg, const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    timeLaserInoStamp = cloudMsg->header.stamp;
    timeLaserInfoCur  = cloudMsg->header.stamp.toSec();

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odomMsg->pose.pose.orientation, orientation);
    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    transformTobeMapped[0] = roll;
    transformTobeMapped[1] = pitch;
    transformTobeMapped[2] = yaw;
    transformTobeMapped[3] = odomMsg->pose.pose.position.x;
    transformTobeMapped[4] = odomMsg->pose.pose.position.y;
    transformTobeMapped[5] = odomMsg->pose.pose.position.z;
    
    laserCloudSurfLastDS->clear();
    pcl::fromROSMsg(*cloudMsg, *laserCloudSurfLastDS);

    saveKeyFramesAndFactor();
}

void optimization::gpsHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr& gpsMsg)
{
    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
    }

    // GPS time to UNIX time
    const double GPS_EPOCH_UNIX_TIME = 315964800.0;
    const double SECONDS_PER_WEEK = 604800.0; 
    const double LEAP_SECONDS = 18.0;
    
    int week = gpsMsg->time.week;
    double tow = gpsMsg->time.tow; 
    double total_gps_seconds = (double)week * SECONDS_PER_WEEK + tow;
    double timestamp_sec = total_gps_seconds + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS;
    
    ros::Time stamp;
    stamp.fromSec(timestamp_sec);
    gpstimestamp = timestamp_sec; 

    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, 
                       trans_local_[0], trans_local_[1], trans_local_[2]);

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = stamp - ros::Duration(gps_offset); 
    gps_odom.header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    
    gps_odom.twist.twist.linear.x = gpsMsg->vel_e;
    gps_odom.twist.twist.linear.y = gpsMsg->vel_n;
    gps_odom.twist.twist.linear.z = -gpsMsg->vel_d;

    gps_odom.pose.covariance[0]  = gpsMsg->h_acc; 
    gps_odom.pose.covariance[7]  = gpsMsg->h_acc; 
    gps_odom.pose.covariance[14] = gpsMsg->v_acc; 

    pubGpsOdom.publish(gps_odom);
    gpsQueue.push_back(gps_odom);
}

// 与 LIVMapper::localization_odom_cbk 对应：把 /localization/odom 当 RTK 喂给离线批优化。
// 位置同样减掉首帧当原点——和 LIVMapper 同进程、同话题、同一条首帧消息，两边原点一致。
void optimization::odomRtkHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    static int cnts = 0;
    if (cnts++ % 10 != 0) {
        return;
    }

    const Eigen::Vector3d p_abs(odomMsg->pose.pose.position.x,
                                odomMsg->pose.pose.position.y,
                                odomMsg->pose.pose.position.z);
    if (!odom_rtk_first_) {
        odom_rtk_first_ = true;
        odom_rtk_origin_ = p_abs;
        ROS_INFO("[opt][RTK(odom)] origin set to [%.3f, %.3f, %.3f]",
                 odom_rtk_origin_[0], odom_rtk_origin_[1], odom_rtk_origin_[2]);
    }
    const Eigen::Vector3d trans_local_ = p_abs - odom_rtk_origin_;

    gpstimestamp = odomMsg->header.stamp.toSec();

    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = odomMsg->header.stamp - ros::Duration(gps_offset);
    gps_odom.header.frame_id = "map";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation.w = 1.0;   // 只用位置，姿态填单位四元数（否则 TUM 里是非法的 0,0,0,0）

    // gps_odom.pose.pose.orientation = odomMsg->pose.pose.orientation;
    // gps_odom.twist = odomMsg->twist;
    // gps_odom.pose.covariance[0]  = 0.03 * 0.03;
    // gps_odom.pose.covariance[7]  = 0.03 * 0.03;
    // gps_odom.pose.covariance[14] = 0.06 * 0.06;

    pubGpsOdom.publish(gps_odom);
    gpsQueue.push_back(gps_odom);
}

void optimization::savekeyframescan()
{
    for (size_t i = 0; i < surfCloudKeyFrames.size(); ++i)
    {
        PointCloudXYZRGB::Ptr keyframe_cloud(new PointCloudXYZRGB());
        *keyframe_cloud = *surfCloudKeyFrames[i];
        Eigen::Affine3f transform = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        pcl::transformPointCloud(*keyframe_cloud, *keyframe_cloud, transform);
        std::string save_path = keyframe_scan_pcd_path_ + std::to_string(i) + ".pcd";
        if (keyframe_cloud->empty()) continue;
        ensureOutDir(keyframe_scan_pcd_path_, "savekeyframescan");
        try
        {
            pcl::io::savePCDFileBinary(save_path, *keyframe_cloud);
        }
        catch (const std::exception &e)
        {
            ROS_WARN("[savekeyframescan] write failed: %s (%s)", save_path.c_str(), e.what());
            return;
        }
    }
}

void optimization::saveOptimizedGlobalMap()
{
    PointCloudXYZRGB::Ptr globalMapCloud(new PointCloudXYZRGB());

    for (size_t i = 0; i < cloudKeyPoses6D->size(); ++i)
    {
        Eigen::Affine3f transform = pclPointToAffine3f(cloudKeyPoses6D->points[i]);
        PointCloudXYZRGB::Ptr original_cloud = surfCloudKeyFrames[i];
        PointCloudXYZRGB::Ptr transformed_cloud(new PointCloudXYZRGB());
        pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
        *globalMapCloud += *transformed_cloud;
    }
    
    std::string save_path;
    if (is_optimized)
    {
        save_path = global_map_pcd_path_ + "after_optimization.pcd";
    }
    else
    {
        save_path = global_map_pcd_path_ + "before_optimization.pcd";
    }
    // 防御：空点云直接跳过，避免 pcl::io::savePCDFileBinary 抛 pcl::IOException
    // ("Input point cloud has no data!")——未捕获会 terminate 整个进程。
    if (globalMapCloud->empty())
    {
        ROS_WARN("[saveOptimizedGlobalMap] global map cloud is empty, skip writing %s", save_path.c_str());
        return;
    }
    // 目录不存在时 PCL 抛 pcl::IOException("Error during open!")，未捕获会 terminate 整个进程
    ensureOutDir(global_map_pcd_path_, "saveOptimizedGlobalMap");
    try
    {
        pcl::io::savePCDFileBinary(save_path, *globalMapCloud);
        ROS_INFO("[saveOptimizedGlobalMap] saved %zu pts -> %s", globalMapCloud->size(), save_path.c_str());
    }
    catch (const std::exception &e)
    {
        ROS_WARN("[saveOptimizedGlobalMap] write failed: %s (%s)", save_path.c_str(), e.what());
    }
}

void optimization::writeOptimizedTumTrajectory() {
    if (cloudKeyPoses6D->empty()) {
        ROS_WARN("No optimized keyframes to write to TUM file.");
        return;
    }

    std::string final_path = opt_tum_output_path_;
    std::ofstream tum_file(final_path);
    if (!tum_file.is_open()) {
        ROS_WARN("[writeOptimizedTumTrajectory] open failed: %s", final_path.c_str());
        return;
    }
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);
    for (const auto& pose : cloudKeyPoses6D->points) {
        tf::Quaternion q = tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw);
        tum_file << pose.time << " "
                 << pose.x << " " << pose.y << " " << pose.z << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    tum_file.close();
}

void optimization::writeRtkTumTrajectory() {
    if (gpsQueue.empty()) {
        ROS_WARN("No RTK data to write to TUM file.");
        return;
    }
    
    std::ofstream tum_file(gps_tum_output_path_);
    if (!tum_file.is_open()) {
        ROS_WARN("[writeRtkTumTrajectory] open failed: %s", gps_tum_output_path_.c_str());
        return;
    }
    tum_file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    tum_file << std::fixed << std::setprecision(6);

    for (const auto& odom_msg : gpsQueue) {
        tum_file << odom_msg.header.stamp.toSec() << " " 
                 << odom_msg.pose.pose.position.x << " " 
                 << odom_msg.pose.pose.position.y << " " 
                 << odom_msg.pose.pose.position.z << " "
                 << odom_msg.pose.pose.orientation.x << " " 
                 << odom_msg.pose.pose.orientation.y << " " 
                 << odom_msg.pose.pose.orientation.z << " " 
                 << odom_msg.pose.pose.orientation.w << std::endl;
    }

    tum_file.close();
}

void optimization::loadData(const std::string& data_dir)
{
    ROS_INFO("Loading data from directory: %s", data_dir.c_str());

    // Load RTK/GPS  
    std::string rtk_path = data_dir + "/rtk.txt"; 
    std::ifstream rtk_file(rtk_path);

    if (rtk_file.is_open()) {
        std::string line;
        int rtk_count = 0;
        gpsQueue.clear(); 

        while (std::getline(rtk_file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            double t, x, y, z, vx, vy, vz, h_acc, v_acc;
            ss >> t >> x >> y >> z ;
            
            nav_msgs::Odometry gps_msg;
            gps_msg.header.stamp = ros::Time().fromSec(t) - ros::Duration(gps_offset);
            gps_msg.header.frame_id = "map"; 

            gps_msg.pose.pose.position.x = x;
            gps_msg.pose.pose.position.y = y;
            gps_msg.pose.pose.position.z = z;
            gps_msg.pose.pose.orientation.w = 1.0;

            gpsQueue.push_back(gps_msg);
            rtk_count++;
        }
        rtk_file.close();
        ROS_INFO("Loaded %d RTK measurements.", rtk_count);
    } else {
        ROS_WARN("Failed to open RTK file: %s", rtk_path.c_str());
    }

    // Load KeyFrames (Odom + PCD) 
    std::string odom_path = data_dir + "/odom.txt";
    std::string cov_path  = data_dir + "/cov.txt";
    std::string pcd_dir   = data_dir + "pcd/";

    std::ifstream odom_file(odom_path);
    std::ifstream cov_file(cov_path);

    if (!odom_file.is_open() || !cov_file.is_open()) {
        ROS_ERROR("Failed to open trajectory files. Check path: %s", data_dir.c_str());
        return;
    }

    std::string line_odom, line_cov;
    int processed_count = 0;

    while (std::getline(odom_file, line_odom) && std::getline(cov_file, line_cov)) {
        
        if (line_odom.empty() || line_odom[0] == '#') continue;
        if (line_cov.empty() || line_cov[0] == '#') continue; 

        std::stringstream ss_odom(line_odom);
        std::stringstream ss_cov(line_cov);

        double t_odom, t_cov;
        ss_odom >> t_odom;
        ss_cov >> t_cov;

        if (std::abs(t_odom - t_cov) > 1e-5) {
            ROS_ERROR("Critical Error: Timestamp mismatch at line %d!", processed_count + 1);
            ROS_ERROR("Odom Time: %.6f, Cov Time: %.6f", t_odom, t_cov);
            break; 
        }

        // Odom 
        double tx, ty, tz, qx, qy, qz, qw;
        ss_odom >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        timeLaserInfoCur = t_odom;
        timeLaserInoStamp = ros::Time().fromSec(t_odom);

        tf::Quaternion orientation(qx, qy, qz, qw);
        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        
        transformTobeMapped[0] = roll;
        transformTobeMapped[1] = pitch;
        transformTobeMapped[2] = yaw;
        transformTobeMapped[3] = tx;
        transformTobeMapped[4] = ty;
        transformTobeMapped[5] = tz;

        // PCD
        std::stringstream ss_filename;
        ss_filename << std::setw(6) << std::setfill('0') << processed_count;
        std::string pcd_path = pcd_dir + ss_filename.str() + ".pcd";

        laserCloudSurfLastDS->clear();
        if (pcl::io::loadPCDFile(pcd_path, *laserCloudSurfLastDS) == -1) {
            ROS_ERROR("Missing PCD file for timestamp %.6f: %s", t_odom, pcd_path.c_str());
            continue; 
        }

        saveKeyFramesAndFactor();
        processed_count++;
    }
    
    odom_file.close();
    cov_file.close();
    ROS_INFO("Load complete. Total frames: %d", processed_count);
}