/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
LIVMapper::LIVMapper(ros::NodeHandle &nh)
    : extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters(nh);
  VoxelMapConfig voxel_config;
  loadVoxelConfig(nh, voxel_config);

  visual_sub_map.reset(new PointCloudXYZI());
  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZRGB());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  vio_manager.reset(new VIOManager());
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters(ros::NodeHandle &nh)
{
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
  nh.param<bool>("common/ros_driver_bug_fix", ros_driver_fix_en, false);
  nh.param<int>("common/img_en", img_en, 1);
  nh.param<int>("common/lidar_en", lidar_en, 1);
  nh.param<string>("common/img_topic", img_topic, "/left_camera/image");

  nh.param<bool>("vio/normal_en", normal_en, true);
  nh.param<bool>("vio/inverse_composition_en", inverse_composition_en, false);
  nh.param<int>("vio/max_iterations", max_iterations, 5);
  nh.param<double>("vio/img_point_cov", IMG_POINT_COV, 100);
  nh.param<bool>("vio/raycast_en", raycast_en, false);
  nh.param<bool>("vio/exposure_estimate_en", exposure_estimate_en, true);
  nh.param<double>("vio/inv_expo_cov", inv_expo_cov, 0.2);
  nh.param<int>("vio/grid_size", grid_size, 5);
  nh.param<int>("vio/grid_n_height", grid_n_height, 17);
  nh.param<int>("vio/patch_pyrimid_level", patch_pyrimid_level, 3);
  nh.param<int>("vio/patch_size", patch_size, 8);
  nh.param<double>("vio/outlier_threshold", outlier_threshold, 1000);

  nh.param<double>("time_offset/exposure_time_init", exposure_time_init, 0.0);
  nh.param<double>("time_offset/img_time_offset", img_time_offset, 0.0);
  nh.param<double>("time_offset/imu_time_offset", imu_time_offset, 0.0);
  nh.param<double>("time_offset/lidar_time_offset", lidar_time_offset, 0.0);
  nh.param<bool>("uav/imu_rate_odom", imu_prop_enable, false);
  nh.param<bool>("uav/gravity_align_en", gravity_align_en, false);

  nh.param<string>("evo/seq_name", seq_name, "01");
  nh.param<bool>("evo/pose_output_en", pose_output_en, false);
  nh.param<double>("imu/gyr_cov", gyr_cov, 1.0);
  nh.param<double>("imu/acc_cov", acc_cov, 1.0);
  nh.param<int>("imu/imu_int_frame", imu_int_frame, 3);
  nh.param<bool>("imu/imu_en", imu_en, false);
  nh.param<bool>("imu/gravity_est_en", gravity_est_en, true);
  nh.param<bool>("imu/ba_bg_est_en", ba_bg_est_en, false);

  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<double>("preprocess/filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<bool>("preprocess/hilti_en", hilti_en, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 6);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 3);
  nh.param<bool>("preprocess/feature_extract_enabled", p_pre->feature_enabled, false);

  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<bool>("pcd_save/colmap_output_en", colmap_output_en, false);
  nh.param<double>("pcd_save/filter_size_pcd", filter_size_pcd, 0.5);
  // 内外参改为从标定文件夹直接读取（格式/目录结构同 Global-LVBA 的 keyframe 模式），
  // 不再从 extrin_calib/extrinsic_T 等参数服务器上的固定数值读取。
  nh.param<std::string>("extrin_calib/calib_path", calib_path_, "");
  nh.param<std::string>("extrin_calib/lidar_frame", lidar_frame_, "hesai128");
  nh.param<std::string>("extrin_calib/vehicle_frame", vehicle_frame_, "vehicle");
  // IMU(ins) 坐标系名：lidar->IMU 外参必须查到 IMU 系，而不是 vehicle 系。
  // 本数据集里 ins 相对 vehicle 有 180°翻转+1.1m 平移，若错用 vehicle 会让外参差一个 180°。
  nh.param<std::string>("extrin_calib/imu_frame", imu_frame_, "ins");
  nh.param<std::string>("extrin_calib/camera_frame", camera_frame_, "cam1");
  nh.param<double>("extrin_calib/cam_scale", cam_scale_, 1.0);
  nh.param<bool>("extrin_calib/undistort_image", undistort_en_, true);  // 图像入口整幅去畸变
  nh.param<double>("debug/plot_time", plot_time, -10);
  nh.param<int>("debug/frame_cnt", frame_cnt, 6);

  nh.param<double>("publish/blind_rgb_points", blind_rgb_points, 0.01);
  nh.param<int>("publish/pub_scan_num", pub_scan_num, 1);
  nh.param<bool>("publish/pub_effect_point_en", pub_effect_point_en, false);
  nh.param<bool>("publish/dense_map_en", dense_map_en, false);

  nh.param<bool>("gps/debug_mode", debug_mode, false);
  p_pre->blind_sqr = p_pre->blind * p_pre->blind;

  //nh.param<bool>("gps/gps_en", rtk_en, false);
  // 用 /localization/odom 当 RTK 观测：odom_rtk_en=true 时不再订阅 GNSS PVT，
  // 直接把该里程计的位置/速度塞进 rtk_buffer（rtk_en 随之打开，否则 sync_packages 不消费）
  nh.param<bool>("gps/odom_rtk_en", odom_rtk_en_, true);
  nh.param<std::string>("gps/odom_rtk_topic", odom_rtk_topic_, "/localization/odom");
  if (odom_rtk_en_) rtk_en = true;
  // 断流恢复健壮性处理总开关，见 LIVMapper.h 里的注释；默认 false，行为与之前一致。
  nh.param<bool>("gps/rtk_robust_recovery_en", rtk_robust_recovery_en_, false);
  nh.param<double>("gps/rtk_gap_thresh", rtk_gap_thresh_, 1.0);
  nh.param<double>("gps/rtk_innov_gate", rtk_innov_gate_, 5.0);
  nh.param<vector<double>>("gps/extrinsic_T", T_I_R, vector<double>());
  nh.param<std::string>("laserMapping/outputfilepath", save_directory, "");
  // ==== 导出为 Global-LVBA keyframe 数据集（二次 LiDAR-Visual BA 用）====
  nh.param<bool>("lvba_export/enable", save_lvba_en_, false);
  nh.param<std::string>("lvba_export/out_dir", lvba_out_dir_, save_directory + "/lvba_dataset");
  pcd_save_file = save_directory + "/debug/pcd/";
  rtk_save_file = save_directory + "/debug/rtk.txt";
  imu_save_file = save_directory + "/debug/imu.txt";
  odom_save_file = save_directory + "/debug/odom.txt"; 
  cov_save_file  = save_directory + "/debug/cov.txt"; 
}

// 从标定文件夹加载内参(相机)+外参(lidar->vehicle、cam->lidar)，替代原来从 ROS 参数服务器
// 读固定数值/走 vk::camera_loader::loadFromRosNs 的方式。文件格式/目录结构与 Global-LVBA
// 的 keyframe 模式（DatasetIO::loadIntrinsicsCustom/loadExtrinsicsCustom）保持一致：
//   calib_path_/intrinsics/<camera_frame_>_Intrinsic.yaml   (cameraMatrix/distCoeffs/ImageWidth/ImageHeight)
//   calib_path_/extrinsics/{id}_{sensor}_2_{parent}_Extrinsics.yaml (r_quaternion_wxyz + t_metric_xyz)
bool LIVMapper::loadCalibrationFromFiles()
{
  namespace fs = std::filesystem;

  const std::string intr_file = calib_path_ + "/intrinsics/" + camera_frame_ + "_Intrinsic.yaml";
  if (!fs::exists(intr_file))
  {
    ROS_ERROR("[LIVMapper] intrinsics file not found: %s", intr_file.c_str());
    return false;
  }
  double fx, fy, cx, cy;
  int width, height;
  std::vector<double> dist_coeffs;   // 读全部畸变系数（本相机是 8 参数 rational 模型）
  try
  {
    YAML::Node r = YAML::LoadFile(intr_file);
    auto K = r["cameraMatrix"]["data"];   // [fx 0 cx 0 fy cy 0 0 1]
    auto D = r["distCoeffs"]["data"];     // [k1 k2 p1 p2 k3 (k4 k5 k6)]
    fx = K[0].as<double>(); cx = K[2].as<double>();
    fy = K[4].as<double>(); cy = K[5].as<double>();
    for (std::size_t i = 0; i < D.size(); ++i) dist_coeffs.push_back(D[i].as<double>());
    width  = r["ImageWidth"].as<int>();
    height = r["ImageHeight"].as<int>();
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("[LIVMapper] failed to parse intrinsics %s: %s", intr_file.c_str(), e.what());
    return false;
  }

  // 是否真的有畸变（任一系数非零）
  bool has_distortion = false;
  for (double c : dist_coeffs) if (std::fabs(c) > 1e-12) { has_distortion = true; break; }

  // 构建整幅去畸变映射（全分辨率、newK=K，保持内参不变；随后 VIO 内部再按 cam_scale resize）。
  // 用完整 distCoeffs（8 参数 rational），cv::initUndistortRectifyMap 会自动按系数个数选模型。
  undistort_ready_ = false;
  if (undistort_en_ && has_distortion)
  {
    cv::Mat Kfull = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cv::Mat Dmat(1, static_cast<int>(dist_coeffs.size()), CV_64F);
    for (int i = 0; i < (int)dist_coeffs.size(); ++i) Dmat.at<double>(0, i) = dist_coeffs[i];
    cv::initUndistortRectifyMap(Kfull, Dmat, cv::Mat(), Kfull, cv::Size(width, height),
                                CV_16SC2, undist_map1_, undist_map2_);
    calib_img_width_  = width;
    calib_img_height_ = height;
    undistort_ready_  = true;
  }

  // 相机模型：整幅去畸变后图像已是 rectified，投影按纯针孔（零畸变），避免与 world2cam 重复加畸变。
  // 若关闭了整幅去畸变(undistort_en_=false)则退回 vk 解析畸变(仅支持到 k3，rational 高阶项会被忽略)。
  if (undistort_ready_)
    vio_manager->cam = new vk::PinholeCamera(width, height, cam_scale_, fx, fy, cx, cy, 0, 0, 0, 0);
  else
  {
    double k1 = dist_coeffs.size() > 0 ? dist_coeffs[0] : 0.0;
    double k2 = dist_coeffs.size() > 1 ? dist_coeffs[1] : 0.0;
    double p1 = dist_coeffs.size() > 2 ? dist_coeffs[2] : 0.0;
    double p2 = dist_coeffs.size() > 3 ? dist_coeffs[3] : 0.0;
    vio_manager->cam = new vk::PinholeCamera(width, height, cam_scale_, fx, fy, cx, cy, k1, k2, p1, p2);
  }

  {
    std::string dstr;
    for (double c : dist_coeffs) { char b[32]; std::snprintf(b, sizeof(b), "%.6g ", c); dstr += b; }
    ROS_INFO("[LIVMapper] loaded intrinsics from %s: %dx%d(scale=%.3f) fx=%.3f fy=%.3f cx=%.3f cy=%.3f dist(%zu)=[%s]",
             intr_file.c_str(), width, height, cam_scale_, fx, fy, cx, cy, dist_coeffs.size(), dstr.c_str());
    if (undistort_ready_)
      ROS_INFO("[LIVMapper] image UNDISTORT enabled: full-res remap %dx%d built (%zu dist coeffs); camera model set to PINHOLE(no distortion)",
               width, height, dist_coeffs.size());
    else if (undistort_en_ && !has_distortion)
      ROS_WARN("[LIVMapper] undistort requested but all dist coeffs are zero -> skip (image treated as already rectified)");
    else if (!undistort_en_)
      ROS_WARN("[LIVMapper] image undistort DISABLED (extrin_calib/undistort_image=false) -> using vk analytic distortion (rational k4..k6 ignored!)");
  }

  const std::string ext_folder = calib_path_ + "/extrinsics";
  if (!fs::is_directory(ext_folder))
  {
    ROS_ERROR("[LIVMapper] extrinsics folder not found: %s", ext_folder.c_str());
    return false;
  }
  SensorExtrinsics ext;
  ext.loadFolder(ext_folder);

  // lidar->IMU 外参：必须查到 IMU(ins) 系。之前误用 vehicle 系，而本数据集 ins 相对 vehicle
  // 有 180°翻转+1.1m，会让外参整体差一个 180° 翻转，导致点云相对 IMU 重力/运动翻转。
  Eigen::Matrix4d T_imu_lidar, T_cam_lidar;
  if (!ext.lookupTransform(imu_frame_, lidar_frame_, T_imu_lidar))
  {
    ROS_ERROR("[LIVMapper] lookup T_%s_%s failed (check filenames under %s)",
               imu_frame_.c_str(), lidar_frame_.c_str(), ext_folder.c_str());
    return false;
  }
  if (!ext.lookupTransform(camera_frame_, lidar_frame_, T_cam_lidar))
  {
    ROS_ERROR("[LIVMapper] lookup T_%s_%s failed (check filenames under %s)",
               camera_frame_.c_str(), lidar_frame_.c_str(), ext_folder.c_str());
    return false;
  }

  // T_imu_lidar = lidar->IMU 外参（喂给 setImuToLidarExtrinsic / p_imu）
  extrinT = {T_imu_lidar(0, 3), T_imu_lidar(1, 3), T_imu_lidar(2, 3)};
  extrinR = {T_imu_lidar(0, 0), T_imu_lidar(0, 1), T_imu_lidar(0, 2),
             T_imu_lidar(1, 0), T_imu_lidar(1, 1), T_imu_lidar(1, 2),
             T_imu_lidar(2, 0), T_imu_lidar(2, 1), T_imu_lidar(2, 2)};
  // T_cam_lidar 就是 lidar->camera 外参 Rcl/Pcl
  cameraextrinT = {T_cam_lidar(0, 3), T_cam_lidar(1, 3), T_cam_lidar(2, 3)};
  cameraextrinR = {T_cam_lidar(0, 0), T_cam_lidar(0, 1), T_cam_lidar(0, 2),
                   T_cam_lidar(1, 0), T_cam_lidar(1, 1), T_cam_lidar(1, 2),
                   T_cam_lidar(2, 0), T_cam_lidar(2, 1), T_cam_lidar(2, 2)};

  // 详细打印最终外参，方便肉眼确认对不对（旋转以欧拉角 deg 展示）
  auto eulerDeg = [](const Eigen::Matrix4d &Tm) {
    Eigen::Matrix3d R = Tm.block<3, 3>(0, 0);
    double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
    double roll  = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI;
    double pitch = std::atan2(-R(2, 0), sy) * 180.0 / M_PI;
    double yaw   = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
    char buf[128]; std::snprintf(buf, sizeof(buf), "roll=%.2f pitch=%.2f yaw=%.2f", roll, pitch, yaw);
    return std::string(buf);
  };
  ROS_INFO("[LIVMapper][extrinsic] lidar->IMU  (%s <- %s): t=[%.4f, %.4f, %.4f]  %s",
           imu_frame_.c_str(), lidar_frame_.c_str(), extrinT[0], extrinT[1], extrinT[2],
           eulerDeg(T_imu_lidar).c_str());
  ROS_INFO("[LIVMapper][extrinsic] lidar->cam  (%s <- %s): t=[%.4f, %.4f, %.4f]  %s",
           camera_frame_.c_str(), lidar_frame_.c_str(), cameraextrinT[0], cameraextrinT[1], cameraextrinT[2],
           eulerDeg(T_cam_lidar).c_str());
  ROS_WARN("[LIVMapper][extrinsic] 若 lidar->IMU 的 roll≈±180 属正常（本车 ins 相对 vehicle 翻转安装）；"
           "若这里又变回 roll≈0，说明 imu_frame 配错（配成 vehicle 了）");
  return true;
}

// ============================================================================
// 导出为 Global-LVBA keyframe 数据集（DatasetIO::loadFramesKeyframe 读取）
//   pointclouds/{idx}_{ts}.pcd  —— 激光系点云 (PointXYZI)
//   odoms/{idx}_{ts}.yaml       —— T_utm_ins（= 本系统 IMU/ins 世界位姿 _state），
//                                   position:[x y z] + pose:[3x3 行主序]
//   images/<cam>/{idx}_{ts}_<cam>.jpg —— 去畸变全分辨率图像
//   calib/                      —— 复制自 calib_path_（Global-LVBA 按纯针孔投影，忽略畸变系数）
//
// 几何自洽：Global-LVBA 用 world = T_utm_ins · T_ins_vehicle · T_vehicle_lidar · p_lidar，
//   而 T_ins_vehicle·T_vehicle_lidar = T_ins_lidar = 本系统 extR/extT，T_utm_ins = _state，
//   点云存激光系，故两边世界坐标完全一致（前提：Global-LVBA 用同一个 calib 文件夹）。
// LIO 关键帧先写点云+odom(LIO 位姿)；同帧 VIO 再写图并用 VIO 位姿覆写 odom（更优）。
// ============================================================================
// 防御性建目录（失败不抛，交由后续写文件的 try/catch 兜底）
static inline void ensureLvbaDir(const std::string &dir)
{
  try { std::filesystem::create_directories(dir); } catch (...) {}
}

// ts 传全精度时间戳：文件名里的 ts 已截成 3 位小数，这里在 yaml 内保留完整精度。
// 注意 StatesGroup 无 timestamp 成员，故时间戳由调用方显式传入。
bool LIVMapper::writeLvbaOdom(const std::string &stem, const StatesGroup &st, double ts)
{
  const std::string odom_dir  = lvba_out_dir_ + "/odoms";
  const std::string odom_path = odom_dir + "/" + stem + ".yaml";
  try
  {
    ensureLvbaDir(odom_dir);
    std::ofstream f(odom_path);
    if (!f.is_open())
    {
      ROS_WARN_THROTTLE(5.0, "[LVBA export] 打不开 odom: %s", odom_path.c_str());
      return false;
    }
    const V3D &p = st.pos_end;
    const M3D &R = st.rot_end;
    f << std::fixed << std::setprecision(3);
    f << "timestamp: [" << ts << "]\n";
    f << std::fixed << std::setprecision(9);
    f << "position: [" << p(0) << ", " << p(1) << ", " << p(2) << "]\n";
    f << "pose: [";
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
      {
        f << R(r, c);
        if (!(r == 2 && c == 2)) f << ", ";
      }
    f << "]\n";
    return f.good();
  }
  catch (const std::exception &e)
  {
    ROS_WARN_THROTTLE(5.0, "[LVBA export] 写 odom 失败: %s (%s)", odom_path.c_str(), e.what());
    return false;
  }
}

void LIVMapper::saveLvbaKeyframeLidar()
{
  if (!save_lvba_en_) return;
  if (feats_undistort == nullptr || feats_undistort->empty()) return;

  // 候选序号 + 主干 {idx}_{ts}（ts 保留 3 位小数）——点云写成功后才提交序号/stem，
  // 保证 idx 连续、不产生孤儿 odom/图像。pcd/odom/image 三者共用同一 stem 严格配对。
  const int cand_idx = lvba_kf_index_ + 1;
  char stem_buf[128];
  snprintf(stem_buf, sizeof(stem_buf), "%d_%.3f", cand_idx, keyframe_time);
  const std::string cand_stem(stem_buf);

  // 1) 激光系去畸变点云 -> pointclouds/{stem}.pcd（转成 PointXYZI，Global-LVBA 读的类型）
  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  cloud_out.reserve(feats_undistort->size());
  for (const auto &pt : feats_undistort->points)
  {
    pcl::PointXYZI q;
    q.x = pt.x; q.y = pt.y; q.z = pt.z; q.intensity = pt.intensity;
    cloud_out.push_back(q);
  }
  const std::string pcd_dir  = lvba_out_dir_ + "/pointclouds";
  const std::string pcd_path = pcd_dir + "/" + cand_stem + ".pcd";
  try
  {
    ensureLvbaDir(pcd_dir);
    // 注意：PCL 写失败会抛 pcl::IOException（不是返回码），必须 try/catch，否则整个节点 terminate
    pcl::io::savePCDFileBinary(pcd_path, cloud_out);
  }
  catch (const std::exception &e)
  {
    ROS_WARN_THROTTLE(5.0, "[LVBA export] 存点云失败(跳过该帧): %s (%s)", pcd_path.c_str(), e.what());
    return;  // 不提交序号/stem、不写 odom，避免孤儿文件
  }

  // 提交
  lvba_kf_index_    = cand_idx;
  lvba_cur_stem_    = cand_stem;
  lvba_cur_kf_time_ = keyframe_time;

  // 2) IMU/ins 世界位姿 -> odoms/{stem}.yaml（此处为 LIO 位姿；同帧 VIO 会用更优位姿覆写）
  writeLvbaOdom(lvba_cur_stem_, _state, keyframe_time);
}

void LIVMapper::saveLvbaKeyframeImageOdom(const cv::Mat &img)
{
  if (!save_lvba_en_) return;
  if (img.empty()) return;
  // 只在“当前 VIO 帧 == 刚存过激光的关键帧”时才写，避免 LIO 早退/写失败时用陈旧 stem 配错图
  if (lvba_kf_index_ < 0 || lvba_cur_stem_.empty()) return;
  if (std::fabs(lvba_cur_kf_time_ - keyframe_time) > 1e-6) return;

  // 用 VIO 更新后的更优位姿覆写 odom
  writeLvbaOdom(lvba_cur_stem_, _state, keyframe_time);

  // 去畸变全分辨率图 -> images/<cam>/{stem}_<cam>.jpg
  const std::string img_dir  = lvba_out_dir_ + "/images/" + camera_frame_;
  const std::string img_path = img_dir + "/" + lvba_cur_stem_ + "_" + camera_frame_ + ".jpg";
  try
  {
    ensureLvbaDir(img_dir);
    if (!cv::imwrite(img_path, img))  // 失败可能返回 false，也可能抛 cv::Exception
      ROS_WARN_THROTTLE(5.0, "[LVBA export] 存图像失败: %s", img_path.c_str());
  }
  catch (const std::exception &e)
  {
    ROS_WARN_THROTTLE(5.0, "[LVBA export] 存图像异常: %s (%s)", img_path.c_str(), e.what());
  }
}

void LIVMapper::initializeComponents()
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  if (!loadCalibrationFromFiles())
    throw std::runtime_error("Failed to load calibration (intrinsics/extrinsics) from: " + calib_path_);

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);
  p_imu->rtk_robust_recovery_en = rtk_robust_recovery_en_;

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() 
{
  // debug 输出目录先建好：这些路径下用 PCL 写盘，目录不存在会抛 pcl::IOException 把节点搞崩
  if (!save_directory.empty())
  {
    try
    {
      std::filesystem::create_directories(save_directory + "/debug/pcd");
      std::filesystem::create_directories(save_directory + "/TUM");
      std::filesystem::create_directories(save_directory + "/vel");
    }
    catch (const std::exception &e)
    {
      ROS_WARN("[initializeFiles] create debug dirs failed: %s", e.what());
    }
  }
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_interval > 0) fout_pcd_pos.open(std::string(ROOT_DIR) + "Log/PCD/scans_pos.json", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);

  // ==== Global-LVBA keyframe 数据集导出：建目录 + 复制标定，使数据集自包含 ====
  if (save_lvba_en_)
  {
    namespace fs = std::filesystem;
    try
    {
      fs::create_directories(lvba_out_dir_ + "/pointclouds");
      fs::create_directories(lvba_out_dir_ + "/odoms");
      fs::create_directories(lvba_out_dir_ + "/images/" + camera_frame_);
      // 复制标定文件夹（intrinsics/ + extrinsics/）到 calib/，Global-LVBA keyframe 直接读。
      // 单独 try/catch：复制失败（如二次运行目标已存在）不应禁掉整个导出。
      const std::string calib_dst = lvba_out_dir_ + "/calib";
      if (calib_path_.empty() || !fs::exists(calib_path_))
      {
        ROS_WARN("[LVBA export] calib_path 无效，未复制标定；请手动把标定放到 %s/", calib_dst.c_str());
      }
      else if (fs::exists(calib_dst + "/intrinsics"))
      {
        ROS_INFO("[LVBA export] calib 已存在，跳过复制: %s", calib_dst.c_str());
      }
      else
      {
        try
        {
          fs::create_directories(calib_dst);
          fs::copy(calib_path_, calib_dst,
                   fs::copy_options::recursive | fs::copy_options::overwrite_existing);
        }
        catch (const std::exception &e)
        {
          ROS_WARN("[LVBA export] 复制标定失败（请手动放到 %s/）: %s", calib_dst.c_str(), e.what());
        }
      }
      ROS_INFO("[LVBA export] enabled, output dir: %s", lvba_out_dir_.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("[LVBA export] 建目录/复制标定失败，禁用导出: %s", e.what());
      save_lvba_en_ = false;
    }
  }
}

void LIVMapper::initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it) 
{
  sub_pcl = p_pre->lidar_type == AVIA ? 
            nh.subscribe(lid_topic, 200000, &LIVMapper::livox_pcl_cbk, this): 
            nh.subscribe(lid_topic, 200000, &LIVMapper::standard_pcl_cbk, this);
  sub_imu = nh.subscribe(imu_topic, 200000, &LIVMapper::imu_cbk, this);
  sub_img = nh.subscribe(img_topic, 200000, &LIVMapper::img_cbk, this);
  
  // RTK 观测二选一：默认 GNSS PVT；odom_rtk_en=true 时改用 /localization/odom
  if (odom_rtk_en_)
  {
    sub_localization_odom = nh.subscribe<nav_msgs::Odometry>(odom_rtk_topic_, 2000, &LIVMapper::localization_odom_cbk, this);
    ROS_INFO("[RTK] use odometry topic as RTK source: %s", odom_rtk_topic_.c_str());
  }
  else
  {
    subGPS_pvt = nh.subscribe<gnss_comm::GnssPVTSolnMsg>("/ublox_driver/receiver_pvt", 2000, &LIVMapper::rtk_cbk, this);
  }

  pub_odom = nh.advertise<nav_msgs::Odometry>("/odometry/fast_livo2", 10000);
  pub_lidarRGB = nh.advertise<sensor_msgs::PointCloud2>("/synced_cloud", 10000);
  pub_rtk = nh.advertise<nav_msgs::Odometry>("/gps/odometry", 10000);


  pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
  pubNormal = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 100);
  pubSubVisualMap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_visual_sub_map_before", 100);
  pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 10);
  pubPath = nh.advertise<nav_msgs::Path>("/path", 10);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/planner_normal", 1);
  voxel_pub = nh.advertise<visualization_msgs::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = nh.advertise<sensor_msgs::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImage = it.advertise("/rgb_img", 1);
  pubImuPropOdom = nh.advertise<nav_msgs::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = nh.createTimer(ros::Duration(0.004), &LIVMapper::imu_prop_callback, this);
  voxelmap_manager->voxel_map_pub_= nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort, T_G_to_W);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();
      ROS_INFO("[HandleLIO] Position after: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
      ROS_INFO("[HandleLIO] p_cov after: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));

      if(rtk_en && rtk_ini)
      {
        handleRTK();
      }
      break;
  }
}

// gps/rtk_robust_recovery_en 打开时用于门限校验 RTK 位置新息：按轴比较残差和
// sqrt(当前状态位置协方差 + 观测噪声方差)，超过 rtk_innov_gate_ 倍就判定为不可信
// （典型场景：断流恢复瞬间的第一批样本、或上游定位节点自身跳变/野值）。
bool LIVMapper::rtkInnovationOk(const Eigen::Vector3d &rtk_pos, const Eigen::Vector3d &meas_std)
{
  const Eigen::Vector3d residual = rtk_pos - _state.pos_end;
  for (int i = 0; i < 3; i++)
  {
    double sigma = std::sqrt(std::max(_state.cov(3 + i, 3 + i), 0.0) + meas_std[i] * meas_std[i]);
    sigma = std::max(sigma, 1e-6);
    if (std::fabs(residual[i]) > rtk_innov_gate_ * sigma) return false;
  }
  return true;
}

void LIVMapper::handleRTK()
{
  ROS_INFO("%s[ RTK Update ]%s", BLUE, RESET);
  if (LidarMeasures.measures.empty()) 
  {
      return; 
  }

  auto rtk_data = LidarMeasures.measures.back().rtk; 
  if(rtk_data.timestamp < 0.00001)
  {
    return;
  }

  std::deque temp_measures = LidarMeasures.measures;

  //ROS_INFO("[HandleRTK] rtk_data.p: [%.6f, %.6f, %.6f]", rtk_data.p[0], rtk_data.p[1], rtk_data.p[2]);
  Eigen::Vector3d z_k = rtk_data.p; 
  z_k[2] = _state.pos_end(2);
  Eigen::Vector3d T_I_to_R;
  T_I_to_R[0] = T_I_R[0];
  T_I_to_R[1] = T_I_R[1];
  T_I_to_R[2] = T_I_R[2];
  ROS_INFO("[HandleRTK] RTK_W: [%f, %f, %f]", z_k(0), z_k(1), z_k(2));
  ROS_INFO("[HandleRTK] Position before: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleRTK] p_cov before: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  ROS_INFO("[HandleRTK] Rotation R_before(R1): [%.6f, %.6f, %.6f]", _state.rot_end(0,0), _state.rot_end(0,1), _state.rot_end(0,2));
  ROS_INFO("[HandleRTK] Rotation R_before(R2): [%.6f, %.6f, %.6f]", _state.rot_end(1,0), _state.rot_end(1,1), _state.rot_end(1,2));
  ROS_INFO("[HandleRTK] Rotation R_before(R3): [%.6f, %.6f, %.6f]", _state.rot_end(2,0), _state.rot_end(2,1), _state.rot_end(2,2));

  Eigen::Matrix3d R_cov = Eigen::Matrix3d::Identity() * 1e-4 * 5;

  
  Eigen::Matrix<double, 3, DIM_STATE> H;
  H.setZero();
  H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> P = _state.cov;
  Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R_cov;
  Eigen::Matrix<double, DIM_STATE, 3> K = P * H.transpose() * S.inverse();
  Eigen::Vector3d y = z_k - (_state.pos_end + _state.rot_end * T_I_to_R);
  ROS_INFO("[HandleRTK] y: [%f, %f, %f]", y(0), y(1), y(2));

  if (rtk_robust_recovery_en_)
  {
    double maha = std::sqrt(std::max((y.transpose() * S.inverse() * y)(0, 0), 0.0));
    if (maha > rtk_innov_gate_)
    {
      ROS_WARN("[HandleRTK] innovation gate failed: maha=%.2f sigma > gate=%.2f, skip this RTK correction", maha, rtk_innov_gate_);
      return;
    }
  }

  Eigen::Matrix<double, DIM_STATE, 1> dx = K * y;
  ROS_INFO("[HandleRTK] dx: [%f, %f, %f, %f, %f, %f]", dx(0), dx(1), dx(2), dx(3), dx(4), dx(5));
  _state += dx;
  
  if (_state.gravity.norm() > 0.1) { 
      _state.gravity += dx.segment<3>(16);
  }

  Eigen::Matrix<double, DIM_STATE, DIM_STATE> I = Eigen::Matrix<double, DIM_STATE, DIM_STATE>::Identity();
  _state.cov = (I - K * H) * P;
  ROS_INFO("[HandleRTK] Position after: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleRTK] p_cov after: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  ROS_INFO("[HandleRTK] Rotation R_after(R1): [%.6f, %.6f, %.6f]", _state.rot_end(0,0), _state.rot_end(0,1), _state.rot_end(0,2));
  ROS_INFO("[HandleRTK] Rotation R_after(R2): [%.6f, %.6f, %.6f]", _state.rot_end(1,0), _state.rot_end(1,1), _state.rot_end(1,2));
  ROS_INFO("[HandleRTK] Rotation R_after(R3): [%.6f, %.6f, %.6f]", _state.rot_end(2,0), _state.rot_end(2,1), _state.rot_end(2,2));
  // std::cout << "[RTK Update] Pos Correction: " << dx.segment<3>(3).transpose() << std::endl;
}

void LIVMapper::handleVIO() 
{
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << std::endl;
    
  if (pcl_w_wait_pub->empty() || (pcl_w_wait_pub == nullptr)) 
  {
    std::cout << "[ VIO ] No point!!!" << std::endl;
    return;
  }
    
  //std::cout << "[ VIO ] Raw feature num: " << pcl_w_wait_pub->points.size() << std::endl;

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) - plot_time) < (frame_cnt / 2 * 0.1)) 
  {
    vio_manager->plot_flag = true;
  } 
  else 
  {
    vio_manager->plot_flag = false;
  }

  // processFrame 会把传入图像原地 resize 成小图（cam_scale），故先克隆全分辨率去畸变图供导出
  cv::Mat lvba_kf_img;
  if (save_lvba_en_ && !LidarMeasures.measures.empty() && !LidarMeasures.measures.back().img.empty())
    lvba_kf_img = LidarMeasures.measures.back().img.clone();

  vio_manager->processFrame(LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_, LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable)
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  // 导出 Global-LVBA keyframe：用 VIO 更新后的更优位姿覆写 odom + 存去畸变全分辨率图像
  saveLvbaKeyframeImageOdom(lvba_kf_img);

  // int size_sub_map = vio_manager->visual_sub_map_cur.size();
  // visual_sub_map->reserve(size_sub_map);
  // for (int i = 0; i < size_sub_map; i++) 
  // {
  //   PointType temp_map;
  //   temp_map.x = vio_manager->visual_sub_map_cur[i]->pos_[0];
  //   temp_map.y = vio_manager->visual_sub_map_cur[i]->pos_[1];
  //   temp_map.z = vio_manager->visual_sub_map_cur[i]->pos_[2];
  //   temp_map.intensity = 0.;
  //   visual_sub_map->push_back(temp_map);
  // }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::handleLIO() 
{    
  ROS_INFO("%s[ LIO Update ]%s", BLUE, RESET);
  ROS_INFO("[HandleLIO] Position before: [%f, %f, %f]", _state.pos_end(0), _state.pos_end(1), _state.pos_end(2));
  ROS_INFO("[HandleLIO] P_cov before: %.12f, %.12f, %.12f", _state.cov(3,3), _state.cov(4,4), _state.cov(5,5));
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  V3D rtk_data = LidarMeasures.measures.back().rtk.p;
  if(rtk_ini && LidarMeasures.measures.back().rtk.timestamp > 0.00001)
  {
    rtk_good = true;
  }
  else
  {
    rtk_good = false;
  }

  if (rtk_good && rtk_robust_recovery_en_)
  {
    const auto &rtk_std = voxelmap_manager->config_setting_.rtk_std_;
    Eigen::Vector3d meas_std(rtk_std[0], rtk_std[1], rtk_std[2]);
    if (!rtkInnovationOk(rtk_data, meas_std))
    {
      ROS_WARN("[HandleLIO] RTK innovation gate failed, skip RTK rows in StateEstimation this frame");
      rtk_good = false;
    }
  }

  ROS_INFO("[HandleLIO] RTW_W: [%f, %f, %f]", rtk_data(0), rtk_data(1), rtk_data(2));
  voxelmap_manager->StateEstimation(state_propagat, rtk_good, rtk_data);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  // 导出 Global-LVBA keyframe：存激光系点云 + odom(LIO 位姿)；同帧 VIO 会再覆写 odom 并存图
  saveLvbaKeyframeLidar();

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) ROS_ERROR("open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) 
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  //std::cout << "[ LIO ] Update Voxel Map" << std::endl;
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  if (!img_en) publish_frame_world(pubLaserCloudFullRes, vio_manager);
  if (pub_effect_point_en) publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path(pubPath);
  publish_mavros(mavros_pose_publisher);

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  // printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  // printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/PCD/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    if (img_en)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(pcl_wait_save);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;

      if(colmap_output_en)
      {
        fout_points << "# 3D point list with one line of data per point\n";
        fout_points << "#  POINT_ID, X, Y, Z, R, G, B, ERROR\n";
        for (size_t i = 0; i < downsampled_cloud->size(); ++i) 
        {
            const auto& point = downsampled_cloud->points[i];
            fout_points << i << " "
                        << std::fixed << std::setprecision(6)
                        << point.x << " " << point.y << " " << point.z << " "
                        << static_cast<int>(point.r) << " "
                        << static_cast<int>(point.g) << " "
                        << static_cast<int>(point.b) << " "
                        << 0 << std::endl;
        }
      }
    }
    else
    {      
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }
}

void LIVMapper::run() 
{
  ros::Rate rate(5000);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!sync_packages(LidarMeasures))
    {
      // [DEBUG] sync 一直失败时，打印各 buffer 大小，看是哪一路没进来
      ROS_WARN_THROTTLE(2.0,
          "[run] sync_packages FAILED (throttle 2s): lidar_buf=%zu imu_buf=%zu img_buf=%zu | lidar_en=%d imu_en=%d img_en=%d | last_ts lidar=%.3f imu=%.3f img=%.3f",
          lid_raw_data_buffer.size(), imu_buffer.size(), img_buffer.size(),
          lidar_en, imu_en, img_en, last_timestamp_lidar, last_timestamp_imu, last_timestamp_img);
      rate.sleep();
      continue;
    }
    ROS_INFO_THROTTLE(2.0, "[run] sync_packages OK (throttle 2s): lidar_buf=%zu imu_buf=%zu img_buf=%zu",
                      lid_raw_data_buffer.size(), imu_buffer.size(), img_buffer.size());
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;
    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback(const ros::TimerEvent &e)
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (int i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom.publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // [DEBUG] 收到 PointCloud2 就打印（在任何 early-return 之前），确认话题/类型对不对
  ROS_INFO_THROTTLE(1.0, "[cbk][lidar] recv PointCloud2 on '%s': raw_pts=%u stamp=%.6f (throttle 1s)",
                    lid_topic.c_str(), msg->width * msg->height, msg->header.stamp.toSec());
  if (!lidar_en) { ROS_WARN_THROTTLE(2.0, "[cbk][lidar] lidar_en=false, dropping cloud"); return; }
  mtx_buffer.lock();

  double cur_head_time = msg->header.stamp.toSec() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  // [DEBUG] 打印预处理(robosense_handler 等)解析后的点数 + buffer 大小；解析为 0 说明
  // lidar_type/点云字段不匹配（本次数据是裸 XYZI，无 ring/timestamp）
  ROS_INFO_THROTTLE(1.0, "[cbk][lidar] after preprocess: parsed_pts=%zu, lidar_buffer=%zu (throttle 1s)",
                    (ptr ? ptr->size() : 0), lid_raw_data_buffer.size() + 1);
  if (!ptr || ptr->empty())
    ROS_WARN_THROTTLE(1.0, "[cbk][lidar] preprocess produced EMPTY cloud (检查 lidar_type=%d 与点云字段是否匹配)",
                      p_pre->lidar_type);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in)
{
  ROS_INFO_THROTTLE(1.0, "[cbk][lidar-livox] recv CustomMsg on '%s': pts=%u stamp=%.6f (throttle 1s)",
                    lid_topic.c_str(), (uint32_t)msg_in->point_num, msg_in->header.stamp.toSec());
  if (!lidar_en) { ROS_WARN_THROTTLE(2.0, "[cbk][lidar-livox] lidar_en=false, dropping cloud"); return; }
  mtx_buffer.lock();
  livox_ros_driver::CustomMsg::Ptr msg(new livox_ros_driver::CustomMsg(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("lidar jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_lidar + 0.1);
  // }
  if (abs(last_timestamp_imu - msg->header.stamp.toSec()) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg->header.stamp.toSec();
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg->header.stamp.toSec();
  //ROS_INFO("Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    ROS_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}


void LIVMapper::rtk_cbk(const gnss_comm::GnssPVTSolnMsg::ConstPtr& gpsMsg)
{
    ROS_INFO("RTK received");
    Eigen::Vector3d trans_local_;
    static bool first_gps = false;
    if (!first_gps) {
        first_gps = true;
        gps_trans_.Reset(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        std::ofstream gps_file_header(rtk_save_file, std::ios::out); 
        if (gps_file_header.is_open()) {
            gps_file_header << "# timestamp x y z vx vy vz h_acc v_acc" << std::endl;
            gps_file_header.close();
        }
    }

    const double GPS_EPOCH_UNIX_TIME = 315964800.0;
    const double SECONDS_PER_WEEK = 604800.0; 
    const double LEAP_SECONDS = 18.0;
    int week = gpsMsg->time.week;
    double tow = gpsMsg->time.tow; 
    double total_gps_seconds = (double)week * SECONDS_PER_WEEK + tow;
    double timestamp_sec = total_gps_seconds + GPS_EPOCH_UNIX_TIME - LEAP_SECONDS;
    ros::Time stamp;
    stamp.fromSec(timestamp_sec);

    gps_trans_.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, trans_local_[0], trans_local_[1], trans_local_[2]);

    RTK rtk_data;
    rtk_data.timestamp = stamp.toSec() - 0.001;
    rtk_data.p[0] = trans_local_[0];
    rtk_data.p[1] = trans_local_[1];
    rtk_data.p[2] = trans_local_[2];
    rtk_data.v[0] = gpsMsg->vel_e;
    rtk_data.v[1] = gpsMsg->vel_n;
    rtk_data.v[2] = -gpsMsg->vel_d;

    // gps/rtk_robust_recovery_en：与上一条 GNSS PVT 的间隔超过 rtk_gap_thresh_ 视为断流恢复，
    // 标记后交给 GetRTKCorrectedState/handleRTK/handleLIO 做特殊处理（见各处调用点注释）。
    static double last_recv_ts = -1.0;
    if (rtk_robust_recovery_en_ && last_recv_ts >= 0.0 && rtk_data.timestamp - last_recv_ts > rtk_gap_thresh_)
    {
      rtk_data.is_resume = true;
      ROS_WARN("[RTK] gap detected: %.3fs (> %.3fs), mark next sample as resume", rtk_data.timestamp - last_recv_ts, rtk_gap_thresh_);
    }
    last_recv_ts = rtk_data.timestamp;
    
    rtk_buffer.push_back(rtk_data);
    ROS_INFO("rtk_data_pushed: %.6f, %.3f, %.3f, %.3f", rtk_data.timestamp, rtk_data.p[0], rtk_data.p[1], rtk_data.p[2]);


    if(trans_local_.norm() > 50.0 && rtk_ini == false && rtk_en == true)
    {
      ROS_INFO("Start INIT RTK to LIVO TRANSFORMATION");
      InitializeRTK();
      ROS_INFO("RTK to LIVO TRANSFORMATION INITIALIZED");
      rtk_ini = true;
    }

    // if(rtk_ini)
    // {
    //trans_local_ = T_G_to_W * Eigen::Vector3d(trans_local_[0], trans_local_[1], trans_local_[2]);
    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = stamp; 
    gps_odom.header.frame_id = "camera_init";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
    gps_odom.twist.twist.linear.x = gpsMsg->vel_e;
    gps_odom.twist.twist.linear.y = gpsMsg->vel_n;
    gps_odom.twist.twist.linear.z = -gpsMsg->vel_d;
    gps_odom.pose.covariance[0] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[7] = gpsMsg->h_acc; 
    gps_odom.pose.covariance[14] = gpsMsg->v_acc; 

    pub_rtk.publish(gps_odom);
    
    if(debug_mode)
    {
      std::ofstream gps_file(rtk_save_file, std::ios::app); 
      if (gps_file.is_open()) {
          gps_file << std::fixed << std::setprecision(6) 
                  << gps_odom.header.stamp.toSec() << " "  // 1. Timestamp
                  << trans_local_[0] << " "                // 2. x
                  << trans_local_[1] << " "                // 3. y
                  << trans_local_[2] << " "                // 4. z
                  << gpsMsg->vel_e << " "                  // 5. vx (East)
                  << gpsMsg->vel_n << " "                  // 6. vy (North)
                  << -gpsMsg->vel_d << " "                 // 7. vz (Up, 注意取反)
                  << gpsMsg->h_acc << " "                  // 8. h_acc
                  << gpsMsg->v_acc                         // 9. v_acc
                  << std::endl;
          gps_file.close();
      }
    // }
  }
}

// 用 /localization/odom (nav_msgs/Odometry) 当作 RTK 观测填 rtk_buffer。
// 与 rtk_cbk 的差别：
//   1) 里程计已经是笛卡尔位姿（UTM/局部地图系），不需要 LLA->ENU，所以不用 gps_trans_；
//   2) 但要减掉首帧位置当原点，保持和 rtk_cbk 一样的"相对首帧"语义——
//      InitializeRTK 的 norm()>50 触发条件和 SVD 的数值稳定性都依赖坐标是小数值；
//   3) 时间戳直接用 header.stamp（同样减 1ms，与 rtk_cbk 对齐 sync_packages 的取帧区间）。
void LIVMapper::localization_odom_cbk(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  // ROS_INFO("RTK(odom) received (throttle 1s). timestamp %.6f ", odomMsg->header.stamp.toSec());
  // if (odomMsg->header.stamp.toSec() > (1785289545.0 + 30) and odomMsg->header.stamp.toSec() < (1785289545.0 + 60))
  // {
  //   ROS_ERROR("RTK(odom) NOT used: timestamp %.6f in [%.6f, %.6f]", odomMsg->header.stamp.toSec(),  1785289545.0 + 30, 1785289545.0 + 60);
  //   return;
  // }
  // ROS_INFO("RTK(odom) received (throttle 1s). timestamp %.6f ", odomMsg->header.stamp.toSec());

    static bool first_odom = false;
    static Eigen::Vector3d odom_origin = Eigen::Vector3d::Zero();

    const Eigen::Vector3d p_abs(odomMsg->pose.pose.position.x,
                                odomMsg->pose.pose.position.y,
                                odomMsg->pose.pose.position.z);
    if (!first_odom) {
        first_odom = true;
        odom_origin = p_abs;
        ROS_INFO("[RTK(odom)] origin set to [%.3f, %.3f, %.3f]", odom_origin[0], odom_origin[1], odom_origin[2]);
        std::ofstream gps_file_header(rtk_save_file, std::ios::out);
        if (gps_file_header.is_open()) {
            gps_file_header << "# timestamp x y z vx vy vz h_acc v_acc" << std::endl;
            gps_file_header.close();
        }
    }

    const Eigen::Vector3d trans_local_ = p_abs - odom_origin;
    // ROS_INFO("[RTK(odom)] trans_local_ set to [%.3f, %.3f, %.3f]", trans_local_[0], trans_local_[1], trans_local_[2]);

    ros::Time stamp = odomMsg->header.stamp;

    // 速度：不用 twist。ROS 约定 nav_msgs/Odometry 的 twist 在 child_frame_id(车体系)下，
    // 而 GetRTKCorrectedState 把 rtk.v 当成 UTM 系速度（只乘 T_G_to_W 的旋转）后**硬赋给
    // state.vel_end**，坐标系一错就是往状态里注入一个方向完全错误的速度。
    // 这里直接用相邻两条 odom 的位置差分，语义上确定就是 UTM 系速度。
    // 注意：不能简单留空——RTK::v 是 Eigen 向量，默认构造是未初始化内存，
    // 硬赋给 vel_end 就是随机速度，比给 0 更糟。
    static bool has_prev = false;
    static double prev_ts = 0.0;
    static Eigen::Vector3d prev_p_abs = Eigen::Vector3d::Zero();
    static Eigen::Vector3d last_vel = Eigen::Vector3d::Zero();

    const double ts = stamp.toSec();
    // gps/rtk_robust_recovery_en：与上一条 odom 的间隔超过 rtk_gap_thresh_ 视为断流恢复。
    // 注意这条刚恢复的样本 last_vel 仍是断流前算出的陈旧值(见上面 dt<0.5 的判断)，
    // 标记 is_resume 后 GetRTKCorrectedState 会整体跳过硬複位，陈旧速度也就不会被用上。
    bool is_gap_resume = false;
    if (has_prev)
    {
      const double dt = ts - prev_ts;
      if (rtk_robust_recovery_en_ && dt > rtk_gap_thresh_)
      {
        is_gap_resume = true;
        ROS_WARN("[RTK(odom)] gap detected: %.3fs (> %.3fs), mark this sample as resume", dt, rtk_gap_thresh_);
      }
      if (dt > 1e-3 && dt < 0.5) last_vel = (p_abs - prev_p_abs) / dt;  // dt 异常(重复戳/大跳变)时沿用上一次
    }
    has_prev = true;
    prev_ts = ts;
    prev_p_abs = p_abs;

    RTK rtk_data;
    rtk_data.timestamp = ts; // - 0.001;
    rtk_data.is_resume = is_gap_resume;
    rtk_data.p[0] = trans_local_[0];
    rtk_data.p[1] = trans_local_[1];
    rtk_data.p[2] = trans_local_[2];
    rtk_data.v = last_vel;

    rtk_buffer.push_back(rtk_data);
    ROS_INFO_THROTTLE(1.0, "rtk_data_pushed(odom): t=%.6f p=[%.3f, %.3f, %.3f] v=[%.2f, %.2f, %.2f] |v|=%.2f (throttle 1s)",
                      rtk_data.timestamp, rtk_data.p[0], rtk_data.p[1], rtk_data.p[2],
                      rtk_data.v[0], rtk_data.v[1], rtk_data.v[2], rtk_data.v.norm());

    if(trans_local_.norm() > 50.0 && rtk_ini == false && rtk_en == true)
    {
      ROS_INFO("Start INIT RTK to LIVO TRANSFORMATION");
      InitializeRTK();
      ROS_INFO("RTK to LIVO TRANSFORMATION INITIALIZED");
      rtk_ini = true;
    }

    // 与 rtk_cbk 一致：把折算到首帧原点的位置作为 /gps/odometry 发出去，便于 rviz 对比
    nav_msgs::Odometry gps_odom;
    gps_odom.header.stamp = stamp;
    gps_odom.header.frame_id = "camera_init";
    gps_odom.pose.pose.position.x = trans_local_[0];
    gps_odom.pose.pose.position.y = trans_local_[1];
    gps_odom.pose.pose.position.z = trans_local_[2];
    gps_odom.pose.pose.orientation = odomMsg->pose.pose.orientation;
    gps_odom.twist = odomMsg->twist;
    // 位置协方差沿用上游 odom 的 xx/yy/zz（没填就是 0）
    gps_odom.pose.covariance[0]  = odomMsg->pose.covariance[0];
    gps_odom.pose.covariance[7]  = odomMsg->pose.covariance[7];
    gps_odom.pose.covariance[14] = odomMsg->pose.covariance[14];

    pub_rtk.publish(gps_odom);

    if(debug_mode)
    {
      std::ofstream gps_file(rtk_save_file, std::ios::app);
      if (gps_file.is_open()) {
          gps_file << std::fixed << std::setprecision(6)
                  << gps_odom.header.stamp.toSec() << " "        // 1. timestamp
                  << trans_local_[0] << " "                      // 2. x
                  << trans_local_[1] << " "                      // 3. y
                  << trans_local_[2] << " "                      // 4. z
                  << rtk_data.v[0] << " "                        // 5. vx
                  << rtk_data.v[1] << " "                        // 6. vy
                  << rtk_data.v[2] << " "                        // 7. vz
                  << std::sqrt(std::max(0.0, odomMsg->pose.covariance[0])) << " "   // 8. h_acc(≈std)
                  << std::sqrt(std::max(0.0, odomMsg->pose.covariance[14]))         // 9. v_acc(≈std)
                  << std::endl;
          gps_file.close();
      }
    }
}

Sophus::SE3 computeSVD(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& target,
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& source)
{
    if (target.empty() || target.size() != source.size()) {
        return Sophus::SE3(); 
    }

    Eigen::Vector3d target_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d source_center = Eigen::Vector3d::Zero();
    for (const auto& p : target) target_center += p;
    for (const auto& p : source) source_center += p;
    target_center /= target.size();
    source_center /= source.size();

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < source.size(); ++i) {
        W += (target[i] - target_center) * (source[i] - source_center).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    if (R.determinant() < 0) { 
        R = svd.matrixU() * Eigen::DiagonalMatrix<double, 3>(1, 1, -1) * svd.matrixV().transpose();
    }

    Eigen::Vector3d t = target_center - R * source_center;
    
    return Sophus::SE3(Sophus::SO3(R), t);
}

void LIVMapper::InitializeRTK()
{
    const double time_sync_threshold = 0.05;   

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_source_livo; // Source
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_target_rtk;  // Target
    auto livo_it = livo_state_buffer.begin();

    for (const auto& rtk_data : rtk_buffer)
    {
        double t_rtk = rtk_data.timestamp;
        while (livo_it != livo_state_buffer.end())
        {
            double t_livo = (*livo_it)[0]; 
            if (t_livo < t_rtk - time_sync_threshold) {
                livo_it++;
            } else {
                break;
            }
        }

        if (livo_it == livo_state_buffer.end()) break;

        auto best_it = livo_it;
        double min_dt = std::abs((*best_it)[0] - t_rtk);

        auto next_it = std::next(livo_it);
        if (next_it != livo_state_buffer.end())
        {
            double dt_next = std::abs((*next_it)[0] - t_rtk);
            if (dt_next < min_dt)
            {
                min_dt = dt_next;
                best_it = next_it;
            }
        }

        if (min_dt < time_sync_threshold)
        {
          if (best_it->size() < 4) {
                ROS_ERROR("LIVO buffer data size is too small (%lu < 4)! Check data collection.", best_it->size());
            }
            pts_target_rtk.push_back(rtk_data.p);
            Eigen::Vector3d p_livo((*best_it)[1], (*best_it)[2], (*best_it)[3]);
            pts_source_livo.push_back(p_livo);
        }
    }

    if (pts_target_rtk.size() < 3) {
        ROS_WARN("Not enough matched points for RTK initialization (found %lu). Waiting for more data...", pts_target_rtk.size());
    }
    ROS_INFO("pts_target_rtk size: %lu, pts_source_livo size: %lu", pts_target_rtk.size(), pts_source_livo.size());
    T_W_to_G = computeSVD(pts_target_rtk, pts_source_livo);
    T_G_to_W = T_W_to_G.inverse();
}

void LIVMapper::imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  // [DEBUG] 收到 IMU 就打印（在任何 early-return 之前），确认话题对不对
  ROS_INFO_THROTTLE(1.0, "[cbk][imu] recv Imu on '%s': stamp=%.6f (throttle 1s)",
                    imu_topic.c_str(), msg_in->header.stamp.toSec());
  if (!imu_en) { ROS_WARN_THROTTLE(2.0, "[cbk][imu] imu_en=false, dropping"); return; }

  // 注意：first lidar 到达之前，IMU 会被这里直接丢弃（下游 sync 依赖 last_timestamp_lidar）
  if (last_timestamp_lidar < 0.0) {
    ROS_WARN_THROTTLE(1.0, "[cbk][imu] last_timestamp_lidar<0（还没收到/解析到 LiDAR），IMU 暂被丢弃");
    return;
  }
  // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - imu_time_offset);
  double timestamp = msg->header.stamp.toSec();

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    ROS_WARN("IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = ros::Time().fromSec(timestamp);

  static bool first_imu_save = true;

  if (first_imu_save) {
      std::ofstream imu_file(imu_save_file, std::ios::out);
      if (imu_file.is_open()) {
          imu_file << "# timestamp ax ay az gx gy gz" << std::endl;
          imu_file.close();
      }
      first_imu_save = false;
  }

  if(debug_mode)
  {
    std::ofstream imu_file(imu_save_file, std::ios::app);
    if (imu_file.is_open()) {
        imu_file << std::fixed << std::setprecision(6)
                << timestamp << " "                     // 时间戳
                << msg->linear_acceleration.x << " "    // ax
                << msg->linear_acceleration.y << " "    // ay
                << msg->linear_acceleration.z << " "    // az
                << msg->angular_velocity.x << " "       // gx
                << msg->angular_velocity.y << " "       // gy
                << msg->angular_velocity.z << std::endl;// gz
        imu_file.close();
    }
  }

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    ROS_ERROR("imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   ROS_WARN("imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat img;
  img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  // 整幅去畸变（在任何下游使用之前）：仅当映射已按该分辨率构建、且图像尺寸匹配时执行。
  // 去畸变后图像为 rectified，配合零畸变的针孔相机模型使用（见 loadCalibrationFromFiles）。
  if (undistort_ready_)
  {
    if (img.cols == calib_img_width_ && img.rows == calib_img_height_)
    {
      cv::Mat rectified;
      cv::remap(img, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
      img = rectified;
    }
    else
    {
      ROS_WARN_THROTTLE(2.0,
          "[cbk][img] 图像尺寸 %dx%d 与标定分辨率 %dx%d 不一致，跳过去畸变（检查内参 ImageWidth/Height 是否匹配数据）",
          img.cols, img.rows, calib_img_width_, calib_img_height_);
    }
  }
  return img;
}

void LIVMapper::img_cbk(const sensor_msgs::ImageConstPtr &msg_in)
{
  // [DEBUG] 收到图像就打印（在任何 early-return 之前），确认话题对不对
  ROS_INFO_THROTTLE(1.0, "[cbk][img] recv Image on '%s': %dx%d stamp=%.6f (throttle 1s)",
                    img_topic.c_str(), msg_in->width, msg_in->height, msg_in->header.stamp.toSec());
  if (!img_en) { ROS_WARN_THROTTLE(2.0, "[cbk][img] img_en=false, dropping"); return; }
  sensor_msgs::Image::Ptr msg(new sensor_msgs::Image(*msg_in));
  // if ((abs(msg->header.stamp.toSec() - last_timestamp_img) > 0.2 && last_timestamp_img > 0) || sync_jump_flag)
  // {
  //   ROS_WARN("img jumps %.3f\n", msg->header.stamp.toSec() - last_timestamp_img);
  //   sync_jump_flag = true;
  //   msg->header.stamp = ros::Time().fromSec(last_timestamp_img + 0.1);
  // }

  // Hiliti2022 40Hz
  // if (hilti_en)
  if (1)
  {
    static int frame_counter = 0;
    if (++frame_counter % 3 != 0) return;
  }
  // double msg_header_time =  msg->header.stamp.toSec();
  double msg_header_time = msg->header.stamp.toSec() + img_time_offset;
  if (abs(msg_header_time - last_timestamp_img) < 0.001) return;
  //ROS_INFO("Get image, its header time: %.6f", msg_header_time);
  if (last_timestamp_lidar < 0) {
    ROS_WARN_THROTTLE(1.0, "[cbk][img] last_timestamp_lidar<0（还没收到/解析到 LiDAR），图像暂被丢弃");
    return;
  }

  if (msg_header_time < last_timestamp_img)
  {
    ROS_ERROR("image loop back. \n");
    return;
  }

  mtx_buffer.lock();

  double img_time_correct = msg_header_time; // last_timestamp_lidar + 0.105;

  if (img_time_correct - last_timestamp_img < 0.02)
  {
    ROS_WARN("Image need Jumps: %.6f", img_time_correct);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    return;
  }

  cv::Mat img_cur = getImageFromMsg(msg);
  img_buffer.push_back(img_cur);
  img_time_buffer.push_back(img_time_correct);

  // ROS_INFO("Correct Image time: %.6f", img_time_correct);

  last_timestamp_img = img_time_correct;
  // cv::imshow("img", img);
  // cv::waitKey(1);
  // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (imu_buffer.front()->header.stamp.toSec() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    if(!img_en)keyframe_time = meas.lidar_frame_end_time;
    return true;

    break;
  }

  case LIVO:
  {
    /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
     * first than VIO imediatly ***/
    EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
    // double t0 = omp_get_wtime();
    switch (last_lio_vio_flg)
    {
    // double img_capture_time = meas.lidar_frame_beg_time + exposure_time_init;
    case WAIT:
    case VIO:
    {
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      if(img_en)keyframe_time = img_capture_time;
      /*** has img topic, but img topic timestamp larger than lidar end time,
       * process lidar topic. After LIO update, the meas.lidar_frame_end_time
       * will be refresh. ***/
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
      // printf("[ Data Cut ] wait \n");
      // printf("[ Data Cut ] last_lio_update_time: %lf \n",
      // meas.last_lio_update_time);

      double lid_newest_time = lid_header_time_buffer.back() + lid_raw_data_buffer.back()->points.back().curvature / double(1000);
      double imu_newest_time = imu_buffer.back()->header.stamp.toSec();

      if (img_capture_time < meas.last_lio_update_time + 0.00001)
      {
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        ROS_ERROR("[ Data Cut ] Throw one image frame! \n");
        return false;
      }

      if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time)
      {
        // ROS_ERROR("lost first camera frame");
        // printf("img_capture_time, lid_newest_time, imu_newest_time: %lf , %lf
        // , %lf \n", img_capture_time, lid_newest_time, imu_newest_time);
        return false;
      }

      struct MeasureGroup m;

      // printf("[ Data Cut ] LIO \n");
      // printf("[ Data Cut ] img_capture_time: %lf \n", img_capture_time);
      m.imu.clear();
      m.rtk.timestamp = 0.0;
      m.rtk.p = Eigen::Vector3d::Zero();
      m.lio_time = img_capture_time;
      mtx_buffer.lock();

      if (rtk_en && rtk_ini)
      {
          while (!rtk_buffer.empty())
          {
            ROS_INFO("[SyncRTK] RTK_buffer front time: %.6f, last_lio_update_time: %.6f, lio_time: %.6f", rtk_buffer.front().timestamp, meas.last_lio_update_time, m.lio_time);
            if (rtk_buffer.front().timestamp > m.lio_time) break;
            if (rtk_buffer.front().timestamp > meas.last_lio_update_time) 
            {
                m.rtk = rtk_buffer.front();
                break;
            }
            rtk_buffer.pop_front();
          }
          ROS_INFO("[SyncRTK] RTK.pushed to MeasureGroup: %.6f, %.3f, %.3f, %.3f", m.rtk.timestamp, m.rtk.p[0], m.rtk.p[1], m.rtk.p[2]);
      }
      
      while (!imu_buffer.empty())
      {
        if (imu_buffer.front()->header.stamp.toSec() > m.lio_time) break;

        if (imu_buffer.front()->header.stamp.toSec() > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

        imu_buffer.pop_front();
        // printf("[ Data Cut ] imu time: %lf \n",
        // imu_buffer.front()->header.stamp.toSec());
      }
      mtx_buffer.unlock();
      sig_buffer.notify_all();

      *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
      PointCloudXYZI().swap(*meas.pcl_proc_next);

      int lid_frame_num = lid_raw_data_buffer.size();
      int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
      meas.pcl_proc_cur->reserve(max_size);
      meas.pcl_proc_next->reserve(max_size);
      // deque<PointCloudXYZI::Ptr> lidar_buffer_tmp;

      while (!lid_raw_data_buffer.empty())
      {
        if (lid_header_time_buffer.front() > img_capture_time) break;
        auto pcl(lid_raw_data_buffer.front()->points);
        double frame_header_time(lid_header_time_buffer.front());
        float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

        for (int i = 0; i < pcl.size(); i++)
        {
          auto pt = pcl[i];
          if (pcl[i].curvature < max_offs_time_ms)
          {
            pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
            meas.pcl_proc_cur->points.push_back(pt);
          }
          else
          {
            pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
            meas.pcl_proc_next->points.push_back(pt);
          }
        }
        lid_raw_data_buffer.pop_front();
        lid_header_time_buffer.pop_front();
      }

      meas.measures.push_back(m);
      meas.lio_vio_flg = LIO;
      // meas.last_lio_update_time = m.lio_time;
      // printf("!!! meas.lio_vio_flg: %d \n", meas.lio_vio_flg);
      // printf("[ Data Cut ] pcl_proc_cur number: %d \n", meas.pcl_proc_cur
      // ->points.size()); printf("[ Data Cut ] LIO process time: %lf \n",
      // omp_get_wtime() - t0);
      return true;
    }

    case LIO:
    {
      double img_capture_time = img_time_buffer.front() + exposure_time_init;
      meas.lio_vio_flg = VIO;
      // printf("[ Data Cut ] VIO \n");
      meas.measures.clear();
      double imu_time = imu_buffer.front()->header.stamp.toSec();

      struct MeasureGroup m;
      m.vio_time = img_capture_time;
      m.lio_time = meas.last_lio_update_time;
      m.img = img_buffer.front();
      mtx_buffer.lock();
      
      // while ((!imu_buffer.empty() && (imu_time < img_capture_time)))
      // {
      //   imu_time = imu_buffer.front()->header.stamp.toSec();
      //   if (imu_time > img_capture_time) break;
      //   m.imu.push_back(imu_buffer.front());
      //   imu_buffer.pop_front();
      //   printf("[ Data Cut ] imu time: %lf \n",
      //   imu_buffer.front()->header.stamp.toSec());
      // }

      img_buffer.pop_front();
      img_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.measures.push_back(m);
      lidar_pushed = false; // after VIO update, the _lidar_frame_end_time will be refresh.
      // printf("[ Data Cut ] VIO process time: %lf \n", omp_get_wtime() - t0);
      return true;
    }

    default:
    {
      // printf("!! WRONG EKF STATE !!");
      return false;
    }
      // return false;
    }
    break;
  }

  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  ROS_ERROR("out sync");
}

void LIVMapper::publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager)
{
  cv::Mat img_rgb = vio_manager->img_cp;
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  // out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;
  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager)
{
  ROS_INFO("Publish frame world pointcloud.");
  if (pcl_w_wait_pub->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB());
  if (img_en)
  {
    static int pub_num = 1;
    *pcl_wait_pub += *pcl_w_wait_pub;
    if(pub_num == pub_scan_num)
    {
      pub_num = 1;
      size_t size = pcl_wait_pub->points.size();
      laserCloudWorldRGB->reserve(size);
      // double inv_expo = _state.inv_expo_time;
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; i++)
      {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub->points[i].x;
        pointRGB.y = pcl_wait_pub->points[i].y;
        pointRGB.z = pcl_wait_pub->points[i].z;

        V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
        V3D pf(vio_manager->new_frame_->w2f(p_w)); if (pf[2] < 0) continue;
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) // 100
        {
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];
          // pointRGB.r = pixel[2] * inv_expo; pointRGB.g = pixel[1] * inv_expo; pointRGB.b = pixel[0] * inv_expo;
          // if (pointRGB.r > 255) pointRGB.r = 255;
          // else if (pointRGB.r < 0) pointRGB.r = 0;
          // if (pointRGB.g > 255) pointRGB.g = 255;
          // else if (pointRGB.g < 0) pointRGB.g = 0;
          // if (pointRGB.b > 255) pointRGB.b = 255;
          // else if (pointRGB.b < 0) pointRGB.b = 0;
          if (pf.norm() > blind_rgb_points) laserCloudWorldRGB->push_back(pointRGB);
        }
      }
    }
    else
    {
      pub_num++;
    }
  }

  /*** Publish Frame ***/
  sensor_msgs::PointCloud2 laserCloudmsg;
  if (img_en)
  {
    // cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  }
  else 
  { 
    pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  }
  laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);

  if(img_en)
  {
    PointCloudXYZRGB::Ptr laserCloudBodyRGB(new PointCloudXYZRGB());
    laserCloudBodyRGB->reserve(laserCloudWorldRGB->size());
    for (const auto& pt_world_rgb : laserCloudWorldRGB->points)
    {
          V3D p_global(pt_world_rgb.x, pt_world_rgb.y, pt_world_rgb.z);
          V3D p_body = _state.rot_end.transpose() * (p_global - _state.pos_end);
          PointTypeRGB pointBodyRGB;
          pointBodyRGB.x = p_body(0);
          pointBodyRGB.y = p_body(1);
          pointBodyRGB.z = p_body(2);
          pointBodyRGB.r = pt_world_rgb.r;
          pointBodyRGB.g = pt_world_rgb.g;
          pointBodyRGB.b = pt_world_rgb.b;
          laserCloudBodyRGB->push_back(pointBodyRGB);
    }
    sensor_msgs::PointCloud2 laserCloudbodymsg;
    pcl::toROSMsg(*laserCloudBodyRGB, laserCloudbodymsg);
    laserCloudbodymsg.header.stamp = ros::Time(keyframe_time);
    laserCloudbodymsg.header.frame_id = "camera_init";
    pub_lidarRGB.publish(laserCloudbodymsg);

    if(debug_mode)
    {
      if (laserCloudBodyRGB->size() > 0) 
      {
          std::stringstream ss;
          ss << std::setfill('0') << std::setw(6) << pcd_file_index;
          std::string pcd_filename = pcd_save_file + ss.str() + ".pcd";    
          try { pcl::io::savePCDFileBinary(pcd_filename, *laserCloudBodyRGB); pcd_file_index++; }
          catch (const std::exception &e)
          { ROS_WARN_THROTTLE(5.0, "[debug pcd] write failed: %s (%s)", pcd_filename.c_str(), e.what()); }
      }
    }
  }
  
  if(!img_en)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudBody(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudBody->reserve(pcl_w_wait_pub->size());
    for (const auto& pt_world : pcl_w_wait_pub->points)
    {
          V3D p_global(pt_world.x, pt_world.y, pt_world.z);
          V3D p_body = _state.rot_end.transpose() * (p_global - _state.pos_end);
          pcl::PointXYZI pointBody;
          pointBody.x = p_body(0);
          pointBody.y = p_body(1);
          pointBody.z = p_body(2);
          laserCloudBody->push_back(pointBody);
    }
      sensor_msgs::PointCloud2 laserCloudbodymsg;
      pcl::toROSMsg(*laserCloudBody, laserCloudbodymsg);
      laserCloudbodymsg.header.stamp = ros::Time(keyframe_time);
      laserCloudbodymsg.header.frame_id = "camera_init";
      pub_lidarRGB.publish(laserCloudbodymsg);

      if(debug_mode)
      {
        if (laserCloudBody->size() > 0) 
        {
          std::stringstream ss;
          ss << std::setfill('0') << std::setw(6) << pcd_file_index;
          std::string pcd_filename = pcd_save_file + ss.str() + ".pcd";
          try { pcl::io::savePCDFileBinary(pcd_filename, *laserCloudBody); pcd_file_index++; }
          catch (const std::exception &e)
          { ROS_WARN_THROTTLE(5.0, "[debug pcd] write failed: %s (%s)", pcd_filename.c_str(), e.what()); }
        }
      }
  }

  
  std::vector<double> livo_xyz(4);
  livo_xyz[0] = keyframe_time;
  livo_xyz[1] = _state.pos_end(0);
  livo_xyz[2] = _state.pos_end(1);
  livo_xyz[3] = _state.pos_end(2);
  livo_state_buffer.push_back(livo_xyz);


  nav_msgs::Odometry odom_msg_ekf;
  odom_msg_ekf.header.stamp = ros::Time(keyframe_time);
  odom_msg_ekf.header.frame_id = "camera_init";
  odom_msg_ekf.pose.pose.position.x = _state.pos_end(0);
  odom_msg_ekf.pose.pose.position.y = _state.pos_end(1);
  odom_msg_ekf.pose.pose.position.z = _state.pos_end(2);
  Eigen::Quaterniond q(_state.rot_end);
  odom_msg_ekf.pose.pose.orientation.w = q.w();
  odom_msg_ekf.pose.pose.orientation.x = q.x();
  odom_msg_ekf.pose.pose.orientation.y = q.y();
  odom_msg_ekf.pose.pose.orientation.z = q.z();
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> pose_cov_map(odom_msg_ekf.pose.covariance.data());

  pose_cov_map.block<3, 3>(0, 0) = _state.cov.block<3, 3>(3, 3);
  pose_cov_map.block<3, 3>(3, 3) = _state.cov.block<3, 3>(0, 0);
  pose_cov_map.block<3, 3>(0, 3) = _state.cov.block<3, 3>(3, 0);
  pose_cov_map.block<3, 3>(3, 0) = _state.cov.block<3, 3>(0, 3);
  pub_odom.publish(odom_msg_ekf); 

  if(debug_mode)
  {
    std::ofstream odom_file(odom_save_file, std::ios::app); 
    if (odom_file.is_open()) {
        odom_file << std::fixed << std::setprecision(6) 
                  << keyframe_time << " "
                  << _state.pos_end(0) << " "
                  << _state.pos_end(1) << " "
                  << _state.pos_end(2) << " "
                  << q.x() << " "
                  << q.y() << " "
                  << q.z() << " "
                  << q.w() << std::endl;
        odom_file.close();
    }

    std::ofstream cov_file(cov_save_file, std::ios::app);
    if (cov_file.is_open()) {
        cov_file << std::fixed << std::setprecision(6) << keyframe_time;
        
        for (int i = 0; i < 36; ++i) {
            cov_file << " " << odom_msg_ekf.pose.covariance[i];
        }
        cov_file << std::endl;
        cov_file.close();
    }
  }
  
  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
    static int scan_wait_num = 0;

    if (img_en)
    {
      *pcl_wait_save += *laserCloudWorldRGB;
    }
    else
    {
      *pcl_wait_save_intensity += *pcl_w_wait_pub;
    }
    scan_wait_num++;

    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      pcd_index++;
      string all_points_dir(string(string(ROOT_DIR) + "Log/PCD/") + to_string(pcd_index) + string(".pcd"));
      pcl::PCDWriter pcd_writer;
      if (pcd_save_en)
      {
        cout << "current scan saved to /PCD/" << all_points_dir << endl;
        if (img_en)
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save); // pcl::io::savePCDFileASCII(all_points_dir, *pcl_wait_save);
          PointCloudXYZRGB().swap(*pcl_wait_save);
        }
        else
        {
          pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
          PointCloudXYZI().swap(*pcl_wait_save_intensity);
        }        
        Eigen::Quaterniond q(_state.rot_end);
        fout_pcd_pos << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.w() << " " << q.x() << " " << q.y()
                     << " " << q.z() << " " << endl;
        scan_wait_num = 0;
      }
    }
  }
  if(laserCloudWorldRGB->size() > 0)  PointCloudXYZI().swap(*pcl_wait_pub); 
  PointCloudXYZI().swap(*pcl_w_wait_pub);
}

void LIVMapper::publish_visual_sub_map(const ros::Publisher &pubSubVisualMap)
{
  PointCloudXYZI::Ptr laserCloudFullRes(visual_sub_map);
  int size = laserCloudFullRes->points.size(); if (size == 0) return;
  PointCloudXYZI::Ptr sub_pcl_visual_map_pub(new PointCloudXYZI());
  *sub_pcl_visual_map_pub = *laserCloudFullRes;
  if (1)
  {
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*sub_pcl_visual_map_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time::now();
    laserCloudmsg.header.frame_id = "camera_init";
    pubSubVisualMap.publish(laserCloudmsg);
  }
}

void LIVMapper::publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = ros::Time::now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect.publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(const ros::Publisher &pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(_state.pos_end(0), _state.pos_end(1), _state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform( tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped") );
  pubOdomAftMapped.publish(odomAftMapped);
}

void LIVMapper::publish_mavros(const ros::Publisher &mavros_pose_publisher)
{
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void LIVMapper::publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}