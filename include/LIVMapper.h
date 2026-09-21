/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include "sensor_extrinsics.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/pinhole_camera.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <GeographicLib/LocalCartesian.hpp>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleVIO();
  void handleLIO();
  void handleRTK();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback(const ros::TimerEvent &e);
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
 
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  void rtk_cbk(const gnss_comm::GnssPVTSolnMsg::ConstPtr& gpsMsg);
  // 用 /localization/odom (nav_msgs/Odometry) 顶替 GNSS PVT 填充 rtk_buffer
  void localization_odom_cbk(const nav_msgs::Odometry::ConstPtr& odomMsg);
  void InitializeRTK();
  // gps/rtk_robust_recovery_en 打开时，handleRTK / handleLIO 用它对 RTK 位置新息做门限校验：
  // 残差在 rtk_innov_gate_ 倍 sigma(当前位置协方差 + 观测噪声) 以内才认为可信。
  bool rtkInnovationOk(const Eigen::Vector3d &rtk_pos, const Eigen::Vector3d &meas_std);
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  void readParameters(ros::NodeHandle &nh);
  bool loadCalibrationFromFiles();
  // 导出为 Global-LVBA keyframe 数据集（供其做二次 LiDAR-Visual BA）
  void saveLvbaKeyframeLidar();                        // handleLIO 调用：存激光系点云 + odom(LIO 位姿)
  void saveLvbaKeyframeImageOdom(const cv::Mat &img);  // handleVIO 调用：存去畸变图 + 覆写 odom(VIO 位姿)
  bool writeLvbaOdom(const std::string &stem, const StatesGroup &st, double ts);  // 写 odoms/{stem}.yaml (T_utm_ins)
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;
  vector<double> extrinR;
  vector<double> cameraextrinT;
  vector<double> cameraextrinR;
  double IMG_POINT_COV;

  // 内外参标定文件（格式/目录结构与 Global-LVBA 的 keyframe 模式一致：
  // calib_path_/intrinsics/<camera_frame_>_Intrinsic.yaml
  // calib_path_/extrinsics/{id}_{sensor}_2_{parent}_Extrinsics.yaml）
  std::string calib_path_;
  std::string lidar_frame_;
  std::string vehicle_frame_;
  std::string imu_frame_;      // IMU(ins) 坐标系名——lidar->IMU 外参要查到这个系，不是 vehicle
  std::string camera_frame_;
  double cam_scale_ = 1.0;

  // 图像整幅去畸变：本相机是 8 参数 rational 畸变模型，vk::PinholeCamera 的解析畸变
  // (最多5参数) 表达不了，故在图像入口用 cv::remap 整幅去畸变，相机模型构造为零畸变(纯针孔)，
  // 避免和 world2cam 里的解析畸变重复处理。
  bool undistort_en_ = true;
  bool undistort_ready_ = false;   // 去畸变映射是否已构建成功
  int calib_img_width_ = 0, calib_img_height_ = 0;  // 去畸变映射对应的原图分辨率
  cv::Mat undist_map1_, undist_map2_;

  PointCloudXYZI::Ptr visual_sub_map;
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZRGB::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;
  VIOManagerPtr vio_manager;
  GeographicLib::LocalCartesian gps_trans_;

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Subscriber subGPS_pvt;
  ros::Subscriber sub_localization_odom;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;

  bool rtk_en = false;
  bool rtk_ini = false;
  // RTK 观测来源换成 /localization/odom（nav_msgs/Odometry）而不是 GNSS PVT
  bool odom_rtk_en_ = false;
  std::string odom_rtk_topic_;
  bool rtk_good = false;
  // 断流恢复健壮性处理总开关：关闭时行为与之前完全一致(硬複位+无门限)。
  // 打开后：1) 断流(间隔>rtk_gap_thresh_)后第一条样本跳过硬複位+协方差清零+陈旧速度注入；
  //        2) handleRTK/handleLIO 对 RTK 位置新息做门限校验，超过 rtk_innov_gate_ 倍 sigma 则丢弃该次修正。
  bool rtk_robust_recovery_en_ = false;
  double rtk_gap_thresh_ = 1.0;   // 秒；超过此间隔视为“断流后恢复”
  double rtk_innov_gate_ = 5.0;   // 新息门限，单位：sigma 倍数
  std::deque<RTK> rtk_buffer;
  std::deque<RTK> rtk_proc_buffer;
  std::deque<std::vector<double>> livo_state_buffer;
  Sophus::SE3 T_W_to_G;
  Sophus::SE3 T_G_to_W;
  vector<double> T_I_R;

  int pcd_file_index = 0;
  bool debug_mode = false;
  std::string save_directory;
  std::string pcd_save_file;
  std::string rtk_save_file;
  std::string imu_save_file;
  std::string odom_save_file; 
  std::string cov_save_file; 
  
  double keyframe_time = 0.0;

  // ==== Global-LVBA keyframe 数据集导出 ====
  bool save_lvba_en_ = false;        // 是否导出为 Global-LVBA keyframe 格式
  std::string lvba_out_dir_;         // 输出根目录（下建 pointclouds/ odoms/ images/<cam>/ calib/）
  int lvba_kf_index_ = -1;           // 关键帧序号：每个 LIO 关键帧 +1，作为文件名 {idx}_{ts} 的 idx
  std::string lvba_cur_stem_;        // 当前关键帧主干 {idx}_{ts}（LIO 存点云/odom，VIO 存图并覆写 odom 时复用）
  double lvba_cur_kf_time_ = -1.0;   // 当前已存激光关键帧的时间戳，用于 VIO 端严格配对

  ros::Subscriber sub_gps;
  ros::Publisher pub_odom;
  ros::Publisher pub_lidarRGB;
  ros::Publisher pub_rtk;
};
#endif