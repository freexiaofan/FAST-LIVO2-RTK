#ifndef LIVO_SENSOR_EXTRINSICS_HPP
#define LIVO_SENSOR_EXTRINSICS_HPP

// 移植自 Global-LVBA 的 sensor_extrinsics.hpp（原出处 HBA/source/sensor_extrinsics_ros.cpp），
// 去掉了 ROS/tf2 静态广播相关部分，只保留：
//   - 解析 {id}_{child}_2_{parent}_Extrinsics.yaml（OpenCV YAML，r_quaternion_wxyz + t_metric_xyz）
//   - 构建 TF 树，lookupTransform 查询任意两坐标系间变换
// 用 yaml-cpp 解析（已验证可解析 %YAML:1.0 + !!opencv-matrix 格式）。

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <sstream>
#include <filesystem>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

// 4x4 变换包装
struct TFTransform {
    Eigen::Matrix4d mat;
    TFTransform() : mat(Eigen::Matrix4d::Identity()) {}
    TFTransform(const Eigen::Matrix4d &m) : mat(m) {}
};

class SensorExtrinsics {
public:
    // 与 tf2_ros Buffer::lookupTransform 一致：out = source 在 target 下的变换 (T_target_source)
    bool lookupTransform(const std::string &target_frame,
                         const std::string &source_frame,
                         Eigen::Matrix4d &out) {
        if (target_frame == source_frame) {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        std::unordered_set<std::string> visited;
        return lookupTransformImpl(target_frame, source_frame, out, visited);
    }

    // 从文件夹加载全部外参文件
    void loadFolder(const std::string &folder) {
        for (const auto &entry : std::filesystem::directory_iterator(folder)) {
            if (entry.is_regular_file())
                loadFile(entry.path().string());
        }
    }

    // 添加一条 TF：parent -> child（mat 表示 child 坐标 -> parent 坐标）
    void addTF(const std::string &parent, const std::string &child, const Eigen::Matrix4d &tf) {
        tf_map_[child].emplace_back(parent, TFTransform(tf));
    }

private:
    // 递归实现，visited 防环
    bool lookupTransformImpl(const std::string &target, const std::string &source,
                             Eigen::Matrix4d &out, std::unordered_set<std::string> &visited) {
        if (target == source) {
            out = Eigen::Matrix4d::Identity();
            return true;
        }
        if (visited.count(source))
            return false;
        visited.insert(source);
        // source 的父节点方向
        if (tf_map_.count(source)) {
            for (const auto &pr : tf_map_[source]) {
                const std::string &parent = pr.first;
                const TFTransform &tf = pr.second;
                Eigen::Matrix4d parent2target;
                if (lookupTransformImpl(target, parent, parent2target, visited)) {
                    out = parent2target * tf.mat;
                    return true;
                }
            }
        }
        // 子节点方向（逆向）
        for (const auto &kv : tf_map_) {
            const std::string &maybe_child = kv.first;
            for (const auto &pr : kv.second) {
                const std::string &parent = pr.first;
                const TFTransform &tf = pr.second;
                if (parent == source) {
                    Eigen::Matrix4d child2target;
                    if (lookupTransformImpl(target, maybe_child, child2target, visited)) {
                        out = child2target * tf.mat.inverse();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // 解析单个外参文件：文件名形如 {id}_{child}_2_{parent}_Extrinsics.yaml。
    // 注意 child/parent 自身可能带下划线（比如 front_lidar、front_pinhole_wide），
    // 不能像最初那样直接按固定下标 tokens[1]/tokens[3] 取——那样 child 一旦是多词
    // 名字，后面所有 token 的下标都会错位（这是排查里实测踩到的坑）。这里改成先去掉
    // 扩展名和末尾的 Extrinsics/extrinsics 标记，再从第 2 个 token 开始找字面量正好是
    // "2" 的那个分隔 token，之前的 token 拼回去是 child，之后的拼回去是 parent。
    void loadFile(const std::string &file) {
        std::string filename = file.substr(file.find_last_of("/\\") + 1);
        const size_t dot = filename.find_last_of('.');
        std::string stem = (dot != std::string::npos) ? filename.substr(0, dot) : filename;

        std::vector<std::string> tokens;
        std::stringstream ss(stem);
        std::string token;
        while (std::getline(ss, token, '_'))
            tokens.push_back(token);

        if (!tokens.empty() && (tokens.back() == "Extrinsics" || tokens.back() == "extrinsics"))
            tokens.pop_back();

        std::string parent, child;
        if (tokens.size() >= 4) {   // 至少 {id} {child...} 2 {parent...}
            int sep_idx = -1;
            for (size_t i = 2; i < tokens.size(); ++i) {
                if (tokens[i] == "2") { sep_idx = static_cast<int>(i); break; }
            }
            if (sep_idx > 0 && static_cast<size_t>(sep_idx) + 1 < tokens.size()) {
                auto join = [](const std::vector<std::string> &v, size_t b, size_t e) {
                    std::string s;
                    for (size_t i = b; i < e; ++i) { if (!s.empty()) s += "_"; s += v[i]; }
                    return s;
                };
                child = join(tokens, 1, static_cast<size_t>(sep_idx));
                parent = join(tokens, static_cast<size_t>(sep_idx) + 1, tokens.size());
            }
        }
        try {
            YAML::Node root = YAML::LoadFile(file);
            std::vector<double> quat(4, 0.0);   // w, x, y, z
            std::vector<double> trans(3, 0.0);
            if (root["r_quaternion_wxyz"]) {
                auto qdata = root["r_quaternion_wxyz"]["data"];
                for (int i = 0; i < 4; ++i) quat[i] = qdata[i].as<double>();
            }
            if (root["t_metric_xyz"]) {
                auto tdata = root["t_metric_xyz"]["data"];
                for (int i = 0; i < 3; ++i) trans[i] = tdata[i].as<double>();
            }
            if (!parent.empty() && !child.empty()) {
                Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
                Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
                mat.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
                mat(0, 3) = trans[0];
                mat(1, 3) = trans[1];
                mat(2, 3) = trans[2];
                addTF(parent, child, mat);
                std::cout << "[SensorExtrinsics] Loaded TF: " << parent << " -> " << child << std::endl;
            }
        } catch (const std::exception &e) {
            std::cerr << "[SensorExtrinsics] YAML parse error in " << file << ": " << e.what() << std::endl;
        }
    }

    // child -> vector of (parent, transform)
    std::unordered_map<std::string, std::vector<std::pair<std::string, TFTransform>>> tf_map_;
};

#endif  // LIVO_SENSOR_EXTRINSICS_HPP
