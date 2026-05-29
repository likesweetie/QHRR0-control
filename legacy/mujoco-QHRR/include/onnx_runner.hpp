#pragma once

#include <iostream>
#include <string>
#include <deque>
#include <vector>
#include <algorithm>
#include <unordered_map>

#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

class OnnxRunner
{
public:
    OnnxRunner(std::string file_name, const YAML::Node& config, const YAML::Node& obs_config);
    virtual ~OnnxRunner();

    void set_commands(double command_x, double command_y, double command_yaw);
    void set_joints(double pos[], double vel[]);
    void set_base_angular_velocity(Eigen::Vector3d base_ang_vel);
    void set_quaternion(Eigen::Quaterniond quat);
    void set_map(std::vector<double> map);
    void set_default_angle(std::vector<double> new_default_angle);
    void set_mode(bool mode);


    void change_policy(std::string file_name, const YAML::Node& config, const YAML::Node& obs_config);

    std::vector<double> get_action() { return scaled_actions; }
    int num_joint() const { return num_joint_; }

    virtual void compute_policy();
    virtual void reset_observations();

protected:
    virtual std::vector<double> compute_observation_();

    std::string file_path_, file_name_;
    std::string policy_path;

    Ort::Env env;
    Ort::Session* session{nullptr};
    Ort::SessionOptions session_options;
    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::string> input_names_str;
    std::vector<std::string> output_names_str;

    std::deque<std::vector<double>> obs_history_;
    int history_length_ = 1;

    int obs_type_ = 0;
    int num_joint_{0};

    double lin_vel_scale{1.0};
    double ang_vel_scale{1.0};
    double dof_pos_scale{1.0};
    double dof_vel_scale{1.0};
    double action_scale{1.0};
    double action_clip_{1.0};

    Eigen::Quaterniond quat_;
    Eigen::Vector3d gravity_vec_;
    Eigen::Vector3d base_ang_vel_;
    Eigen::Vector3d commands_;

    std::vector<double> dof_pos;
    std::vector<double> dof_vel;
    std::vector<double> map_;
    std::vector<double> default_joint_angle_;
    std::vector<double> delta_dof_pos;
    std::vector<double> actions;
    std::vector<double> scaled_actions;
    bool mode_{false};

    std::vector<double> last_actions;
    std::vector<double> last_last_actions;
    std::vector<double> last_last_last_actions;

    std::vector<int> input_joint_idx_conversion_, output_joint_idx_conversion_;
    std::string joint_idx_style_;

    std::vector<std::string> obs_components_;
    std::unordered_map<std::string, double> obs_scales_;

    int history_stride_ = 1;
    int step_counter_ = 0;

    int nan_counter = 0;           // 클래스 멤버로 선언
    const int max_nan_allowed = 3; // 연속 NaN 허용 횟수

private:
    void parsing_yaml_(const YAML::Node& config, const YAML::Node& obs_config);
    void initialize_policy_();
    void load_obs_config_(const YAML::Node& obs_config);
    double get_obs_scale_(const std::string& key, double default_scale) const;
};
