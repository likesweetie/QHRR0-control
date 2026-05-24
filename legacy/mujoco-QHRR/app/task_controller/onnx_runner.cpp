#include "onnx_runner.hpp"

#include <array>
#include <stdexcept>
#include <utility>

namespace {

std::vector<std::string> DefaultObsComponents() {
    return {
        "base_ang_vel_",
        "projected_gravity",
        "lin_vel_x_commands_",
        "lin_vel_y_commands_",
        "ang_vel_z_commands_",
        "delta_dof_pos",
        "dof_vel",
        "actions",
    };
}

}  // namespace

OnnxRunner::OnnxRunner(std::string file_name, const YAML::Node& config, const YAML::Node& obs_config)
{
    file_name_ = std::move(file_name);
    gravity_vec_ << 0, 0, -1;
    quat_ = Eigen::Quaterniond::Identity();
    base_ang_vel_.setZero();
    commands_.setZero();

    parsing_yaml_(config, obs_config);
    initialize_policy_();

    std::cout << "[OnnxRunner] OnnxRunner initialized" << std::endl;
}

OnnxRunner::~OnnxRunner()
{
    delete session;
}

void OnnxRunner::load_obs_config_(const YAML::Node& obs_config)
{
    obs_components_.clear();
    obs_scales_.clear();

    const YAML::Node components = obs_config["observations"]["components"];
    if (components && components.IsSequence()) {
        for (const auto& component : components) {
            obs_components_.push_back(component.as<std::string>());
        }
    }

    if (obs_components_.empty()) {
        obs_components_ = DefaultObsComponents();
    }

    const YAML::Node scales = obs_config["observations"]["scales"];
    if (scales && scales.IsMap()) {
        for (auto it = scales.begin(); it != scales.end(); ++it) {
            obs_scales_[it->first.as<std::string>()] = it->second.as<double>();
        }
    }
}

double OnnxRunner::get_obs_scale_(const std::string& key, double default_scale) const
{
    const auto it = obs_scales_.find(key);
    if (it != obs_scales_.end()) {
        return it->second;
    }
    return default_scale;
}

void OnnxRunner::parsing_yaml_(const YAML::Node& config, const YAML::Node& obs_config)
{
    file_path_ = config["file_path"].as<std::string>();

    num_joint_ = config["num_joint"].as<int>();

    dof_pos.resize(num_joint_);
    dof_vel.resize(num_joint_);
    default_joint_angle_.resize(num_joint_);
    delta_dof_pos.resize(num_joint_);
    actions.resize(num_joint_);
    scaled_actions.resize(num_joint_);
    last_actions.resize(num_joint_);
    last_last_actions.resize(num_joint_);
    last_last_last_actions.resize(num_joint_);  

    joint_idx_style_ = config["joint_idx_style"].as<std::string>();
    std::string group_name = "joint_idx_conversion_" + joint_idx_style_;
    input_joint_idx_conversion_ = config[group_name]["input"].as<std::vector<int>>();
    output_joint_idx_conversion_ = config[group_name]["output"].as<std::vector<int>>();

    std::vector<double> default_joint_angle_original = config["default_joint_angle"].as<std::vector<double>>();
    std::cout << "[OnnxRunner] Default joint angles: ";
    for (const auto& angle : default_joint_angle_original) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;

    int i = 0;
    for (int idx : input_joint_idx_conversion_)
    {
        default_joint_angle_[i] = default_joint_angle_original[idx];
        i++;
    }

    action_scale = config["action_scale"].as<double>();
    action_clip_ = config["action_clip"].as<double>();

    obs_type_ = config["obs_type"].as<int>();

    load_obs_config_(obs_config);
}

void OnnxRunner::initialize_policy_()
{
    policy_path = file_path_ + file_name_;

    env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "OnnxRunner");
    session_options = Ort::SessionOptions();
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session = new Ort::Session(env, policy_path.c_str(), session_options);

    Ort::AllocatorWithDefaultOptions allocator;
    input_names_str.clear();
    input_names.clear();
    output_names_str.clear();
    output_names.clear();

    input_names_str.reserve(session->GetInputCount());
    for (size_t i = 0; i < session->GetInputCount(); i++) {
        auto name_ptr = session->GetInputNameAllocated(i, allocator);
        input_names_str.emplace_back(name_ptr.get());
    }
    input_names.reserve(input_names_str.size());
    for (auto& s : input_names_str) {
        input_names.push_back(s.c_str());
        std::cout << "[DEBUG] Parsed input name: " << s << std::endl;
    }

    output_names_str.reserve(session->GetOutputCount());
    for (size_t i = 0; i < session->GetOutputCount(); i++) {
        auto name_ptr = session->GetOutputNameAllocated(i, allocator);
        output_names_str.emplace_back(name_ptr.get());
    }
    output_names.reserve(output_names_str.size());
    for (auto& s : output_names_str) {
        output_names.push_back(s.c_str());
        std::cout << "[DEBUG] Parsed output name: " << s << std::endl;
    }

    std::cout << "[OnnxRunner] ONNX environment initialized" << std::endl;
}

void OnnxRunner::set_commands(double command_x, double command_y, double command_yaw)
{
    commands_ << command_x, command_y, command_yaw;
}

void OnnxRunner::set_joints(double pos[], double vel[])
{
    std::vector<double> pos_reordered(num_joint_, 0.0);
    std::vector<double> vel_reordered(num_joint_, 0.0);

    int i = 0;
    for (int idx : input_joint_idx_conversion_)
    {
        pos_reordered[i] = pos[idx];
        vel_reordered[i] = vel[idx];
        i++;
    }

    for (int joint = 0; joint < num_joint_; joint++)
    {
        dof_pos[joint] = pos_reordered[joint];
        switch(obs_type_)
        {
            case 0:
                delta_dof_pos[joint] = dof_pos[joint] - default_joint_angle_[joint];
                break;
            case 1:
                delta_dof_pos[joint] = dof_pos[joint];
                break;
            default:
                delta_dof_pos[joint] = dof_pos[joint] - default_joint_angle_[joint];
                break;
        }
        dof_vel[joint] = vel_reordered[joint];
    }
}

void OnnxRunner::set_base_angular_velocity(Eigen::Vector3d base_ang_vel)
{
    base_ang_vel_ = base_ang_vel;
}

void OnnxRunner::set_quaternion(Eigen::Quaterniond quat)
{
    quat_ = quat;
}

void OnnxRunner::set_default_angle(std::vector<double> new_default_angle)
{
    int i = 0;
    for (int idx : input_joint_idx_conversion_)
    {
        default_joint_angle_[i] = new_default_angle[idx];
        i++;
    }
}

void OnnxRunner::set_map(std::vector<double> map)
{
    map_ = std::move(map);
}

void OnnxRunner::set_mode(bool mode)
{
    mode_ = mode;
}

std::vector<double> OnnxRunner::compute_observation_()
{
    std::vector<double> obs_vec;

    
    const Eigen::Matrix3d rotation_matrix = quat_.normalized().toRotationMatrix();
    const Eigen::Vector3d projected_gravity = rotation_matrix.transpose() * gravity_vec_;
    const double alpha = std::clamp(((-0.0 - projected_gravity[2]) / 1.0), 0.0, 1.0);
    const double beta = std::clamp(((-projected_gravity[0]) / 1.0), 0.0, 1.0) * mode_;

    for (const std::string& component : obs_components_) {
        if (component == "base_ang_vel_") {
            const double scale = get_obs_scale_(component, ang_vel_scale);
            for (int i = 0; i < 3; ++i) {
                obs_vec.push_back(base_ang_vel_[i] * scale);
            }
            continue;
        }

        if (component == "projected_gravity") {
            const double scale = get_obs_scale_(component, 1.0);
            for (int i = 0; i < 3; ++i) {
                obs_vec.push_back(projected_gravity[i] * scale);
            }
            continue;
        }

        if (component == "lin_vel_x_commands_") {
            const double scale = get_obs_scale_(component, lin_vel_scale);
            obs_vec.push_back(commands_[0] * scale);
            continue;
        }

        if (component == "lin_vel_y_commands_") {
            const double scale = get_obs_scale_(component, lin_vel_scale);
            obs_vec.push_back(commands_[1] * scale);
            continue;
        }

        if (component == "ang_vel_z_commands_") {
            const double scale = get_obs_scale_(component, ang_vel_scale);
            obs_vec.push_back(commands_[2] * scale);
            continue;
        }

        if (component == "dof_pos") {
            const double scale = get_obs_scale_(component, dof_pos_scale);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(dof_pos[i] * scale);
            }
            continue;
        }

        if (component == "delta_dof_pos") {
            const double scale = get_obs_scale_(component, dof_pos_scale);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(delta_dof_pos[i] * scale);
            }
            continue;
        }

        if (component == "dof_vel") {
            const double scale = get_obs_scale_(component, dof_vel_scale);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(dof_vel[i] * scale);
            }
            continue;
        }

        if (component == "last_actions") {
            const double scale = get_obs_scale_(component, 1.0);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(last_actions[i] * scale);
            }
            continue;
        }

        if (component == "last_last_actions") {
            const double scale = get_obs_scale_(component, 1.0);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(last_last_actions[i] * scale);
            }
            continue;
        }

        if (component == "last_last_last_actions") {
            const double scale = get_obs_scale_(component, 1.0);
            for (int i = 0; i < num_joint_; ++i) {
                obs_vec.push_back(last_last_last_actions[i] * scale);
            }
            continue;
        }

        if (component == "alpha") {
            const double scale = get_obs_scale_(component, 1.0);
            obs_vec.push_back(alpha * scale);
            continue;
        }

        if (component == "beta") {
            const double scale = get_obs_scale_(component, 1.0);
            obs_vec.push_back(beta * scale);
            continue;
        }

        if (component == "mode") {
            const double scale = get_obs_scale_(component, 1.0);
            obs_vec.push_back(mode_ * scale);
            continue;
        }

        throw std::runtime_error("[OnnxRunner] Unsupported observation component: " + component);
    }

    return obs_vec;
}

void OnnxRunner::compute_policy()
{
    std::vector<double> obs_vec = compute_observation_();
    std::vector<float> obs_vec_fp32(obs_vec.size(), 0.0f);
    for (size_t i = 0; i < obs_vec.size(); ++i) {
        obs_vec_fp32[i] = static_cast<float>(obs_vec[i]);
    }

    std::vector<int64_t> obs_shape{1, static_cast<int64_t>(obs_vec.size())};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_obs = Ort::Value::CreateTensor<float>(
        memory_info, obs_vec_fp32.data(), obs_vec_fp32.size(), obs_shape.data(), obs_shape.size()
    );
    std::array<Ort::Value, 1> input_tensors = {std::move(input_obs)};

    auto output_tensors = session->Run(
        Ort::RunOptions{nullptr},
        input_names.data(), input_tensors.data(), input_tensors.size(),
        output_names.data(), 1
    );

    float* output_data = output_tensors[0].GetTensorMutableData<float>();

    std::vector<double> scaled_actions_original(num_joint_, 0.0);

    for (int i = 0; i < num_joint_; i++)
    {
        actions[i] = std::clamp(static_cast<double>(output_data[i]), -action_clip_, action_clip_);
        scaled_actions_original[i] = actions[i] * action_scale;
    }

    int i = 0;
    for (int idx : output_joint_idx_conversion_)
    {
        scaled_actions[i] = scaled_actions_original[idx];
        i++;
    }


    bool is_nan = false;
    for (int i = 0; i < num_joint_; i++)
    {
        if (std::isnan(output_data[i]) || std::isinf(output_data[i]))
        {
            is_nan = true;

            break;
        }
    }

    if (is_nan)
    {
        nan_counter++;
        std::cerr << "[⚠️ Warning] NaN detected! Counter: " << nan_counter << std::endl;

        for (int i = 0; i < num_joint_; i++)
        {
            actions[i] = 0.0f;
            scaled_actions[i] = 0.0f;
        }

        if (nan_counter >= max_nan_allowed)
        {
            is_nan = true;
            std::cerr << "[🛑 ERROR] NaN persisted. Resetting robot..." << std::endl;

            for (int i = 0; i < num_joint_; i++)
            {
                actions[i] = 0.0f;
                scaled_actions[i] = 0.0f;
            }

            nan_counter = 0; // 카운터 초기화
        }
        return;
    }

    // 정상 출력일 경우 카운터 초기화
    nan_counter = 0;


    last_last_last_actions = last_last_actions;
    last_last_actions = last_actions;
    last_actions = actions;

    step_counter_++;
}

void OnnxRunner::reset_observations()
{
    for (int i = 0; i < num_joint_; i++)
    {
        dof_pos[i] = 0.0;
        dof_vel[i] = 0.0;
        delta_dof_pos[i] = 0.0;
        actions[i] = 0.0;
        scaled_actions[i] = 0.0;
        last_actions[i] = 0.0;
        last_last_actions[i] = 0.0;
        last_last_last_actions[i] = 0.0;
    }

    step_counter_ = 0;
    // (void)compute_observation_();
}

void OnnxRunner::change_policy(std::string file_name, const YAML::Node& config, const YAML::Node& obs_config)
{
    if (session != nullptr) {
        delete session;
        session = nullptr;
    }

    file_name_ = std::move(file_name);

    parsing_yaml_(config, obs_config);
    reset_observations();
    initialize_policy_();

    std::cout << "[OnnxRunner] OnnxRunner initialized" << std::endl;
}
