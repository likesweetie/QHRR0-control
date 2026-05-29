#include "task_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "rnn_onnx_runner.hpp"

namespace {
namespace fs = std::filesystem;

struct RunnerConfigBundle {
    std::string name;
    std::string directory;
    YAML::Node runner_config;
    YAML::Node obs_config;
};

fs::path ResolveProjectRoot() {
    const char* explicit_root = std::getenv("QHRR_PROJECT_ROOT");
    fs::path root = (explicit_root && explicit_root[0])
        ? fs::path(explicit_root)
        : fs::current_path();
    if (root.is_relative()) {
        root = fs::current_path() / root;
    }
    if (!fs::exists(root) || !fs::is_directory(root)) {
        throw std::runtime_error("[task_controller] QHRR_PROJECT_ROOT is not a directory: " + root.string());
    }
    return fs::weakly_canonical(root);
}

bool HasPolicyConfig(const fs::path& path) {
    return fs::exists(path / "policy_list.yaml") || fs::exists(path / "runner_config.yaml");
}

fs::path ResolvePathFromProjectRoot(const fs::path& path) {
    if (path.is_absolute()) {
        return path;
    }
    return ResolveProjectRoot() / path;
}

std::string ResolvePolicyConfigDir() {
    const char* explicit_dir = std::getenv("POLICY_CONFIG_DIR");
    if (!explicit_dir || !explicit_dir[0]) {
        throw std::runtime_error("[task_controller] POLICY_CONFIG_DIR environment variable is not set");
    }

    const fs::path policy_config_root = ResolvePathFromProjectRoot(explicit_dir);
    if (HasPolicyConfig(policy_config_root)) {
        return fs::weakly_canonical(policy_config_root).string();
    }

    const char* robot_name = std::getenv("ROBOT_NAME");
    if (!robot_name || !robot_name[0]) {
        throw std::runtime_error(
            "[task_controller] POLICY_CONFIG_DIR is a policy root, so ROBOT_NAME is required: " +
            policy_config_root.string());
    }

    const fs::path robot_policy_config = policy_config_root / robot_name;
    if (!HasPolicyConfig(robot_policy_config)) {
        throw std::runtime_error(
            "[task_controller] policy config directory not found: " +
            robot_policy_config.string());
    }
    return fs::weakly_canonical(robot_policy_config).string();
}

YAML::Node LoadOptionalYaml(const fs::path& path) {
    if (!fs::exists(path)) {
        return YAML::Node();
    }
    return YAML::LoadFile(path.string());
}

std::vector<RunnerConfigBundle> LoadRunnerBundles(const std::string& policy_config_dir) {
    std::vector<RunnerConfigBundle> bundles;
    const fs::path root(policy_config_dir);

    if (!fs::exists(root) || !fs::is_directory(root)) {
        throw std::runtime_error("[task_controller] policy config directory not found: " + policy_config_dir);
    }

    const fs::path policy_list_path = root / "policy_list.yaml";
    if (fs::exists(policy_list_path)) {
        const YAML::Node policy_list = YAML::LoadFile(policy_list_path.string());
        const YAML::Node names = policy_list["list_of_policy_names"];
        if (!names || !names.IsSequence()) {
            throw std::runtime_error("[task_controller] invalid policy_list.yaml: " + policy_list_path.string());
        }

        for (const auto& name_node : names) {
            std::cout << "[task_controller] waiting for session: " << name_node.as<std::string>() << std::endl;
            const std::string session_name = name_node.as<std::string>();
            const fs::path session_dir = root / session_name;
            const fs::path runner_config_path = session_dir / "runner_config.yaml";
            if (!fs::exists(runner_config_path)) {
                throw std::runtime_error(
                    "[task_controller] session config not found for policy '" + session_name + "': " +
                    runner_config_path.string());
            }

            RunnerConfigBundle bundle;
            bundle.name = session_name;
            bundle.directory = session_dir.string();
            bundle.runner_config = YAML::LoadFile(runner_config_path.string());
            bundle.obs_config = LoadOptionalYaml(session_dir / "obs_config.yaml");
            bundles.push_back(std::move(bundle));
        }
        return bundles;
    }

    const fs::path runner_config_path = root / "runner_config.yaml";
    if (!fs::exists(runner_config_path)) {
        throw std::runtime_error("[task_controller] no runner_config.yaml found under: " + policy_config_dir);
    }

    RunnerConfigBundle bundle;
    bundle.name = root.filename().string();
    bundle.directory = root.string();
    bundle.runner_config = YAML::LoadFile(runner_config_path.string());
    bundle.obs_config = LoadOptionalYaml(root / "obs_config.yaml");
    bundles.push_back(std::move(bundle));
    return bundles;
}

fs::path StripLeadingParents(const fs::path& path) {
    fs::path stripped;
    bool stripping = true;
    for (const auto& part : path) {
        const std::string value = part.string();
        if (stripping && (value == "." || value == "..")) {
            continue;
        }
        stripping = false;
        stripped /= part;
    }
    return stripped.empty() ? path : stripped;
}

std::string ResolvePolicyFilePath(const RunnerConfigBundle& bundle) {
    if (!bundle.runner_config["file_path"]) {
        throw std::runtime_error("[task_controller] runner_config missing file_path for session: " + bundle.name);
    }

    const fs::path raw_path(bundle.runner_config["file_path"].as<std::string>());
    std::vector<fs::path> candidates;
    if (raw_path.is_absolute()) {
        candidates.push_back(raw_path);
    } else {
        const fs::path project_root = ResolveProjectRoot();
        candidates.push_back(project_root / raw_path);
        candidates.push_back(project_root / StripLeadingParents(raw_path));
    }

    for (const fs::path& candidate : candidates) {
        const fs::path policy_path = candidate / "policy.onnx";
        if (fs::exists(policy_path)) {
            fs::path resolved = fs::weakly_canonical(candidate);
            return (resolved / "").string();
        }
    }

    throw std::runtime_error(
        "[task_controller] policy.onnx not found for session '" + bundle.name +
        "' from file_path: " + raw_path.string());
}

std::unique_ptr<OnnxRunner> CreateRunner(const RunnerConfigBundle& bundle) {
    YAML::Node runner_config = YAML::Clone(bundle.runner_config);
    runner_config["file_path"] = ResolvePolicyFilePath(bundle);
    const std::string runner_type =
        runner_config["runner_type"] ? runner_config["runner_type"].as<std::string>() : "rnn";

    if (runner_type == "onnx") {
        std::cout << "[task_controller] using OnnxRunner for session " << bundle.name << std::endl;
        return std::make_unique<OnnxRunner>("policy.onnx", runner_config, bundle.obs_config);
    }

    if (runner_type == "rnn") {
        std::cout << "[task_controller] using RNNOnnxRunner for session " << bundle.name << std::endl;
        return std::make_unique<RNNOnnxRunner>("policy.onnx", runner_config, bundle.obs_config);
    }

    throw std::runtime_error(
        "[task_controller] Unsupported runner_type: " + runner_type +
        " (expected 'rnn' or 'onnx')");
}

}  // namespace

TaskController::TaskController(double control_frequency, ShmData* shared_memory)
    : runner_(nullptr),
      shared_memory_(shared_memory),
      control_frequency_(control_frequency),
      state_(TASK_INIT),
      running_(false),
      last_state_seq_(0) {}

TaskController::~TaskController() {
    stop();
    runner_ = nullptr;
}

void TaskController::initialize() {
    sessions_.clear();

    const std::string policy_config_dir = ResolvePolicyConfigDir();
    const std::vector<RunnerConfigBundle> bundles = LoadRunnerBundles(policy_config_dir);

    for (const auto& bundle : bundles) {
        RunnerSession session;
        session.name = bundle.name;
        session.directory = bundle.directory;
        session.runner = CreateRunner(bundle);
        session.runner->reset_observations();
        sessions_.push_back(std::move(session));
    }

    if (sessions_.empty()) {
        throw std::runtime_error("[task_controller] no runner sessions were loaded");
    }

    runner_ = sessions_[0].runner.get();
    auto defaults = bundles[0].runner_config["default_joint_angle"].as<std::vector<double>>();
    default_joint_angle_.assign(defaults.begin(), defaults.end());
    std::cout << "[task_controller] default joint angles: ";
    for (const auto& angle : default_joint_angle_) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
    std::cout << "[task_controller] loaded runner" << sessions_[0].name
              << " runner session(s) from " << policy_config_dir << std::endl;

    state_ = TASK_RL;
    active_session_ = &sessions_[0];
}

void TaskController::start() {
    running_ = true;
    pthread_create(&thread_, nullptr, TaskController::thread_function_wrapper, this);
}

void TaskController::join() {
    pthread_join(thread_, nullptr);
}

void TaskController::stop() {
    running_ = false;
}

void* TaskController::thread_function_wrapper(void* context) {
    return static_cast<TaskController*>(context)->TaskController::loop();
}

void* TaskController::loop() {
    if (!runner_ || !shared_memory_) {
        std::cerr << "[task_controller] not initialized\n";
        return nullptr;
    }

    const double frequency = (control_frequency_ > 0.0) ? control_frequency_ : 50.0;
    const auto period = std::chrono::microseconds(
    static_cast<int64_t>(1000000.0 / frequency));

    int nq = 0, nv = 0, nu = 0;
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));


    std::cout << "[task_controller] loop start" << std::endl;
    nq = std::max(0, std::min(shared_memory_->nq, static_cast<int>(kShmMaxQpos)));
    nv = std::max(0, std::min(shared_memory_->nv, static_cast<int>(kShmMaxQvel)));
    nu = std::max(0, std::min(shared_memory_->nu, static_cast<int>(kShmMaxCtrl)));
    std::cout << "[task_controller] nq: " << nq << ", nv: " << nv << ", nu: " << nu << std::endl;
    int loop_count = 0;
    Eigen::Vector3d gravity_vec;
    gravity_vec << 0.0, 0.0, -1.0;
    while (running_.load()) 
    {
        const auto tic = std::chrono::steady_clock::now();

        loop_count++;


        Eigen::Quaterniond cur_quat = Eigen::Quaterniond((shared_memory_->quat[0]),(shared_memory_->quat[1]),(shared_memory_->quat[2]),(shared_memory_->quat[3]));

        const Eigen::Matrix3d rotation_matrix = cur_quat.normalized().toRotationMatrix();
        const Eigen::Vector3d projected_gravity = rotation_matrix.transpose() * gravity_vec;
        const double alpha = std::clamp(((-0.0 - projected_gravity[2]) / 1.0), 0.0, 1.0);
        const double beta = std::clamp(((-projected_gravity[0]) / 1.0), 0.0, 1.0) * shared_memory_->a_button;
        // const double alpha = 0.5;

        if ((state_ == TASK_INIT) && (loop_count > 100) && (active_session_->name == "rbq") )
        {
            state_ = TASK_RL;
        }
        
        // if ((state_ == TASK_RL) && (projected_gravity[2] > -0.5) && (active_session_->name == "rbq")) 
        // {
        //     state_ = TASK_RECOVERY;
        //     active_session_ = &sessions_[1];
        //     cur_runner_ = active_session_->runner.get();     
        //     cur_runner_->reset_observations();
        // }

        // if ((state_ == TASK_RECOVERY) && (projected_gravity[2] < -0.9) && (active_session_->name == "rbq_recovery")) 
        // {
        //     state_ = TASK_RL;
        //     active_session_ = &sessions_[0];
        //     cur_runner_ = active_session_->runner.get();     
        //     cur_runner_->reset_observations();

        // }


        cur_runner_ = active_session_->runner.get();     

                
        std::vector<double> action(nu, 0.0);
        std::vector<double> action_offset(nu, 0.0);


        for (int i = 0; i < nu; ++i) 
        {
            action[i] = (default_joint_angle_[i]);
        }

        if (active_session_->name == "rbq_recovery") 
        {
            for (int i = 0; i < nu; ++i) 
            {
                action_offset[i] = (shared_memory_->qpos[7 + i]);
            }
        }
        else if (active_session_->name == "rbq" || active_session_->name == "qhrr")
        {

            std::array<double, 12> stand_joint_pos;
            std::array<double, 12> quad_joint_pos = {
                0.0, 0.7, -1.4, 
                0.0, 0.7, -1.4, 
                0.0, 0.7, -1.4,
                0.0, 0.7, -1.4       // KP
            };

            // 1. stand_joint_pos = default_joint_angle_
            for (int i = 0; i < nu; ++i)
            {
                stand_joint_pos[i] = default_joint_angle_[i];
            }

            // 2. idx = [2, 3, 6, 7, 10, 11] 만 현재 joint_pos로 덮어쓰기
            // const std::array<int, 6> idx = {0, 1, 2, 3, 4, 5}; ##Front Flip
            const std::array<int, 6> idx = {6, 7, 8, 9, 10, 11}; //##Back Flip

            for (const int j : idx)
            {
                stand_joint_pos[j] = shared_memory_->qpos[7 + j];
            }

            // 3. alpha / beta blending
            for (int i = 0; i < nu; ++i)
            {
                const double q = shared_memory_->qpos[7 + i];

                action_offset[i] =
                    (alpha * 1.0 * (quad_joint_pos[i] - q))
                + (beta  * 2.0 * (stand_joint_pos[i] - q))
                + q;
            }
        }
        else if (active_session_->name == "qhrr1")
        {
            for (int i = 0; i < nu; ++i)
            {
                action_offset[i] = (default_joint_angle_[i]);
            }
        }
            



        std::vector<double> dof_pos(nu, 0.0);
        std::vector<double> dof_vel(nu, 0.0);

        const int joint_pos_count = std::max(0, std::min(nu, nq));
        const int joint_vel_count = std::max(0, std::min(nu, nv));

        for (int i = 0; i < joint_pos_count; ++i) 
        {
            dof_pos[i] = (shared_memory_->qpos[7 + i]);
        }
        for (int i = 0; i < joint_vel_count; ++i) 
        {
            dof_vel[i] = shared_memory_->qvel[6 + i];
        }

        cur_runner_->set_base_angular_velocity(Eigen::Vector3d(
            (shared_memory_->ang_vel[0]),
            (shared_memory_->ang_vel[1]),
            (shared_memory_->ang_vel[2])));
        cur_runner_->set_quaternion(Eigen::Quaterniond(
            (shared_memory_->quat[0]),
            (shared_memory_->quat[1]),
            (shared_memory_->quat[2]),
            (shared_memory_->quat[3])));
        cur_runner_->set_joints(dof_pos.data(), dof_vel.data());

        if (active_session_->name == "rbq_recovery") {
            cur_runner_->set_commands(1.0, 0.0 , 0.0);

        }
        else if (active_session_->name == "rbq" || active_session_->name == "qhrr" || active_session_->name == "qhrr1") {
            cur_runner_->set_commands(
            (shared_memory_->lin_vel_target[0]),
            (shared_memory_->lin_vel_target[1]),
            (shared_memory_->ang_vel_target[2]));
        }
        else {
            cur_runner_->set_commands(0.0, 0.0, 0.0);
        }
        cur_runner_->set_mode(shared_memory_->a_button);

        if ((state_ == TASK_INIT)) 
        {
            for (int i = 0; i < nu; ++i) 
            {
                action[i] = default_joint_angle_[i];
            }
            
        }
        else
        {
            cur_runner_->compute_policy();
            action = cur_runner_->get_action();
        }


        for (int i = 0; i < nu; ++i) 
        {
            shared_memory_->q_target[i] = action[i] + action_offset[i];
        }
        
        const auto elapsed = std::chrono::steady_clock::now() - tic;
        if (elapsed < period) {
            std::this_thread::sleep_for(period - elapsed);
        }
        else {
            std::cerr << "[task_controller] loop took longer than expected" << std::endl;
        }
    }

    return nullptr;
}
