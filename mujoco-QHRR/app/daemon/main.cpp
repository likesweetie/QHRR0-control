#include "daemon/daemon.hpp"

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <unistd.h>

namespace {

struct CliOptions {
    std::string config_path;
    std::string robot_name{"qhrr"};
};

struct RobotConfig {
    std::string model_path;
    std::string policy_config_dir;
};

static std::atomic<bool> g_run{true};
static std::atomic<int>  g_got_signal{0};
static Daemon d;

static void SafeWrite(const char* msg)
{
    ::write(STDERR_FILENO, msg, std::strlen(msg));
}

static void HandleSignal(int sig)
{
    g_got_signal.store(sig, std::memory_order_relaxed);
    g_run.store(false, std::memory_order_relaxed);
    SafeWrite("[WARN] signal caught, shutting down...\n");
}

static bool IsRobotName(const std::string& value)
{
    return value == "qhrr" || value == "rbq" || value == "qhrr1";
}

static bool ParseCli(int argc, char** argv, CliOptions& options)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i] ? argv[i] : "";
        if (arg.empty()) {
            continue;
        }

        if (arg == "--robot") {
            if (i + 1 >= argc) {
                std::cerr << "[ERROR] --robot requires a value\n";
                return false;
            }
            options.robot_name = argv[++i];
            continue;
        }

        if (arg == "--config") {
            if (i + 1 >= argc) {
                std::cerr << "[ERROR] --config requires a value\n";
                return false;
            }
            options.config_path = argv[++i];
            continue;
        }

        if (arg == "-h" || arg == "--help") {
            std::cout << "usage: daemon [--robot qhrr|rbq] [--config path]\n";
            return false;
        }

        if (arg.size() >= 5 && arg.substr(arg.size() - 5) == ".yaml") {
            options.config_path = arg;
            continue;
        }

        if (IsRobotName(arg)) {
            options.robot_name = arg;
            continue;
        }

        std::cerr << "[ERROR] unknown argument: " << arg << "\n";
        return false;
    }

    if (options.config_path.empty()) {
        if (::access("../config/app_config/daemon_config.yaml", R_OK) == 0) {
            options.config_path = "../config/app_config/daemon_config.yaml";
        } else {
            options.config_path = "config/app_config/daemon_config.yaml";
        }
    }

    if (!IsRobotName(options.robot_name)) {
        std::cerr << "[ERROR] unsupported robot: " << options.robot_name
                  << " (expected qhrr or rbq)\n";
        return false;
    }

    return true;
}

static bool LoadShmConfig(const YAML::Node& cfg, ShmConfig& shm)
{
    if (!cfg["shm"]) return false;
    const auto& s = cfg["shm"];

    if (!s["name"] || !s["size_bytes"]) return false;

    shm.name = s["name"].as<std::string>();
    shm.size_bytes = s["size_bytes"].as<std::size_t>();

    if (s["create_if_missing"]) shm.create_if_missing = s["create_if_missing"].as<bool>();
    if (s["unlink_on_destroy"]) shm.unlink_on_destroy = s["unlink_on_destroy"].as<bool>();
    return true;
}

static bool LoadProcessSpecs(const YAML::Node& cfg, std::vector<ProcessSpec>& procs)
{
    if (!cfg["processes"] || !cfg["processes"].IsSequence()) return false;

    for (const auto& p : cfg["processes"]) {
        if (!p["name"] || (!p["exec_path"] && !p["exec"])) {
            std::cerr << "[ERROR] each process needs 'name' and 'exec_path'(or legacy 'exec')\n";
            return false;
        }

        ProcessSpec spec;
        spec.name = p["name"].as<std::string>();

        if (p["exec_path"]) spec.exec_path = p["exec_path"].as<std::string>();
        else               spec.exec_path = p["exec"].as<std::string>();

        if (p["args"]) {
            if (!p["args"].IsSequence()) {
                std::cerr << "[ERROR] args must be sequence\n";
                return false;
            }
            for (const auto& a : p["args"]) spec.args.push_back(a.as<std::string>());
        }

        if (p["launch_in_terminal"]) spec.launch_in_terminal = p["launch_in_terminal"].as<bool>();
        else                        spec.launch_in_terminal = false;
        if (p["create_process_group"]) spec.create_process_group = p["create_process_group"].as<bool>();
        if (p["wait_after_launch"]) spec.wait_after_launch_sec = p["wait_after_launch"].as<double>();

        if (p["env"]) {
            if (!p["env"].IsMap()) {
                std::cerr << "[ERROR] env must be map\n";
                return false;
            }
            for (auto it = p["env"].begin(); it != p["env"].end(); ++it) {
                spec.env[it->first.as<std::string>()] = it->second.as<std::string>();
            }
        }

        procs.push_back(std::move(spec));
    }
    return true;
}

static bool LoadRobotConfig(const YAML::Node& cfg, const std::string& robot_name, RobotConfig& robot)
{
    const YAML::Node robots = cfg["robots"];
    if (!robots || !robots[robot_name]) {
        std::cerr << "[ERROR] robot config not found: " << robot_name << "\n";
        return false;
    }

    const YAML::Node selected = robots[robot_name];
    if (!selected["model_path"] || !selected["policy_config_dir"]) {
        std::cerr << "[ERROR] robot config requires model_path and policy_config_dir\n";
        return false;
    }

    robot.model_path = selected["model_path"].as<std::string>();
    robot.policy_config_dir = selected["policy_config_dir"].as<std::string>();
    return true;
}

static void ApplyRobotSelection(std::vector<ProcessSpec>& procs,
                                const std::string& robot_name,
                                const RobotConfig& robot_cfg)
{
    const std::string pd_config_path = robot_cfg.policy_config_dir + "/" + robot_name + "/pd_config.yaml";
    const std::string runner_config_path = robot_cfg.policy_config_dir +"/" + robot_name;

    for (auto& proc : procs) {
        proc.env["ROBOT_NAME"] = robot_name;
        proc.env["POLICY_CONFIG_DIR"] = robot_cfg.policy_config_dir;

        if (proc.name == "mujoco_simulate") {
            proc.args = {robot_cfg.model_path};
            proc.env["PD_CONFIG_PATH"] = pd_config_path;
        }

        if (proc.name == "task_controller") {
            
        }
    }
}

static void ShutdownAll(const ShmConfig& shm_cfg)
{
    d.TerminateAll(SIGTERM);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    d.TerminateAll(SIGKILL);

    if (d.shm) shm_utils::CloseShm(d.shm);
    if (shm_cfg.unlink_on_destroy) shm_utils::DeleteShmByName(shm_cfg.name.c_str());
}

}  // namespace

int main(int argc, char** argv)
{
    CliOptions options;
    if (!ParseCli(argc, argv, options)) {
        return 1;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(options.config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "[ERROR] failed to load config: " << e.what() << "\n";
        return 1;
    }

    ShmConfig shm_cfg;
    if (!LoadShmConfig(cfg, shm_cfg)) {
        std::cerr << "[ERROR] invalid shm config\n";
        return 1;
    }

    RobotConfig robot_cfg;
    if (!LoadRobotConfig(cfg, options.robot_name, robot_cfg)) {
        return 1;
    }

    bool created = false;
    d.shm = shm_utils::CreateShm(shm_cfg.name.c_str(), shm_cfg.create_if_missing, &created);
    if (!d.shm) {
        std::cerr << "[ERROR] failed to create/open shm: " << shm_cfg.name
                  << " errno=" << errno << " (" << std::strerror(errno) << ")\n";
        return 1;
    }

    if (created) {
        std::cout << "[INFO] SHM created: " << shm_cfg.name << "\n";
        std::memset(d.shm, 0, sizeof(ShmData));
        std::strncpy(d.shm->name, shm_cfg.name.c_str(), sizeof(d.shm->name) - 1);
        d.shm->name[sizeof(d.shm->name) - 1] = '\0';
    }

    std::vector<ProcessSpec> procs;
    if (!LoadProcessSpecs(cfg, procs)) {
        std::cerr << "[ERROR] invalid process list\n";
        ShutdownAll(shm_cfg);
        return 1;
    }

    ApplyRobotSelection(procs, options.robot_name, robot_cfg);
    std::cout << "[INFO] selected robot=" << options.robot_name
              << " model=" << robot_cfg.model_path
              << " policy_dir=" << robot_cfg.policy_config_dir << "\n";

    for (const auto& p : procs) {
        auto pid = d.Spawn(p);
        if (!pid) {
            std::cerr << "[WARN] spawn failed: " << p.name
                      << " errno=" << d.LastErrno() << " (" << std::strerror(d.LastErrno()) << ")\n";
        } else {
            std::cout << "[INFO] spawned: " << p.name << " pid=" << *pid << "\n";
            if (p.wait_after_launch_sec > 0.0) {
                const auto wait_ms =
                    static_cast<int>(p.wait_after_launch_sec * 1000.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
            }
        }
    }

    while (g_run.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    const int sig = g_got_signal.load(std::memory_order_relaxed);
    std::cout << "[INFO] shutdown signal=" << sig << "\n";

    ShutdownAll(shm_cfg);
    return 0;
}
