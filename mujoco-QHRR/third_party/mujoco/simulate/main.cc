// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Lab-specific MuJoCo simulate main.
// Target platform: Ubuntu PC.
//
// This file intentionally removes Windows/macOS compatibility branches from
// the original MuJoCo simulate sample main. The GUI and interaction model are
// still provided by mj::Simulate.

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <unistd.h>

#include <mujoco/mujoco.h>

#include <yaml-cpp/yaml.h>

#include "array_safety.h"
#include "glfw_adapter.h"
#include "simulate.h"

#include "mujoco_can/mujoco_can_bridge.hpp"
#include "mujoco_can/socket_can_adapter.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using seconds = std::chrono::duration<double>;

constexpr double k_sync_misalign = 0.1;
constexpr double k_sim_refresh_fraction = 0.7;
constexpr int k_error_length = 1024;

// Global model/data are kept to match the official simulate sample structure.
// The physics thread owns stepping, while the render thread accesses them
// through sim.mtx.
mjModel* g_model = nullptr;
mjData* g_data = nullptr;

// Virtual CAN bridge and optional SocketCAN/vcan adapter.
//
// Ownership:
//   - These objects are created in main() before the physics thread starts.
//   - The physics thread calls their runtime methods while holding sim.mtx.
//   - The render thread does not touch them directly.
std::unique_ptr<mjcan::MujocoCanBridge> g_can_bridge = nullptr;
std::unique_ptr<mjcan::SocketCanAdapter> g_socket_can_adapter = nullptr;

struct VirtualCanConfig {
  bool enabled = true;

  bool socketcan_enabled = true;
  std::string socketcan_interface = "vcan0";

  std::string base_body_name = "base";
  int motor_id_base = 1;

  double command_timeout_s = 0.1;

  mjcan::SPGMITConfig spg_mit_config;
  mjcan::E2BoxImuFirmwareConfig e2box_imu_config;
  mjcan::MujocoCanBridge::DeviceConfig device_config;
};

VirtualCanConfig g_virtual_can_config;

// Returns the directory containing the current executable.
//
// Ubuntu-only implementation:
//   /proc/self/exe -> absolute path to current executable.
std::string get_executable_dir() {
  std::size_t buffer_size = 256;

  while (true) {
    std::string path_buffer(buffer_size, '\0');

    const ssize_t written =
        readlink("/proc/self/exe", path_buffer.data(), path_buffer.size() - 1);

    if (written < 0) {
      std::fprintf(
          stderr,
          "[main] Failed to resolve /proc/self/exe: %s\n",
          std::strerror(errno));
      return "";
    }

    if (static_cast<std::size_t>(written) < path_buffer.size() - 1) {
      path_buffer.resize(static_cast<std::size_t>(written));

      const std::size_t slash_pos = path_buffer.find_last_of('/');
      if (slash_pos == std::string::npos) {
        std::fprintf(stderr, "[main] Failed to parse executable directory.\n");
        return "";
      }

      return path_buffer.substr(0, slash_pos);
    }

    buffer_size *= 2;
  }
}

// Loads MuJoCo plugins from ./mujoco_plugin next to the executable.
//
// This is kept because robot models may use MuJoCo actuator/sensor plugins.
// If your model does not use plugins, this function simply scans an empty or
// missing directory and continues.
void scan_plugin_libraries() {
  const int built_in_plugin_count = mjp_pluginCount();

  if (built_in_plugin_count > 0) {
    std::printf("[main] Built-in MuJoCo plugins:\n");

    for (int i = 0; i < built_in_plugin_count; ++i) {
      std::printf("  - %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  const std::string executable_dir = get_executable_dir();

  if (executable_dir.empty()) {
    std::printf("[main] Plugin scan skipped: executable directory not found.\n");
    return;
  }

  const std::string plugin_dir = executable_dir + "/" + MUJOCO_PLUGIN_DIR;

  std::printf("[main] Scanning MuJoCo plugin directory: %s\n", plugin_dir.c_str());

  mj_loadAllPluginLibraries(
      plugin_dir.c_str(),
      +[](const char* filename, int first, int count) {
        std::printf("[main] Plugins registered by '%s':\n", filename);

        for (int i = first; i < first + count; ++i) {
          std::printf("  - %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

// Loads the fixed MuJoCo CAN configuration from simulate/configs.
//
// CMake should define MUJOCO_SIM_CONFIG_DIR as:
//   ${MUJOCO_DIR}/simulate/configs
//
// Expected file:
//   ${MUJOCO_SIM_CONFIG_DIR}/mujoco_can.yaml
std::string get_fixed_mujoco_can_config_path() {
#ifdef MUJOCO_SIM_CONFIG_DIR
  return std::string(MUJOCO_SIM_CONFIG_DIR) + "/mujoco_can.yaml";
#else
  return "third_party/mujoco/simulate/configs/mujoco_can.yaml";
#endif
}

template <typename T>
T yaml_get_or(
    const YAML::Node& node,
    const std::string& key,
    const T& default_value) {
  if (!node || !node[key]) {
    return default_value;
  }

  try {
    return node[key].as<T>();
  } catch (const YAML::Exception& e) {
    std::printf(
        "[main] Invalid YAML value for key '%s': %s. Using default.\n",
        key.c_str(),
        e.what());
    return default_value;
  }
}

uint32_t yaml_get_u32_or(
    const YAML::Node& node,
    const std::string& key,
    uint32_t default_value) {
  if (!node || !node[key]) {
    return default_value;
  }

  try {
    if (node[key].IsScalar()) {
      const std::string text = node[key].as<std::string>();
      return static_cast<uint32_t>(std::stoul(text, nullptr, 0));
    }
  } catch (const std::exception& e) {
    std::printf(
        "[main] Invalid uint32 YAML value for key '%s': %s. Using default.\n",
        key.c_str(),
        e.what());
  }

  return default_value;
}

mjcan::SPGMITConfig load_spg_mit_config(
    const YAML::Node& node,
    const mjcan::SPGMITConfig& defaults) {
  mjcan::SPGMITConfig config = defaults;

  if (!node) {
    return config;
  }

  config.p_max_rad =
      yaml_get_or<double>(node, "p_max_rad", config.p_max_rad);
  config.v_max_rad_s =
      yaml_get_or<double>(node, "v_max_rad_s", config.v_max_rad_s);
  config.kp_max =
      yaml_get_or<double>(node, "kp_max", config.kp_max);
  config.kd_max =
      yaml_get_or<double>(node, "kd_max", config.kd_max);
  config.tau_max_nm =
      yaml_get_or<double>(node, "tau_max_nm", config.tau_max_nm);

  config.feedback_position_max_rad =
      yaml_get_or<double>(
          node,
          "feedback_position_max_rad",
          config.feedback_position_max_rad);

  config.iq_full_scale_count =
      yaml_get_or<double>(
          node,
          "iq_full_scale_count",
          config.iq_full_scale_count);
  config.iq_full_scale_current_a =
      yaml_get_or<double>(
          node,
          "iq_full_scale_current_a",
          config.iq_full_scale_current_a);

  config.periodic_feedback =
      yaml_get_or<bool>(
          node,
          "periodic_feedback",
          config.periodic_feedback);
  config.periodic_feedback_s =
      yaml_get_or<double>(
          node,
          "periodic_feedback_s",
          config.periodic_feedback_s);

  config.set_zero_hold_s =
      yaml_get_or<double>(
          node,
          "set_zero_hold_s",
          config.set_zero_hold_s);

  return config;
}

mjcan::E2BoxImuFirmwareConfig load_e2box_imu_config(
    const YAML::Node& node,
    const mjcan::E2BoxImuFirmwareConfig& defaults) {
  mjcan::E2BoxImuFirmwareConfig config = defaults;

  if (!node) {
    return config;
  }

  config.request_id =
      yaml_get_u32_or(node, "request_id", config.request_id);
  config.quat_id =
      yaml_get_u32_or(node, "quat_id", config.quat_id);
  config.gyro_id =
      yaml_get_u32_or(node, "gyro_id", config.gyro_id);

  config.cmd_get_quat = static_cast<uint8_t>(
      yaml_get_u32_or(node, "cmd_get_quat", config.cmd_get_quat));
  config.cmd_get_gyro = static_cast<uint8_t>(
      yaml_get_u32_or(node, "cmd_get_gyro", config.cmd_get_gyro));
  config.cmd_get_all = static_cast<uint8_t>(
      yaml_get_u32_or(node, "cmd_get_all", config.cmd_get_all));

  config.quat_scale =
      yaml_get_or<double>(node, "quat_scale", config.quat_scale);
  config.gyro_scale =
      yaml_get_or<double>(node, "gyro_scale", config.gyro_scale);
  config.normalize_quat =
      yaml_get_or<bool>(node, "normalize_quat", config.normalize_quat);

  return config;
}

mjcan::MujocoCanBridge::DeviceConfig load_bridge_device_config(
    const YAML::Node& can,
    const mjcan::SPGMITConfig& spg_defaults,
    const mjcan::E2BoxImuFirmwareConfig& imu_defaults) {
  mjcan::MujocoCanBridge::DeviceConfig device_config;

  const YAML::Node actuators = can["actuators"];
  if (actuators && actuators.IsSequence()) {
    for (const YAML::Node& node : actuators) {
      mjcan::MujocoCanBridge::ActuatorDeviceConfig config;

      config.enabled =
          yaml_get_or<bool>(node, "enabled", config.enabled);

      config.logical_name =
          yaml_get_or<std::string>(node, "name", config.logical_name);

      config.mujoco_joint_name =
          yaml_get_or<std::string>(
              node,
              "mujoco_joint",
              config.mujoco_joint_name);

      config.mujoco_actuator_name =
          yaml_get_or<std::string>(
              node,
              "mujoco_actuator",
              config.mujoco_actuator_name);

      config.motor_id =
          yaml_get_or<int>(node, "motor_id", config.motor_id);

      config.can_id =
          yaml_get_u32_or(node, "can_id", config.can_id);

      config.sign =
          yaml_get_or<double>(node, "sign", config.sign);

      config.offset_rad =
          yaml_get_or<double>(node, "offset_rad", config.offset_rad);

      config.spg_mit_config =
          load_spg_mit_config(node["spg_mit"], spg_defaults);

      device_config.actuators.push_back(std::move(config));
    }
  }

  const YAML::Node imus = can["imus"];
  if (imus && imus.IsSequence()) {
    for (const YAML::Node& node : imus) {
      mjcan::MujocoCanBridge::ImuDeviceConfig config;

      config.enabled =
          yaml_get_or<bool>(node, "enabled", config.enabled);

      config.type =
          yaml_get_or<std::string>(node, "type", config.type);

      config.e2box_config =
          load_e2box_imu_config(node["e2box"], imu_defaults);

      device_config.imus.push_back(std::move(config));
    }
  }

  return device_config;
}

VirtualCanConfig load_virtual_can_config_from_fixed_path() {
  VirtualCanConfig config;

  const std::string config_path = get_fixed_mujoco_can_config_path();

  YAML::Node root;
  try {
    root = YAML::LoadFile(config_path);
  } catch (const YAML::Exception& e) {
    std::printf(
        "[main] Failed to load MuJoCo CAN config '%s': %s. Using defaults.\n",
        config_path.c_str(),
        e.what());
    return config;
  }

  const YAML::Node can = root["mujoco_can"];
  if (!can) {
    std::printf(
        "[main] Config '%s' has no 'mujoco_can' section. Using defaults.\n",
        config_path.c_str());
    return config;
  }

  config.enabled =
      yaml_get_or<bool>(can, "enabled", config.enabled);

  config.base_body_name =
      yaml_get_or<std::string>(can, "base_body_name", config.base_body_name);

  config.motor_id_base =
      yaml_get_or<int>(can, "motor_id_base", config.motor_id_base);

  config.command_timeout_s =
      yaml_get_or<double>(can, "command_timeout_s", config.command_timeout_s);

  const YAML::Node socketcan = can["socketcan"];
  if (socketcan) {
    config.socketcan_enabled =
        yaml_get_or<bool>(
            socketcan,
            "enabled",
            config.socketcan_enabled);

    config.socketcan_interface =
        yaml_get_or<std::string>(
            socketcan,
            "interface",
            config.socketcan_interface);
  }

  config.spg_mit_config =
      load_spg_mit_config(can["spg_mit_defaults"], config.spg_mit_config);

  config.e2box_imu_config =
      load_e2box_imu_config(can["e2box_imu_defaults"], config.e2box_imu_config);

  config.device_config =
      load_bridge_device_config(
          can,
          config.spg_mit_config,
          config.e2box_imu_config);

  std::printf("[main] Loaded MuJoCo CAN config: %s\n", config_path.c_str());
  std::printf(
      "[main]   enabled=%s\n"
      "[main]   socketcan.enabled=%s\n"
      "[main]   socketcan.interface=%s\n"
      "[main]   base_body=%s\n"
      "[main]   motor_id_base=%d\n"
      "[main]   command_timeout_s=%.6f\n"
      "[main]   spg_mit.periodic_feedback=%s\n"
      "[main]   actuators=%zu\n"
      "[main]   imus=%zu\n",
      config.enabled ? "true" : "false",
      config.socketcan_enabled ? "true" : "false",
      config.socketcan_interface.c_str(),
      config.base_body_name.c_str(),
      config.motor_id_base,
      config.command_timeout_s,
      config.spg_mit_config.periodic_feedback ? "true" : "false",
      config.device_config.actuators.size(),
      config.device_config.imus.size());

  return config;
}

// Initializes the virtual CAN bridge and, if enabled, the SocketCAN adapter.
void initialize_virtual_can(const VirtualCanConfig& config) {
  if (!config.enabled) {
    std::printf("[main] MuJoCo CAN bridge disabled by config.\n");
    return;
  }

  g_can_bridge = std::make_unique<mjcan::MujocoCanBridge>();

  g_can_bridge->set_spg_mit_config(config.spg_mit_config);
  g_can_bridge->set_e2box_imu_config(config.e2box_imu_config);
  g_can_bridge->set_device_config(config.device_config);

  if (!config.base_body_name.empty()) {
    g_can_bridge->set_base_body_name(config.base_body_name);
    std::printf(
        "[main] MuJoCo CAN base body: %s\n",
        config.base_body_name.c_str());
  }

  if (config.motor_id_base > 0) {
    g_can_bridge->set_motor_id_base(config.motor_id_base);
    std::printf(
        "[main] MuJoCo CAN motor id base: %d\n",
        config.motor_id_base);
  } else {
    std::printf(
        "[main] Invalid motor_id_base=%d. Keeping bridge default.\n",
        config.motor_id_base);
  }

  g_can_bridge->set_command_timeout(config.command_timeout_s);
  std::printf(
      "[main] MuJoCo CAN command timeout: %.6f s\n",
      config.command_timeout_s);

  if (!config.socketcan_enabled) {
    std::printf("[main] SocketCAN adapter disabled by config.\n");
    return;
  }

  const std::string interface_name = config.socketcan_interface;

  if (interface_name.empty() ||
      interface_name == "none" ||
      interface_name == "off" ||
      interface_name == "OFF") {
    std::printf("[main] SocketCAN adapter disabled.\n");
    return;
  }

  g_socket_can_adapter = std::make_unique<mjcan::SocketCanAdapter>();

  if (!g_socket_can_adapter->open(interface_name)) {
    std::printf(
        "[main] SocketCAN adapter unavailable on '%s'. "
        "Continuing with in-process virtual CAN only.\n",
        interface_name.c_str());
    g_socket_can_adapter.reset();
    return;
  }

  std::printf("[main] SocketCAN adapter connected to %s.\n", interface_name.c_str());
}

void shutdown_virtual_can() {
  if (g_socket_can_adapter) {
    g_socket_can_adapter->close();
    g_socket_can_adapter.reset();
  }

  if (g_can_bridge) {
    g_can_bridge->shutdown();
    g_can_bridge.reset();
  }
}

void reset_virtual_can_model() {
  if (g_can_bridge && g_model && g_data) {
    g_can_bridge->reset_model(g_model, g_data);
  }
}

void poll_virtual_can_rx() {
  if (g_socket_can_adapter && g_can_bridge) {
    g_socket_can_adapter->poll_rx(g_can_bridge.get());
  }
}

void flush_virtual_can_tx() {
  if (g_socket_can_adapter && g_can_bridge) {
    g_socket_can_adapter->flush_tx(g_can_bridge.get());
  }
}

// Returns a MuJoCo warning message when the simulation diverged.
// If auto-reset is disabled, this returns nullptr.
const char* check_divergence(int disable_flags, const mjData* data) {
  if (disable_flags & mjDSBL_AUTORESET) {
    for (mjtWarning warning : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (data->warning[warning].number > 0) {
        return mju_warningText(warning, data->warning[warning].lastinfo);
      }
    }
  }

  return nullptr;
}

// Loads a MuJoCo model from .xml, .mjb, or MuJoCo-supported model formats.
//
// The returned mjModel must be deleted with mj_deleteModel().
mjModel* load_model(const char* file, mj::Simulate& sim) {
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  if (!filename[0]) {
    std::printf("[main] Empty model filename.\n");
    return nullptr;
  }

  char load_error[k_error_length] = "";
  mjModel* new_model = nullptr;

  const auto load_start = mj::Simulate::Clock::now();

  const std::string filename_str(filename);
  std::string extension;

  const std::size_t dot_pos = filename_str.rfind('.');
  if (dot_pos != std::string::npos && dot_pos < filename_str.length() - 1) {
    extension = filename_str.substr(dot_pos);
  }

  std::printf("[main] Loading model: %s\n", filename);

  if (extension == ".mjb") {
    new_model = mj_loadModel(filename, nullptr);

    if (!new_model) {
      mju::strcpy_arr(load_error, "could not load binary model");
    }

  } else if (extension == ".xml") {
    new_model = mj_loadXML(filename, nullptr, load_error, k_error_length);

  } else {
    mjSpec* spec = mj_parse(filename, nullptr, nullptr, load_error, k_error_length);

    if (!spec) {
      mju::strcpy_arr(load_error, "could not parse model");
    } else {
      new_model = mj_compile(spec, nullptr);
      mj_deleteSpec(spec);
    }
  }

  if (load_error[0]) {
    const int error_length = mju::strlen_arr(load_error);
    if (error_length > 0 && load_error[error_length - 1] == '\n') {
      load_error[error_length - 1] = '\0';
    }
  }

  const auto load_interval = mj::Simulate::Clock::now() - load_start;
  const double load_seconds = seconds(load_interval).count();

  if (!new_model) {
    std::printf("[main] Model load failed: %s\n", load_error);
    mju::strcpy_arr(sim.load_error, load_error);
    return nullptr;
  }

  if (load_error[0]) {
    std::printf("[main] Model compiled with warning. Simulation paused:\n  %s\n", load_error);
    sim.run = 0;

  } else if (load_seconds > 0.25) {
    mju::sprintf_arr(load_error, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, load_error);

  std::printf(
      "[main] Model loaded successfully. nq=%d, nv=%d, nu=%d, nbody=%d\n",
      new_model->nq,
      new_model->nv,
      new_model->nu,
      new_model->nbody);

  return new_model;
}

// Replaces the active model/data pair used by the physics thread.
//
// The caller must hold sim.mtx.
void replace_model_and_data(mjModel* new_model, mjData* new_data) {
  mj_deleteData(g_data);
  mj_deleteModel(g_model);

  g_model = new_model;
  g_data = new_data;

  mj_forward(g_model, g_data);
}

// Performs one MuJoCo step and checks for divergence.
bool step_simulation(mj::Simulate& sim) {
  poll_virtual_can_rx();

  if (g_can_bridge) {
    g_can_bridge->before_step(g_model, g_data);
  }

  mj_step(g_model, g_data);

  if (g_can_bridge) {
    g_can_bridge->after_step(g_model, g_data);
  }

  flush_virtual_can_tx();

  const char* message = check_divergence(g_model->opt.disableflags, g_data);

  if (message) {
    sim.run = 0;
    mju::strcpy_arr(sim.load_error, message);
    std::printf("[main] Simulation stopped due to divergence: %s\n", message);
    return false;
  }

  return true;
}

// Updates the model while paused.
//
// This keeps rendering, joint sliders, and control sliders responsive.
// It also keeps pause-state CAN request/response available.
void forward_simulation_while_paused(mj::Simulate& sim) {
  poll_virtual_can_rx();

  if (g_can_bridge) {
    g_can_bridge->before_forward(g_model, g_data);
  }

  mj_forward(g_model, g_data);

  if (g_can_bridge) {
    g_can_bridge->after_forward(g_model, g_data);
  }

  flush_virtual_can_tx();

  if (sim.pause_update) {
    mju_copy(g_data->qacc_warmstart, g_data->qacc, g_model->nv);
  }

  sim.speed_changed = true;
}

// Handles drag-and-drop model loading.
void handle_drop_load_request(mj::Simulate& sim) {
  if (!sim.droploadrequest.load()) {
    return;
  }

  sim.LoadMessage(sim.dropfilename);

  mjModel* new_model = load_model(sim.dropfilename, sim);
  sim.droploadrequest.store(false);

  mjData* new_data = nullptr;
  if (new_model) {
    new_data = mj_makeData(new_model);
  }

  if (!new_data) {
    sim.LoadMessageClear();
    return;
  }

  sim.Load(new_model, new_data, sim.dropfilename);

  const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
  replace_model_and_data(new_model, new_data);

  reset_virtual_can_model();

  std::printf("[main] Drop-loaded model is now active.\n");
}

// Handles model loading requested from the GUI.
void handle_ui_load_request(mj::Simulate& sim) {
  if (!sim.uiloadrequest.load()) {
    return;
  }

  sim.uiloadrequest.fetch_sub(1);
  sim.LoadMessage(sim.filename);

  mjModel* new_model = load_model(sim.filename, sim);

  mjData* new_data = nullptr;
  if (new_model) {
    new_data = mj_makeData(new_model);
  }

  if (!new_data) {
    sim.LoadMessageClear();
    return;
  }

  sim.Load(new_model, new_data, sim.filename);

  const std::unique_lock<std::recursive_mutex> lock(sim.mtx);
  replace_model_and_data(new_model, new_data);

  reset_virtual_can_model();

  std::printf("[main] UI-loaded model is now active.\n");
}

// Runs MuJoCo physics in a background thread while the GUI render loop runs
// in the main thread.
//
// This loop is also the correct place to insert robot hardware wrappers,
// because all mjData control writes and state reads happen under sim.mtx.
void physics_loop(mj::Simulate& sim) {
  std::chrono::time_point<mj::Simulate::Clock> sync_cpu;
  mjtNum sync_sim = 0;

  while (!sim.exitrequest.load()) {
    handle_drop_load_request(sim);
    handle_ui_load_request(sim);

    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

    if (!g_model) {
      continue;
    }

    if (sim.run) {
      bool stepped = false;

      const auto start_cpu = mj::Simulate::Clock::now();

      const auto elapsed_cpu = start_cpu - sync_cpu;
      const double elapsed_sim = g_data->time - sync_sim;

      const double slowdown = 100.0 / sim.percentRealTime[sim.real_time_index];

      const bool misaligned =
          std::abs(seconds(elapsed_cpu).count() / slowdown - elapsed_sim) > k_sync_misalign;

      if (elapsed_sim < 0 ||
          elapsed_cpu.count() < 0 ||
          sync_cpu.time_since_epoch().count() == 0 ||
          misaligned ||
          sim.speed_changed) {
        sync_cpu = start_cpu;
        sync_sim = g_data->time;
        sim.speed_changed = false;

        sim.InjectNoise(sim.key);

        stepped = step_simulation(sim);

      } else {
        bool measured = false;
        const mjtNum previous_sim_time = g_data->time;

        const double refresh_time = k_sim_refresh_fraction / sim.refresh_rate;

        while (seconds((g_data->time - sync_sim) * slowdown) <
                   mj::Simulate::Clock::now() - sync_cpu &&
               mj::Simulate::Clock::now() - start_cpu < seconds(refresh_time)) {
          if (!measured && elapsed_sim) {
            sim.measured_slowdown =
                std::chrono::duration<double>(elapsed_cpu).count() / elapsed_sim;
            measured = true;
          }

          sim.InjectNoise(sim.key);

          if (step_simulation(sim)) {
            stepped = true;
          } else {
            break;
          }

          if (g_data->time < previous_sim_time) {
            break;
          }
        }
      }

      if (stepped) {
        sim.AddToHistory();
      }

    } else {
      forward_simulation_while_paused(sim);
    }
  }
}

// Loads the initial model, creates mjData, attaches them to the GUI,
// then enters the physics loop.
void physics_thread(mj::Simulate* sim, const char* filename) {
  if (filename != nullptr) {
    sim->LoadMessage(filename);

    g_model = load_model(filename, *sim);

    if (g_model) {
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);
      g_data = mj_makeData(g_model);
    }

    if (g_data) {
      sim->Load(g_model, g_data, filename);

      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(g_model, g_data);

      reset_virtual_can_model();

      std::printf("[main] Initial model is active.\n");

    } else {
      sim->LoadMessageClear();
    }
  }

  physics_loop(*sim);

  mj_deleteData(g_data);
  mj_deleteModel(g_model);

  g_data = nullptr;
  g_model = nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  std::printf("[main] Starting MuJoCo simulate runner for Ubuntu PC.\n");
  std::printf("[main] MuJoCo version: %s\n", mj_versionString());

  if (mjVERSION_HEADER != mj_version()) {
    mju_error("MuJoCo header and library versions are different");
  }

  scan_plugin_libraries();

  g_virtual_can_config = load_virtual_can_config_from_fixed_path();
  initialize_virtual_can(g_virtual_can_config);

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam,
      &opt,
      &pert,
      /* is_passive = */ false);

  const char* filename = nullptr;

  if (argc > 1) {
    filename = argv[1];
    std::printf("[main] Initial model file: %s\n", filename);
  } else {
    std::printf("[main] No initial model file provided. Use GUI load or drag-and-drop.\n");
  }

  std::printf("[main] Starting physics thread.\n");
  std::thread physics_thread_handle(&physics_thread, sim.get(), filename);

  std::printf("[main] Starting GUI render loop.\n");
  sim->RenderLoop();

  std::printf("[main] GUI loop exited. Waiting for physics thread.\n");
  physics_thread_handle.join();

  shutdown_virtual_can();

  std::printf("[main] Shutdown complete.\n");

  return EXIT_SUCCESS;
}