#pragma once

#include "actuator_firmware_base.hpp"
#include "spg_firmware.hpp"
#include "imu_firmware_base.hpp"
#include "e2box_imu_firmware.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mjcan {

// -----------------------------------------------------------------------------
// In-process virtual CAN bus
// -----------------------------------------------------------------------------
//
// 현재 버전은 simulation thread 안에서만 접근한다고 가정합니다.
// 외부 socket/IPC/SocketCAN bridge thread가 붙으면 mutex 또는 lock-free queue로
// 교체해야 합니다.
//
class VirtualCanBus {
public:
  void push_host_frame(const CanFrame& frame);
  bool pop_host_frame(CanFrame* frame);

  void push_device_frame(const CanFrame& frame);
  bool pop_device_frame(CanFrame* frame);

  void clear();

  std::size_t host_frame_count() const;
  std::size_t device_frame_count() const;

private:
  std::deque<CanFrame> host_to_device_;
  std::deque<CanFrame> device_to_host_;
};

// -----------------------------------------------------------------------------
// MujocoCanBridge
// -----------------------------------------------------------------------------
//
// main.cc hook 대상:
//
//   before_step()
//     - 현재 mjData snapshot 업데이트
//     - host -> device CAN frame 처리
//     - firmware command를 command buffer에 반영
//     - command buffer를 data->ctrl에 적용
//
//   mj_step()
//
//   after_step()
//     - mjData snapshot 업데이트
//     - firmware feedback/ACK frame을 device -> host queue에 publish
//
class MujocoCanBridge {
public:
  MujocoCanBridge();
  ~MujocoCanBridge();

  MujocoCanBridge(const MujocoCanBridge&) = delete;
  MujocoCanBridge& operator=(const MujocoCanBridge&) = delete;

  void reset_model(const mjModel* model, mjData* data);

  void before_step(const mjModel* model, mjData* data);
  void after_step(const mjModel* model, mjData* data);

  void before_forward(const mjModel* model, mjData* data);
  void after_forward(const mjModel* model, mjData* data);

  void shutdown();

  // ---------------------------------------------------------------------------
  // External endpoint API
  // ---------------------------------------------------------------------------
  //
  // 나중에 C++ test, IPC, local socket, SocketCAN-vcan adapter 등에서 이 API로
  // virtual bus에 frame을 넣고 빼면 됩니다.
  //
  void push_host_frame(const CanFrame& frame);
  bool pop_device_frame(CanFrame* frame);

  std::size_t host_frame_count() const;
  std::size_t device_frame_count() const;

  // command_timeout_s <= 0이면 timeout을 비활성화합니다.
  // 안전 관점에서는 0.05~0.2 s 정도를 권장합니다.
  void set_command_timeout(double command_timeout_s);

  // model마다 base body 이름이 다를 수 있습니다.
  // 예: "base", "trunk", "torso", "pelvis"
  void set_base_body_name(const std::string& body_name);

  // 자동 actuator scan 시 motor id를 motor_id_base + binding_index로 부여합니다.
  // motor_id=0 is valid. 기본값 0이면 CAN ID는 0x140, 0x141, ...
  void set_motor_id_base(int motor_id_base);

  void set_spg_mit_config(const SPGMITConfig& config);
  void set_e2box_imu_config(const E2BoxImuFirmwareConfig& config);

  struct ActuatorDeviceConfig {
    bool enabled = true;

    // Optional user-facing name. If empty, MuJoCo actuator name is used.
    std::string logical_name;

    // At least one of these should be set for explicit YAML mapping.
    // If only joint is set, the bridge searches a joint actuator attached to it.
    std::string mujoco_joint_name;
    std::string mujoco_actuator_name;

    // If can_id is nonzero, it overrides motor_id.
    // Otherwise can_id = 0x140 + motor_id.
    // If motor_id < 0, motor_id = motor_id_base + binding_index.
    int motor_id = -1;
    uint32_t can_id = 0;

    double sign = 1.0;
    double offset_rad = 0.0;

    SPGMITConfig spg_mit_config;
  };

  struct ImuDeviceConfig {
    bool enabled = true;

    // Currently only "e2box" is supported.
    std::string type = "e2box";

    E2BoxImuFirmwareConfig e2box_config;
  };

  struct DeviceConfig {
    // Empty actuators => automatic scan of MuJoCo hinge joint actuators.
    std::vector<ActuatorDeviceConfig> actuators;

    // Empty IMUs => one default E2Box IMU.
    std::vector<ImuDeviceConfig> imus;
  };

  void set_device_config(const DeviceConfig& config);

private:
  struct StateBuffer {
    double sim_time = 0.0;

    int nq = 0;
    int nv = 0;
    int nu = 0;
    int nbody = 0;

    std::vector<double> qpos;
    std::vector<double> qvel;
    std::vector<double> ctrl;
    std::vector<double> actuator_force;

    // MuJoCo xquat convention: w, x, y, z
    std::array<double, 4> base_quat_wxyz{1.0, 0.0, 0.0, 0.0};

    // E2Box firmware에는 host-side에서 최종적으로 받고 싶은 convention으로 넘깁니다.
    // 현재 skeleton은 free joint qvel angular part를 사용합니다.
    // 실제 IMU frame 정확도가 필요하면 gyro sensor를 MJCF에 명시하고 sensordata에서
    // 읽는 방식으로 바꾸는 것을 권장합니다.
    std::array<double, 3> base_gyro_xyz{0.0, 0.0, 0.0};
  };

  struct ActuatorBinding {
    std::string logical_name;
    std::string mujoco_joint_name;
    std::string mujoco_actuator_name;

    int motor_id = -1;
    uint32_t can_id = 0;

    int joint_id = -1;
    int actuator_id = -1;

    int qpos_adr = -1;
    int qvel_adr = -1;
    int ctrl_adr = -1;

    // logical joint convention:
    //   q_logical  = sign * (q_mujoco - offset_rad)
    //   dq_logical = sign * dq_mujoco
    //   tau_mujoco = sign * tau_logical
    double sign = 1.0;
    double offset_rad = 0.0;

    SPGMITConfig spg_mit_config;
  };

  struct CommandBuffer {
    std::vector<ActuatorCommand> actuator_commands;

    void clear() {
      actuator_commands.clear();
    }
  };

  class VirtualActuatorDevice {
  public:
    VirtualActuatorDevice(
        int binding_index,
        ActuatorBinding binding,
        std::unique_ptr<ActuatorFirmwareBase> firmware);

    ~VirtualActuatorDevice();

    VirtualActuatorDevice(const VirtualActuatorDevice&) = delete;
    VirtualActuatorDevice& operator=(const VirtualActuatorDevice&) = delete;

    VirtualActuatorDevice(VirtualActuatorDevice&&) noexcept;
    VirtualActuatorDevice& operator=(VirtualActuatorDevice&&) noexcept;

    bool accepts(const CanFrame& frame) const;

    void reset(double sim_time);

    void on_frame(
        const CanFrame& frame,
        CommandBuffer* command_buffer,
        double sim_time);

    void publish_feedback(
        const StateBuffer& state,
        const CommandBuffer& command_buffer,
        VirtualCanBus* bus,
        double sim_time);

    int binding_index() const {
      return binding_index_;
    }

    const ActuatorBinding& binding() const {
      return binding_;
    }

  private:
    ActuatorFeedbackSample make_feedback_sample(
        const StateBuffer& state,
        const CommandBuffer& command_buffer,
        double sim_time) const;

  private:
    int binding_index_ = -1;
    ActuatorBinding binding_;
    std::unique_ptr<ActuatorFirmwareBase> firmware_;
  };

  class VirtualImuDevice {
  public:
    explicit VirtualImuDevice(std::unique_ptr<ImuFirmwareBase> firmware);

    ~VirtualImuDevice();

    VirtualImuDevice(const VirtualImuDevice&) = delete;
    VirtualImuDevice& operator=(const VirtualImuDevice&) = delete;

    VirtualImuDevice(VirtualImuDevice&&) noexcept;
    VirtualImuDevice& operator=(VirtualImuDevice&&) noexcept;

    bool accepts(const CanFrame& frame) const;

    void reset(double sim_time);

    void on_frame(
        const CanFrame& frame,
        const ImuSample& sample,
        VirtualCanBus* bus,
        double sim_time);

  private:
    std::unique_ptr<ImuFirmwareBase> firmware_;
  };

private:
  void build_default_actuator_bindings(const mjModel* model);
  bool build_actuator_binding_from_config(
      const mjModel* model,
      const ActuatorDeviceConfig& config,
      int binding_index,
      ActuatorBinding* binding) const;
  int find_actuator_id_for_joint(const mjModel* model, int joint_id) const;
  void build_default_devices();

  void update_state_buffer(const mjModel* model, const mjData* data);

  void process_host_frames(double sim_time);
  void apply_commands_to_mujoco(const mjModel* model, mjData* data);
  void publish_device_feedback(double sim_time);

  void resolve_base_body(const mjModel* model);
  void resolve_base_free_joint(const mjModel* model);

  ImuSample make_imu_sample() const;

  double get_logical_position(
      const ActuatorBinding& binding,
      const mjData* data) const;

  double get_logical_velocity(
      const ActuatorBinding& binding,
      const mjData* data) const;

  double compute_logical_torque_command(
      const ActuatorCommand& command,
      double q_logical,
      double dq_logical) const;

  double clamp_ctrl_if_needed(
      const mjModel* model,
      int ctrl_adr,
      double ctrl) const;

private:
  VirtualCanBus bus_;

  StateBuffer state_;
  CommandBuffer command_buffer_;

  std::vector<ActuatorBinding> actuator_bindings_;
  std::vector<VirtualActuatorDevice> actuators_;
  std::vector<VirtualImuDevice> imus_;

  std::string base_body_name_;
  int base_body_id_ = -1;
  int base_free_joint_dof_adr_ = -1;

  int motor_id_base_ = 0;

  double command_timeout_s_ = 0.1;

  SPGMITConfig spg_mit_config_;
  E2BoxImuFirmwareConfig e2box_imu_config_;
  DeviceConfig device_config_;

  bool initialized_ = false;
};

}  // namespace mjcan
