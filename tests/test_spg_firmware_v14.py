from __future__ import annotations

import shutil
import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MUJOCO_CAN_DIR = PROJECT_ROOT / "third_party" / "mujoco" / "simulate" / "mujoco_can"


class SPGFirmwareV14Test(unittest.TestCase):
    def test_mujoco_spg_firmware_v14_mit_feedback_and_zero_capture(self) -> None:
        if shutil.which("g++") is None:
            self.skipTest("g++ is required for SPGFirmware behavior test")

        source = r"""
        #include "spg_firmware.hpp"

        #include <cmath>
        #include <cstdint>
        #include <iostream>
        #include <vector>

        namespace {

        constexpr double kPi = 3.14159265358979323846;
        constexpr double kEps = 1e-9;

        mjcan::CanFrame frame_with_opcode(uint8_t opcode) {
          mjcan::CanFrame frame;
          frame.can_id = 0x141;
          frame.dlc = 8;
          frame.data.fill(0);
          frame.data[0] = opcode;
          return frame;
        }

        mjcan::CanFrame mit_control_frame() {
          return frame_with_opcode(0xC0);
        }

        mjcan::CanFrame set_zero_frame(int16_t centideg) {
          mjcan::CanFrame frame = frame_with_opcode(0xC3);
          const uint16_t raw = static_cast<uint16_t>(centideg);
          frame.data[6] = static_cast<uint8_t>(raw & 0xFF);
          frame.data[7] = static_cast<uint8_t>((raw >> 8) & 0xFF);
          return frame;
        }

        int16_t read_i16_le(const mjcan::CanFrame& frame, int offset) {
          const uint16_t raw =
              static_cast<uint16_t>(frame.data[offset]) |
              static_cast<uint16_t>(frame.data[offset + 1] << 8);
          return static_cast<int16_t>(raw);
        }

        mjcan::CanFrame find_status(const std::vector<mjcan::CanFrame>& frames) {
          for (const mjcan::CanFrame& frame : frames) {
            if (frame.dlc == 8 && frame.data[0] == 0xC0) {
              return frame;
            }
          }
          std::cerr << "missing 0xC0 status frame\n";
          std::exit(1);
        }

        void require_near(double actual, double expected, double eps, const char* label) {
          if (std::abs(actual - expected) > eps) {
            std::cerr << label << ": got " << actual << ", expected " << expected << "\n";
            std::exit(1);
          }
        }

        }  // namespace

        int main() {
          mjcan::SPGMITConfig config;
          config.feedback_position_max_rad = 12.56;

          {
            mjcan::SPGFirmware firmware(0x141, config);
            firmware.reset(0.0);

            mjcan::ActuatorFeedbackSample zero_sample;
            zero_sample.position_rad = 0.0;
            firmware.on_can_frame(frame_with_opcode(0xC1), 0.001);
            firmware.make_feedback_frames(zero_sample, 0.002);

            mjcan::ActuatorFeedbackSample pos_sample;
            pos_sample.position_rad = config.feedback_position_max_rad;
            firmware.on_can_frame(mit_control_frame(), 0.003);
            auto frames = firmware.make_feedback_frames(pos_sample, 0.004);
            const int16_t pos_i16 = read_i16_le(find_status(frames), 6);
            if (pos_i16 < 32766) {
              std::cerr << "positive endpoint was not encoded as int16 max: " << pos_i16 << "\n";
              return 1;
            }

            mjcan::ActuatorFeedbackSample neg_sample;
            neg_sample.position_rad = -config.feedback_position_max_rad;
            firmware.on_can_frame(mit_control_frame(), 0.005);
            frames = firmware.make_feedback_frames(neg_sample, 0.006);
            const int16_t neg_i16 = read_i16_le(find_status(frames), 6);
            if (neg_i16 > -32766) {
              std::cerr << "negative endpoint was not encoded as signed int16: " << neg_i16 << "\n";
              return 1;
            }
          }

          {
            mjcan::SPGFirmware firmware(0x141, config);
            firmware.reset(0.0);

            mjcan::ActuatorFeedbackSample sample;
            sample.position_rad = 1.25;

            firmware.on_can_frame(frame_with_opcode(0xC1), 0.001);
            firmware.make_feedback_frames(sample, 0.002);
            require_near(
                firmware.mit_zero_reference_rad(),
                1.25,
                kEps,
                "first 0xC1 auto zero capture");

            firmware.on_can_frame(mit_control_frame(), 0.003);
            auto frames = firmware.make_feedback_frames(sample, 0.004);
            const int16_t p_i16 = read_i16_le(find_status(frames), 6);
            if (std::abs(static_cast<int>(p_i16)) > 1) {
              std::cerr << "auto zero did not make current position near zero: " << p_i16 << "\n";
              return 1;
            }
          }

          {
            mjcan::SPGFirmware firmware(0x141, config);
            firmware.reset(0.0);

            mjcan::ActuatorFeedbackSample sample;
            sample.position_rad = 1.0;

            firmware.on_can_frame(set_zero_frame(3000), 0.001);
            firmware.make_feedback_frames(sample, 0.002);
            const double zero_after_c3 = firmware.mit_zero_reference_rad();
            const double expected_zero = 1.0 - (30.0 * kPi / 180.0);
            require_near(zero_after_c3, expected_zero, kEps, "0xC3 user zero reference");

            firmware.on_can_frame(frame_with_opcode(0xC1), 0.003);
            firmware.make_feedback_frames(sample, 0.004);
            require_near(
                firmware.mit_zero_reference_rad(),
                zero_after_c3,
                kEps,
                "0xC1 must preserve user 0xC3 zero");
          }

          return 0;
        }
        """

        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source_path = tmp_path / "spg_firmware_v14_test.cc"
            binary_path = tmp_path / "spg_firmware_v14_test"
            source_path.write_text(textwrap.dedent(source), encoding="utf-8")

            compile_cmd = [
                "g++",
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-I",
                str(MUJOCO_CAN_DIR),
                str(source_path),
                str(MUJOCO_CAN_DIR / "spg_firmware.cc"),
                str(MUJOCO_CAN_DIR / "actuator_firmware_base.cc"),
                "-o",
                str(binary_path),
            ]
            subprocess.run(compile_cmd, check=True, cwd=PROJECT_ROOT)
            subprocess.run([str(binary_path)], check=True, cwd=PROJECT_ROOT)


if __name__ == "__main__":
    unittest.main()
