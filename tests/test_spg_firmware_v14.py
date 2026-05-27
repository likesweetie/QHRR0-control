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
