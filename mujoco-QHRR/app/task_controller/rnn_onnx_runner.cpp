// RNNOnnxRunner.cpp
#include "rnn_onnx_runner.hpp"

#include <cstring>   // std::memcpy
#include <iostream>
#include <algorithm> // std::clamp


static int64_t get_dim_or_throw(const std::vector<int64_t>& shape, size_t idx, const char* what)
{
    if (idx >= shape.size() || shape[idx] <= 0) {
        throw std::runtime_error(std::string("[RNNOnnxRunner] Invalid/dynamic shape for ") + what);
    }
    return shape[idx];
}

void RNNOnnxRunner::reset_observations()
{

    for (int i = 0; i < num_joint_; i++) {
        dof_pos[i] = 0.0;
        dof_vel[i] = 0.0;
        delta_dof_pos[i] = 0.0;
        actions[i] = 0.0;
        scaled_actions[i] = 0.0;
    }


    if (!h_buf_.empty()) {
        std::fill(h_buf_.begin(), h_buf_.end(), 0.0f);
    }
}

void RNNOnnxRunner::compute_policy()
{

    std::vector<double> obs_vec = compute_observation_();
    std::vector<float> obs_vec_fp32(obs_vec.size(), 0.0f);
    for (size_t i = 0; i < obs_vec.size(); ++i) {
        obs_vec_fp32[i] = static_cast<float>(obs_vec[i]);
    }

    if (h_buf_.empty()) {
        if (session->GetInputCount() < 2) {
            throw std::runtime_error("[RNNOnnxRunner] Model does not have 2 inputs (obs, h_in).");
        }

        Ort::TypeInfo h_typeinfo = session->GetInputTypeInfo(1);
        auto h_shape_info = h_typeinfo.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> h_shape = h_shape_info.GetShape();

        // 기대: [num_layers, 1, hidden_size]
        num_layers_  = static_cast<int>(get_dim_or_throw(h_shape, 0, "h_in.num_layers"));
        int64_t batch = get_dim_or_throw(h_shape, 1, "h_in.batch");
        hidden_size_ = static_cast<int>(get_dim_or_throw(h_shape, 2, "h_in.hidden_size"));

        if (batch != 1) {
            std::cerr << "[RNNOnnxRunner] Warning: h_in batch != 1 (" << batch << "). "
                      << "This runner assumes batch=1.\n";
        }

        h_buf_.assign(static_cast<size_t>(num_layers_ * 1 * hidden_size_), 0.0f);

        std::cout << "[RNNOnnxRunner] Initialized hidden buffer: num_layers="
                  << num_layers_ << ", hidden_size=" << hidden_size_ << std::endl;
    }

    // 3) 입력 텐서 생성
    std::vector<int64_t> obs_shape{1, static_cast<int64_t>(obs_vec.size())};
    std::vector<int64_t> h_shape{static_cast<int64_t>(num_layers_), 1, static_cast<int64_t>(hidden_size_)};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_obs = Ort::Value::CreateTensor<float>(
        memory_info,
        obs_vec_fp32.data(),
        obs_vec_fp32.size(),
        obs_shape.data(),
        obs_shape.size()
    );

    Ort::Value input_h = Ort::Value::CreateTensor<float>(
        memory_info,
        h_buf_.data(),
        h_buf_.size(),
        h_shape.data(),
        h_shape.size()
    );

    std::array<Ort::Value, 2> input_tensors = {std::move(input_obs), std::move(input_h)};

    const size_t out_count = session->GetOutputCount();
    if (out_count < 2) {
        throw std::runtime_error("[RNNOnnxRunner] Model does not have 2 outputs (actions, h_out).");
    }

    auto output_tensors = session->Run(
        Ort::RunOptions{nullptr},
        input_names.data(),
        input_tensors.data(),
        input_tensors.size(),
        output_names.data(),
        out_count
    );

    // 5) actions 읽기
    float* output_actions = output_tensors[num_joint_].GetTensorMutableData<float>();

    std::vector<double> scaled_actions_original(num_joint_, 0.0);
    // if (num_joint_ > 3) {
    //     throw std::runtime_error("[RNNOnnxRunner] num_joint_ too large for local array.");
    // }

    for (int i = 0; i < num_joint_; i++) {
        actions[i] = std::clamp(static_cast<double>(output_actions[i]), -action_clip_, action_clip_);
        scaled_actions_original[i] = actions[i] * action_scale;
    }

    int i=0;
    for (int idx : output_joint_idx_conversion_)
    {
        scaled_actions[i] = scaled_actions_original[idx];
        i++;
    }



    // 6) h_out 저장 (다음 step의 h_in으로 사용)
    float* output_h = output_tensors[1].GetTensorMutableData<float>();
    std::memcpy(h_buf_.data(), output_h, sizeof(float) * h_buf_.size());

}
