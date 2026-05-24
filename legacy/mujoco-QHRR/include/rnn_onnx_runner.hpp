
#include "onnx_runner.hpp"

class RNNOnnxRunner : public OnnxRunner {
public:
    using OnnxRunner::OnnxRunner;
    virtual ~RNNOnnxRunner() {}


    virtual void compute_policy() override;
    virtual void reset_observations() override;

    int num_layers_{0};
    int hidden_size_{0};
    std::vector<float> h_buf_;   // size = num_layers_ * 1 * hidden_size_


protected:
    // virtual std::vector<float> compute_observation_() override;

private:


};
