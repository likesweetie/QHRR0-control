#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <pthread.h>
#include "onnx_runner.hpp"
#include "shared_memory.hpp"

class TaskController {
public:
    TaskController(double control_frequency, ShmData* shared_memory);
    ~TaskController();

    void initialize();
    void start();
    void join();
    void stop();

private:
    struct RunnerSession {
        std::string name;
        std::string directory;
        std::unique_ptr<OnnxRunner> runner;
    };

    static void* thread_function_wrapper(void*);
    void* loop();

    enum TaskMode {
        TASK_IDLE = 0,
        TASK_INIT,
        TASK_RL,
        TASK_RECOVERY,
    };

    std::vector<RunnerSession> sessions_;
    RunnerSession* active_session_;

    OnnxRunner* runner_;
    OnnxRunner* cur_runner_;
    
    ShmData* shared_memory_;
    pthread_t thread_;

    double control_frequency_;

    int state_;
    std::atomic<bool> running_;
    std::uint64_t last_state_seq_;

    std::vector<double> default_joint_angle_;
    std::vector<int> home_idx_conversion;
    std::string joint_idx_style = "rainbow";
};
