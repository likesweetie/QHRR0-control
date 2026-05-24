#include "task_controller.hpp"
#include "shm_utils.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {
std::atomic<bool> g_run{true};

void HandleSignal(int) {
    g_run.store(false, std::memory_order_relaxed);
}

const char* ResolveShmName() {
    const char* name = std::getenv("SHM_NAME");
    if (!name || !name[0]) {
        name = "/shm0";
    }
    return name;
}
}  // namespace

int main() {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const char* shm_name = ResolveShmName();
    std::cout << "[task_controller] waiting shm: " << shm_name << std::endl;

    ShmData* shm = nullptr;
    while (g_run.load(std::memory_order_relaxed) && !shm) {
        shm = shm_utils::OpenShm(shm_name);
        if (!shm) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!shm) {
        std::cerr << "[task_controller] failed to open shm" << std::endl;
        return 1;
    }

    TaskController controller(50.0, shm);
    controller.initialize();
    controller.start();

    while (g_run.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    controller.stop();
    controller.join();
    shm_utils::CloseShm(shm);
    return 0;
}
