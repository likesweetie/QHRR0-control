#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "shared_memory.hpp"
#include "shm_utils.hpp"


// =====================
// Shared memory (POSIX shm_open + mmap)
// =====================
struct ShmConfig
{
    std::string name;              // e.g. "/robot_shm"
    std::size_t size_bytes{0};     // total bytes
    bool create_if_missing{true};  // O_CREAT
    bool unlink_on_destroy{true}; // shm_unlink on Close/Destructor
};

// =====================
// Process spawn (fork + execve)
// =====================
struct ProcessSpec
{
    std::string name;                   // logical name: "comm", "motor", ...
    std::string exec_path;              // absolute path recommended
    std::vector<std::string> args;      // argv[1..]
    std::map<std::string, std::string> env; // setenv() pairs
    bool create_process_group{true};    // setpgid(0,0) in child
    bool launch_in_terminal{true};
    double wait_after_launch_sec{0.5};  // delay before spawning next process
};

struct ProcessInfo
{
    int pid{-1};

    // Last observed state (queried/updated by caller)
    bool running{false};

    // Exit info after Reap()
    bool exited{false};
    int exit_code{0};
    int term_signal{0};

    int pgid{-1};                  // process group id (보통 pid)
    bool use_process_group{false}; // create_process_group 적용 여부
};

static std::string ShellEscapeSingleQuotes(const std::string& s)
{
    // bash single-quote safe: '  ->  '\'' 
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    return out;
}

static std::string BuildCommandLineForBash(const ProcessSpec& spec)
{
    std::string cmd;
    cmd += "'";
    cmd += ShellEscapeSingleQuotes(spec.exec_path);
    cmd += "'";

    for (const auto& a : spec.args) {
        cmd += " '";
        cmd += ShellEscapeSingleQuotes(a);
        cmd += "'";
    }
    return cmd;
}

class Daemon
{
public:
    Daemon() = default;
    ~Daemon();

    Daemon(const Daemon&) = delete;
    Daemon& operator=(const Daemon&) = delete;

    ShmData* shm;

    // ---- process ----
    // Spawn a process. Returns pid on success, nullopt on failure.
    std::optional<int> Spawn(const ProcessSpec& spec);

    // Send a signal to a spawned process by name. Returns false if not found or kill fails.
    bool Signal(const std::string& name, int sig);

    // Non-blocking reap of any exited children (waitpid(-1, WNOHANG)).
    // Updates internal ProcessInfo and returns list of names that changed to exited.
    std::vector<std::string> Reap();

    // Update 'running' flag using kill(pid,0). (No heartbeat / no health check)
    bool RefreshRunning(const std::string& name);

    // Query info
    std::optional<ProcessInfo> Get(const std::string& name) const;

    // Convenience: terminate all known children (SIGTERM), then optional SIGKILL after timeout is NOT included.
    void TerminateAll(int sig = 15 /*SIGTERM*/);

    int LastErrno() const noexcept { return m_errno; }

private:
    // internal helper to build argv for execve
    static std::vector<char*> BuildArgv_(const std::string& exec, const std::vector<std::string>& args);
    bool PkillExact_(const std::string& comm_name, int sig);

private:

    std::map<std::string, ProcessSpec> m_specs;
    std::map<std::string, ProcessInfo> m_procs;

    int m_errno{0};
};
