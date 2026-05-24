#include "daemon/daemon.hpp"

#include <cerrno>
#include <cstring>
#include <optional>
#include <signal.h>
#include <string>

#include <sys/wait.h>
#include <unistd.h>

// -------------------------
// Daemon
// -------------------------

Daemon::~Daemon()
{
    // 데몬 종료 시 등록된 프로세스들을 종료
    TerminateAll(15 /*SIGTERM*/);
}

std::vector<char*> Daemon::BuildArgv_(const std::string& exec, const std::vector<std::string>& args)
{
    std::vector<char*> argv;
    argv.reserve(1 + args.size() + 1);
    argv.push_back(const_cast<char*>(exec.c_str()));
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    return argv;
}

bool Daemon::PkillExact_(const std::string& comm_name, int sig)
{
    m_errno = 0;
    if (comm_name.empty()) {
        m_errno = EINVAL;
        return false;
    }

    pid_t child = ::fork();
    if (child < 0) {
        m_errno = errno;
        return false;
    }

    if (child == 0) {
        // pkill -<sig> -x <comm_name>
        std::string sig_opt = "-" + std::to_string(sig);
        ::execlp("pkill",
                 "pkill",
                 sig_opt.c_str(),
                 "-x",
                 comm_name.c_str(),
                 static_cast<char*>(nullptr));
        _exit(127);
    }

    int status = 0;
    if (::waitpid(child, &status, 0) < 0) {
        m_errno = errno;
        return false;
    }

    // pkill exit code:
    // 0: match found and signaled
    // 1: no processes matched
    // 2: error
    if (!WIFEXITED(status)) {
        m_errno = EINTR;
        return false;
    }

    const int ec = WEXITSTATUS(status);
    if (ec == 0) return true;
    if (ec == 1) return false;

    m_errno = ECHILD;
    return false;
}

std::optional<int> Daemon::Spawn(const ProcessSpec& spec)
{
    m_errno = 0;

    if (spec.name.empty() || spec.exec_path.empty()) {
        m_errno = EINVAL;
        return std::nullopt;
    }

    // 이름 충돌 방지
    if (m_specs.find(spec.name) != m_specs.end() ||
        m_procs.find(spec.name) != m_procs.end()) {
        m_errno = EEXIST;
        return std::nullopt;
    }

    // spec 저장
    m_specs[spec.name] = spec;

    pid_t pid = ::fork();
    if (pid < 0) {
        m_errno = errno;
        m_specs.erase(spec.name);
        return std::nullopt;
    }

    if (pid == 0) {
        // ----- child -----
        if (spec.create_process_group) {
            (void)::setpgid(0, 0);
        }

        for (const auto& [k, v] : spec.env) ::setenv(k.c_str(), v.c_str(), 1);
        ::setenv("WORKER_NAME", spec.name.c_str(), 1);

        // ★ 새 터미널에 띄우고 싶으면 gnome-terminal을 exec
        if (spec.launch_in_terminal) {
            // BuildCommandLineForBash(spec)는 daemon.hpp에 이미 정의되어 있는 helper 사용
            std::string exiter = "";
            if (std::getenv("KILL_ON_EXIT")) 
            {
                exiter = "; exec bash";
            }
            const std::string cmd = BuildCommandLineForBash(spec) + exiter;
            const std::string term = "/usr/bin/gnome-terminal";
            std::vector<std::string> targs = {"--", "bash", "-lc", cmd};

            auto argv = BuildArgv_(term, targs);
            extern char** environ;
            ::execve(term.c_str(), argv.data(), environ);
            _exit(127);
        }

        // ★ 터미널 없이 그냥 직접 실행
        auto argv = BuildArgv_(spec.exec_path, spec.args);
        extern char** environ;
        ::execve(spec.exec_path.c_str(), argv.data(), environ);
        _exit(127);
    }

    // ----- parent -----
    ProcessInfo info;
    info.pid = static_cast<int>(pid);
    info.running = true;
    info.use_process_group = spec.create_process_group;
    info.pgid = spec.create_process_group ? static_cast<int>(pid) : -1;
    m_procs[spec.name] = info;

    return static_cast<int>(pid);
}

bool Daemon::Signal(const std::string& proc_name, int sig)
{
    auto spec_it = m_specs.find(proc_name);
    if (spec_it == m_specs.end()) {
        return PkillExact_(proc_name, sig);
    }

    const std::string& exec_path = spec_it->second.exec_path;
    const std::size_t slash = exec_path.find_last_of('/');
    const std::string exec_name =
        (slash == std::string::npos) ? exec_path : exec_path.substr(slash + 1);

    if (!exec_name.empty() && PkillExact_(exec_name, sig)) {
        return true;
    }
    return PkillExact_(proc_name, sig);
}

void Daemon::TerminateAll(int sig)
{
    for (auto& [name, spec] : m_specs) {
        const std::string& exec_path = spec.exec_path;
        const std::size_t slash = exec_path.find_last_of('/');
        const std::string exec_name =
            (slash == std::string::npos) ? exec_path : exec_path.substr(slash + 1);

        bool killed = false;
        if (!exec_name.empty()) {
            killed = PkillExact_(exec_name, sig);
        }
        if (!killed) {
            (void)PkillExact_(name, sig);
        }
    }

    for (auto& [_, proc] : m_procs) {
        proc.running = false;
    }
}

std::vector<std::string> Daemon::Reap()
{
    std::vector<std::string> exited_names;
    int status = 0;

    while (true) {
        const pid_t reaped = ::waitpid(-1, &status, WNOHANG);
        if (reaped <= 0) break;

        for (auto& [name, proc] : m_procs) {
            if (proc.pid != reaped) continue;
            proc.running = false;
            proc.exited = true;
            if (WIFEXITED(status)) {
                proc.exit_code = WEXITSTATUS(status);
                proc.term_signal = 0;
            } else if (WIFSIGNALED(status)) {
                proc.exit_code = 0;
                proc.term_signal = WTERMSIG(status);
            }
            exited_names.push_back(name);
            break;
        }
    }

    return exited_names;
}

bool Daemon::RefreshRunning(const std::string& name)
{
    auto it = m_procs.find(name);
    if (it == m_procs.end()) {
        m_errno = ENOENT;
        return false;
    }

    auto& proc = it->second;
    if (proc.pid <= 0) {
        proc.running = false;
        return false;
    }

    const int rc = ::kill(proc.pid, 0);
    proc.running = (rc == 0 || errno == EPERM);
    if (!proc.running && errno != ESRCH) {
        m_errno = errno;
    }
    return proc.running;
}

std::optional<ProcessInfo> Daemon::Get(const std::string& name) const
{
    auto it = m_procs.find(name);
    if (it == m_procs.end()) {
        return std::nullopt;
    }
    return it->second;
}
