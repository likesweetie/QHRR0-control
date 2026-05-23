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

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include "shared_memory.hpp"
#include "shm_utils.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
  #include <sys/errno.h>
  #include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

using Seconds = std::chrono::duration<double>;

constexpr const char* kDefaultShmName = "/shm0";
ShmData* g_shm = nullptr;
std::uint64_t g_last_command_seq = 0;
struct PdConfig {
  double kp{0.0};
  double kd{0.0};
  int num_joint{0};
};
PdConfig g_pd;

const char* ResolvePdConfigPath() {
  const char* env_path = std::getenv("PD_CONFIG_PATH");
  if (env_path && env_path[0]) {
    return env_path;
  }
  return nullptr;
}

const char* ResolvePolicyConfigDir() {
  const char* env_path = std::getenv("POLICY_CONFIG_DIR");
  if (env_path && env_path[0]) {
    return env_path;
  }
  return nullptr;
}

const char* ResolveShmName() {
  const char* env_name = std::getenv("SHM_NAME");
  if (!env_name || !env_name[0]) {
    env_name = std::getenv("SHM_NAME");
  }
  return (env_name && env_name[0]) ? env_name : kDefaultShmName;
}

int ClampCount(int value, std::size_t cap) {
  if (value <= 0) {
    return 0;
  }
  return static_cast<int>(std::min<std::size_t>(static_cast<std::size_t>(value), cap));
}

void AttachSharedMemory() {
  const char* shm_name = ResolveShmName();
  bool created = false;
  g_shm = shm_utils::CreateShm(shm_name, true, &created);
  if (!g_shm) {
    std::cerr << "[simulate] failed to open/create SHM: " << shm_name << '\n';
    return;
  }
  if (created) {
    std::memset(g_shm, 0, sizeof(ShmData));
  }
  std::strncpy(g_shm->name, shm_name, sizeof(g_shm->name) - 1);
  g_shm->name[sizeof(g_shm->name) - 1] = '\0';
  std::cout << "[simulate] SHM attached: " << g_shm->name << '\n';
}

void LoadPdConfig() {
  const char* explicit_path = ResolvePdConfigPath();
  const char* policy_config_dir = ResolvePolicyConfigDir();
  std::string policy_config_pd_path;
  if (policy_config_dir && policy_config_dir[0]) {
    policy_config_pd_path = std::string(policy_config_dir) + "/pd_config.yaml";
  }

  const char* candidates[] = {
      explicit_path,
      policy_config_pd_path.empty() ? nullptr : policy_config_pd_path.c_str(),
      "../config/policy_config/pd_config.yaml",
      "config/policy_config/pd_config.yaml",
  };

  for (const char* path : candidates) {
    if (!path || !path[0]) continue;
    try {
      YAML::Node cfg = YAML::LoadFile(path);
      if (cfg["kp"]) g_pd.kp = cfg["kp"].as<double>();
      if (cfg["kd"]) g_pd.kd = cfg["kd"].as<double>();
      if (cfg["num_joint"]) g_pd.num_joint = cfg["num_joint"].as<int>();
      std::cout << "[simulate] PD config loaded: " << path
                << " kp=" << g_pd.kp
                << " kd=" << g_pd.kd
                << " num_joint=" << g_pd.num_joint << '\n';
      return;
    } catch (const std::exception&) {
      // Try next candidate.
    }
  }

  std::cout << "[simulate] PD config not found. Using defaults"
            << " kp=" << g_pd.kp
            << " kd=" << g_pd.kd
            << " num_joint=" << g_pd.num_joint << '\n';
}

bool GetActuatedQAndQd(const mjModel* model, const mjData* data, int actuator_id,
                       mjtNum* out_q, mjtNum* out_qd) {
  if (model->actuator_trntype[actuator_id] == mjTRN_JOINT) {
    const int joint_id = model->actuator_trnid[2 * actuator_id];
    if (joint_id >= 0 && joint_id < model->njnt) {
      const int qpos_adr = model->jnt_qposadr[joint_id];
      const int dof_adr = model->jnt_dofadr[joint_id];
      // std::cout << "[simulate] joint_id: " << joint_id << ", dof_adr: " << dof_adr << std::endl;
      if (qpos_adr >= 0 && qpos_adr < model->nq &&
          dof_adr >= 0 && dof_adr < model->nv) {
        *out_q = data->qpos[qpos_adr];
        *out_qd = data->qvel[dof_adr];
        return true;
      }
    }
  }

  if (actuator_id < model->nq && actuator_id < model->nv) {
    *out_q = data->qpos[actuator_id];
    *out_qd = data->qvel[actuator_id];
    return true;
  }

  return false;
}

void ExtractBaseQuatAndAngVel(const mjModel* model, const mjData* data,
                              double out_quat[4], double out_ang_vel[3]) {
  out_quat[0] = 1.0;
  out_quat[1] = 0.0;
  out_quat[2] = 0.0;
  out_quat[3] = 0.0;
  out_ang_vel[0] = 0.0;
  out_ang_vel[1] = 0.0;
  out_ang_vel[2] = 0.0;

  // int body_id = mj_name2id(model, mjOBJ_BODY, "IMU");
  // mjtNum vel[6];
  // mj_objectVelocity(model, data, mjOBJ_BODY, body_id, vel, 0);

  // mjtNum* ang_vel = vel;      // [0:3]
  // mjtNum* lin_vel = vel + 3;  // [3:6]

  // const mjtNum* quat = data->xquat + 4 * body_id;

  // out_quat[0] = static_cast<double>(quat[0]);
  // out_quat[1] = static_cast<double>(quat[1]);
  // out_quat[2] = static_cast<double>(quat[2]);
  // out_quat[3] = static_cast<double>(quat[3]);

  // // free joint qvel layout: [vx, vy, vz, wx, wy, wz]
  // out_ang_vel[0] = static_cast<double>(ang_vel[0]);
  // out_ang_vel[1] = static_cast<double>(ang_vel[1]);
  // out_ang_vel[2] = static_cast<double>(ang_vel[2]);

  out_quat[0] = static_cast<double>(data->qpos[3]);
  out_quat[1] = static_cast<double>(data->qpos[4]);
  out_quat[2] = static_cast<double>(data->qpos[5]);
  out_quat[3] = static_cast<double>(data->qpos[6]);

  // free joint qvel layout: [vx, vy, vz, wx, wy, wz]
  out_ang_vel[0] = static_cast<double>(data->qvel[3]);
  out_ang_vel[1] = static_cast<double>(data->qvel[4]);
  out_ang_vel[2] = static_cast<double>(data->qvel[5]);
  return;


  // for (int j = 0; j < model->njnt; ++j) {
  //   if (model->jnt_type[j] != mjJNT_FREE) {
  //     continue;
  //   }

  //   const int qadr = model->jnt_qposadr[j];
  //   const int dadr = model->jnt_dofadr[j];
  //   if (qadr < 0 || dadr < 0) {
  //     continue;
  //   }
  //   // free joint qpos layout: [x, y, z, qw, qx, qy, qz]
  //   if (qadr >= model->nq || dadr >= model->nv) {
  //     continue;
  //   }
  // }
  return;
}

void ApplyControlFromShm(const mjModel* model, mjData* data) {
  if (!g_shm || !model || !data || model->nu <= 0) {
    std::cerr << "[simulate] Invalid input to ApplyControlFromShm\n";
    return;
  }

  const int nu = ClampCount(model->nu, kShmMaxCtrl);
  int controlled = nu;
  if (g_pd.num_joint > 0) {
    controlled = std::min(controlled, g_pd.num_joint);
  }

  for (int i = 0; i < controlled; ++i) {
    mjtNum q = 0;
    mjtNum qd = 0;
    if (!GetActuatedQAndQd(model, data, i, &q, &qd)) {
      data->ctrl[i] = 0;
      std::cerr << "[simulate] Failed to get actuated q and qd for actuator " << i << "\n";
      continue;
    }

    const mjtNum q_target = static_cast<mjtNum>(g_shm->q_target[i]);
    const mjtNum u = static_cast<mjtNum>(g_pd.kp) * (q_target - q) -
                     static_cast<mjtNum>(g_pd.kd) * qd;
    data->ctrl[i] = u;
  }
  for (int i = controlled; i < model->nu; ++i) {
    data->ctrl[i] = 0;
  }

  const std::uint64_t command_seq = g_shm->command_seq;
  g_last_command_seq = command_seq;
  g_shm->applied_command_seq = command_seq;
}

void PublishStateToShm(const mjModel* model, const mjData* data) {
  if (!g_shm || !model || !data) {
    return;
  }

  const int nq = ClampCount(model->nq, kShmMaxQpos);
  const int nv = ClampCount(model->nv, kShmMaxQvel);
  const int nu = ClampCount(model->nu, kShmMaxCtrl);
  const int nsensordata = ClampCount(model->nsensordata, kShmMaxSensorData);

  g_shm->counter++;
  g_shm->sim_time = data->time;
  g_shm->nq = nq;
  g_shm->nv = nv;
  g_shm->nu = nu;
  g_shm->nsensordata = nsensordata;


  
  if (nq > 0) {
    std::memcpy(g_shm->qpos, data->qpos, sizeof(double) * nq);
  }
  if (nv > 0) {
    std::memcpy(g_shm->qvel, data->qvel, sizeof(double) * nv);
  }
  ExtractBaseQuatAndAngVel(model, data, g_shm->quat, g_shm->ang_vel);
  if (nu > 0) {
    std::memcpy(g_shm->ctrl_applied, data->ctrl, sizeof(double) * nu);
  }
  if (nsensordata > 0) {
    std::memcpy(g_shm->sensordata, data->sensordata, sizeof(double) * nsensordata);
  }

  g_shm->state_seq++;
}

//---------------------------------------- plugin handling -----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *=2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char* path = buf.get();
#else
  const char* path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}



// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif


  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}


//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);
        PublishStateToShm(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);
        PublishStateToShm(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            ApplyControlFromShm(m, d);
            mj_step(m, d);
            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              ApplyControlFromShm(m, d);
              mj_step(m, d);
              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          ApplyControlFromShm(m, d);
          mj_forward(m, d);
          sim.speed_changed = true;
        }

        PublishStateToShm(m, d);
      }
    }  // release std::lock_guard<std::mutex>
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);
      PublishStateToShm(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char* title, const char* msg);
static const char* rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char* msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char** argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();
  LoadPdConfig();
  AttachSharedMemory();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );

  const char* filename = nullptr;
  if (argc >  1) {
    filename = argv[1];
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  if (g_shm) {
    shm_utils::CloseShm(g_shm);
    g_shm = nullptr;
  }

  return 0;
}
