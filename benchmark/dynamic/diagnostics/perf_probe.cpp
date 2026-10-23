// Observation-only Linux x86_64 ELF interposition. Original calls are forwarded once.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "my_dwa_controller/controller_diagnostics.hpp"

using Pose = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::Twist;
using Command = geometry_msgs::msg::TwistStamped;
using Compute = Command (*)(void *, const Pose &, const Twist &, void *);
constexpr const char * MY = "_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE";
constexpr const char * DW = "_ZN8dwb_core15DWBLocalPlanner23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE";
constexpr const char * REPORT = "_ZN17my_dwa_controller21ControllerDiagnostics6reportERKN6rclcpp6LoggerERKNS_16CycleDiagnosticsEPKcb";

struct Header { char magic[8]; std::uint64_t next, dropped, record_size, capacity, reserved[3]; };
struct Record {
  std::uint64_t start_ns, end_ns, cpu_ns;
  double ros_stamp, stages[7];
  std::uint32_t controller, threw;
  std::uint64_t processed, valid, scored, reserved;
  double selected_v, selected_w;
};
static_assert(sizeof(Header) == 64, "Header layout");
static_assert(sizeof(Record) == 144, "Record layout");
Header * header = nullptr;
Record * records = nullptr;
thread_local Record * current = nullptr;

std::uint64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::uint64_t cpu_ns() {
  timespec ts{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return std::uint64_t(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}
void initialize() {
  if (header) {return;}
  const char * directory = std::getenv("NAVPERF_DIR");
  if (!directory) {return;}
  const std::string file = std::string(directory) + "/perf_" + std::to_string(getpid()) + ".bin";
  const std::size_t capacity = 16384, bytes = sizeof(Header) + capacity * sizeof(Record);
  const int fd = open(file.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
  if (fd < 0 || ftruncate(fd, bytes)) {std::abort();}
  void * mapped = mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  close(fd);
  if (mapped == MAP_FAILED) {std::abort();}
  header = static_cast<Header *>(mapped);
  std::memcpy(header->magic, "NAVPERF2", 8);
  header->record_size = sizeof(Record); header->capacity = capacity;
  records = reinterpret_cast<Record *>(header + 1);
}
void * original(const char * key, const char * fallback, const char * symbol) {
  const char * path = std::getenv(key);
  void * handle = dlopen(path ? path : fallback, RTLD_LAZY | RTLD_NOLOAD);
  void * result = handle ? dlsym(handle, symbol) : nullptr;
  if (!result) {std::fprintf(stderr, "NAVPERF original symbol unavailable: %s\n", symbol); std::abort();}
  return result;
}
Command measure(Compute function, unsigned controller, void * self,
  const Pose & pose, const Twist & velocity, void * checker) {
  initialize();
  const auto index = header ? __atomic_fetch_add(&header->next, 1ULL, __ATOMIC_RELAXED) : 0;
  Record * record = header && index < header->capacity ? &records[index] : nullptr;
  if (!record && header) {__atomic_fetch_add(&header->dropped, 1ULL, __ATOMIC_RELAXED);}
  Record * previous = current;
  current = record;
  if (record) {
    record->controller = controller;
    for (double & value : record->stages) {value = __builtin_nan("");}
    record->start_ns = steady_ns();
  }
  const auto cpu_start = cpu_ns();
  try {
    Command result = function(self, pose, velocity, checker);
    const auto cpu_end = cpu_ns(), wall_end = steady_ns();
    if (record) {
      record->cpu_ns = cpu_end - cpu_start;
      record->ros_stamp = result.header.stamp.sec + result.header.stamp.nanosec * 1e-9;
      record->selected_v = result.twist.linear.x; record->selected_w = result.twist.angular.z;
      __atomic_store_n(&record->end_ns, wall_end, __ATOMIC_RELEASE);
    }
    current = previous;
    return result;
  } catch (...) {
    if (record) {
      record->cpu_ns = cpu_ns() - cpu_start; record->threw = 1;
      __atomic_store_n(&record->end_ns, steady_ns(), __ATOMIC_RELEASE);
    }
    current = previous;
    throw;
  }
}
extern "C" Command my_compute(void *, const Pose &, const Twist &, void *) asm("_ZN17my_dwa_controller15MyDWAController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" Command my_compute(void * self, const Pose & pose, const Twist & velocity, void * checker) {
  static Compute function = reinterpret_cast<Compute>(original("NAVPERF_MYDWA_LIB", "/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so", MY));
  return measure(function, 1, self, pose, velocity, checker);
}
extern "C" Command dw_compute(void *, const Pose &, const Twist &, void *) asm("_ZN8dwb_core15DWBLocalPlanner23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" Command dw_compute(void * self, const Pose & pose, const Twist & velocity, void * checker) {
  static Compute function = reinterpret_cast<Compute>(original("NAVPERF_DWB_LIB", "/opt/ros/humble/lib/libdwb_core.so", DW));
  return measure(function, 2, self, pose, velocity, checker);
}
using Report = void (*)(void *, const rclcpp::Logger &, const my_dwa_controller::CycleDiagnostics &, const char *, bool);
extern "C" void report(void *, const rclcpp::Logger &, const my_dwa_controller::CycleDiagnostics &, const char *, bool) asm("_ZN17my_dwa_controller21ControllerDiagnostics6reportERKN6rclcpp6LoggerERKNS_16CycleDiagnosticsEPKcb");
extern "C" void report(void * self, const rclcpp::Logger & logger,
  const my_dwa_controller::CycleDiagnostics & cycle, const char * error, bool failed) {
  static Report function = reinterpret_cast<Report>(original("NAVPERF_MYDWA_LIB", "/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so", REPORT));
  if (current) {
    const double stages[] = {double(steady_ns() - current->start_ns) * 1e-6,
      cycle.state_wait_ms, cycle.path_ms, cycle.map_wait_ms, cycle.predict_ms,
      cycle.obstacle_ms, cycle.score_ms};
    std::memcpy(current->stages, stages, sizeof(stages));
    current->processed = cycle.processed; current->valid = cycle.valid; current->scored = cycle.scored;
  }
  function(self, logger, cycle, error, failed);
}