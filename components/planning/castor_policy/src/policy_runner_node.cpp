// Onboard runner for the flycrane MARL policy (skeleton).
//
// What it does today:
//   - builds this robot's one-hot agent id from robot.yaml (team.size, team.index)
//   - if a model is mounted, loads it with ONNX Runtime, checks its input width
//     against (obs_frame_base + team.size) * history, and times one inference
//   - runs the control-rate timer and reports its rate and jitter on <ns>/planning/status
//
// Not yet: the observation pipeline (drone + payload state), and any output.
// It publishes no setpoints.

#include <pthread.h>
#include <sched.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "castor_interfaces/msg/policy_status.hpp"
#include "castor_policy/team.hpp"
#include "rclcpp/rclcpp.hpp"

#if CASTOR_HAVE_ORT
#include <onnxruntime_cxx_api.h>
#endif

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class PolicyRunner : public rclcpp::Node {
public:
  PolicyRunner() : Node("policy_runner") {
    team_size_ = static_cast<int>(declare_parameter<int64_t>("team_size", 0));
    team_index_ = static_cast<int>(declare_parameter<int64_t>("team_index", -1));
    model_path_ = declare_parameter<std::string>("model_path", "/var/lib/castor/models/policy.onnx");
    rate_hz_ = declare_parameter<double>("rate_hz", 100.0);
    const int frame_base = static_cast<int>(declare_parameter<int64_t>("obs_frame_base", 42));
    const int history = static_cast<int>(declare_parameter<int64_t>("history", 3));
    run_inference_ = declare_parameter<bool>("run_inference_every_step", false);
    const int fifo_priority = static_cast<int>(declare_parameter<int64_t>("sched_fifo_priority", 0));

    one_hot_ = castor_policy::one_hot(team_size_, team_index_);  // throws on a bad team config
    expected_dim_ = castor_policy::expected_obs_dim(frame_base, team_size_, history);
    obs_.assign(expected_dim_, 0.0f);

    std::string hot;
    for (float x : one_hot_) {
      hot += (hot.empty() ? "" : ", ") + std::to_string(static_cast<int>(x));
    }
    RCLCPP_INFO(get_logger(), "team slot %d of %d, one-hot [%s], expected obs width %zu", team_index_,
                team_size_, hot.c_str(), expected_dim_);

    load_model();
    if (fifo_priority > 0) {
      set_fifo(fifo_priority);
    }

    status_pub_ = create_publisher<castor_interfaces::msg::PolicyStatus>("status", 10);
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    period_ = std::chrono::duration_cast<Clock::duration>(period);
    next_due_ = Clock::now() + period_;
    window_start_ = Clock::now();
    step_timer_ = create_wall_timer(period, [this] { step(); });
    status_timer_ = create_wall_timer(1s, [this] { publish_status(); });
  }

private:
  void load_model() {
#if CASTOR_HAVE_ORT
    if (!std::filesystem::exists(model_path_)) {
      RCLCPP_WARN(get_logger(), "no model at %s; running without one", model_path_.c_str());
      return;
    }
    try {
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "castor_policy");
      Ort::SessionOptions opts;
      // One thread: multi-threaded ORT was slower and spin-burned cores on the
      // Pi 5 benchmark (2026-09-29).
      opts.SetIntraOpNumThreads(1);
      opts.SetInterOpNumThreads(1);
      opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(*env_, model_path_.c_str(), opts);

      Ort::AllocatorWithDefaultOptions alloc;
      input_name_ = session_->GetInputNameAllocated(0, alloc).get();
      output_name_ = session_->GetOutputNameAllocated(0, alloc).get();
      const auto shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      model_dim_ = shape.empty() || shape.back() < 0 ? 0 : static_cast<std::size_t>(shape.back());
      model_loaded_ = true;
      obs_ok_ = model_dim_ == expected_dim_;
      if (!obs_ok_) {
        RCLCPP_ERROR(get_logger(),
                     "model %s takes %zu inputs but this team needs %zu: was it trained for team size %d?",
                     model_path_.c_str(), model_dim_, expected_dim_, team_size_);
        return;
      }
      const auto t0 = Clock::now();
      infer();
      last_inference_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
      RCLCPP_INFO(get_logger(), "loaded %s (%zu inputs), first inference %.3f ms", model_path_.c_str(),
                  model_dim_, last_inference_ms_);
    } catch (const Ort::Exception &e) {
      RCLCPP_ERROR(get_logger(), "failed to load %s: %s", model_path_.c_str(), e.what());
      model_loaded_ = false;
    }
#else
    RCLCPP_WARN(get_logger(), "built without ONNX Runtime; model %s not loaded", model_path_.c_str());
#endif
  }

#if CASTOR_HAVE_ORT
  void infer() {
    const std::array<int64_t, 2> shape{1, static_cast<int64_t>(obs_.size())};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input = Ort::Value::CreateTensor<float>(mem, obs_.data(), obs_.size(), shape.data(), shape.size());
    const char *in_names[] = {input_name_.c_str()};
    const char *out_names[] = {output_name_.c_str()};
    session_->Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
  }
#endif

  void set_fifo(int priority) {
    sched_param sp{};
    sp.sched_priority = priority;
    const int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (err != 0) {
      RCLCPP_WARN(get_logger(), "SCHED_FIFO %d refused (%s); the container needs cap_add SYS_NICE and an rtprio ulimit",
                  priority, std::strerror(err));
    } else {
      RCLCPP_INFO(get_logger(), "running SCHED_FIFO priority %d", priority);
    }
  }

  void step() {
    const auto now = Clock::now();
    const double late_ms = std::chrono::duration<double, std::milli>(now - next_due_).count();
    max_late_ms_ = std::max(max_late_ms_, late_ms);
    next_due_ += period_;
    if (now - next_due_ > 10 * period_) {
      next_due_ = now + period_;  // fell far behind (e.g. suspended); resynchronise
    }
    ++steps_;
#if CASTOR_HAVE_ORT
    if (run_inference_ && model_loaded_ && obs_ok_) {
      const auto t0 = Clock::now();
      try {
        infer();
      } catch (const Ort::Exception &e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
        return;
      }
      last_inference_ms_ = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
      window_ms_.push_back(last_inference_ms_);
    }
#endif
  }

  void publish_status() {
    const auto now = Clock::now();
    const double window = std::chrono::duration<double>(now - window_start_).count();
    castor_interfaces::msg::PolicyStatus s;
    s.header.stamp = get_clock()->now();
    s.team_size = static_cast<uint32_t>(team_size_);
    s.team_index = static_cast<uint32_t>(team_index_);
    s.one_hot = one_hot_;
    s.model_loaded = model_loaded_;
    s.obs_dim_ok = obs_ok_;
    s.expected_obs_dim = static_cast<uint32_t>(expected_dim_);
    s.model_obs_dim = static_cast<uint32_t>(model_dim_);
    s.loop_rate_hz = window > 0 ? steps_ / window : 0.0;
    s.max_jitter_ms = std::max(0.0, max_late_ms_);
    s.last_inference_ms = last_inference_ms_;
    s.inference_samples = static_cast<uint32_t>(window_ms_.size());
    s.inference_mean_ms = s.inference_p50_ms = s.inference_p99_ms = s.inference_max_ms = -1.0;
    if (!window_ms_.empty()) {
      std::sort(window_ms_.begin(), window_ms_.end());
      const auto pct = [this](double q) {
        return window_ms_[std::min(window_ms_.size() - 1, static_cast<std::size_t>(q * window_ms_.size()))];
      };
      double sum = 0.0;
      for (double x : window_ms_) sum += x;
      s.inference_mean_ms = sum / window_ms_.size();
      s.inference_p50_ms = pct(0.50);
      s.inference_p99_ms = pct(0.99);
      s.inference_max_ms = window_ms_.back();
      if (++windows_since_log_ >= 10) {
        RCLCPP_INFO(get_logger(), "inference over 1 s: n=%u mean %.3f p50 %.3f p99 %.3f max %.3f ms, loop %.1f Hz",
                    s.inference_samples, s.inference_mean_ms, s.inference_p50_ms, s.inference_p99_ms,
                    s.inference_max_ms, s.loop_rate_hz);
        windows_since_log_ = 0;
      }
      window_ms_.clear();
    }
    status_pub_->publish(s);
    steps_ = 0;
    max_late_ms_ = 0.0;
    window_start_ = now;
  }

  int team_size_{0}, team_index_{-1};
  std::string model_path_;
  double rate_hz_{100.0};
  bool run_inference_{false};
  std::vector<float> one_hot_, obs_;
  std::size_t expected_dim_{0}, model_dim_{0};
  bool model_loaded_{false}, obs_ok_{false};
  double last_inference_ms_{-1.0};

  Clock::duration period_{};
  Clock::time_point next_due_, window_start_;
  double max_late_ms_{0.0};
  uint64_t steps_{0};
  std::vector<double> window_ms_;
  int windows_since_log_{0};

#if CASTOR_HAVE_ORT
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
#endif

  rclcpp::Publisher<castor_interfaces::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr step_timer_, status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PolicyRunner>());
  } catch (const std::invalid_argument &e) {
    RCLCPP_FATAL(rclcpp::get_logger("policy_runner"), "bad team configuration: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
