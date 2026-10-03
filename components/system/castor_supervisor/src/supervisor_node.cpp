// CASTOR supervisor: a YASMIN state machine for the vehicle's modes, with a
// BehaviorTree.CPP tree ticked inside MISSION for task sequencing.
//
//   BOOT -> WAIT_COMPONENTS -> IDLE -> PREFLIGHT -> READY -> MISSION -> LANDING -> IDLE
//   any state -> EMERGENCY (latched until /team/command "reset" and the vehicle is safe)
//
// The supervisor commands nothing on the vehicle: arming, take-off and landing
// are done by the operator / ground station and PX4. It follows the FC's state,
// tells the team what it is doing (<ns>/system/state), and owns the update gate
// (/run/castor/update_gate) that castor-update.sh checks before restarting the
// stack.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "castor_interfaces/msg/heartbeat.hpp"
#include "castor_interfaces/msg/supervisor_state.hpp"
#include "castor_interfaces/msg/team_command.hpp"
#include "castor_interfaces/msg/vehicle_state.hpp"
#include "castor_supervisor/rules.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

using namespace std::chrono_literals;
using castor_interfaces::msg::Heartbeat;
using castor_interfaces::msg::SupervisorState;
using castor_interfaces::msg::TeamCommand;
using castor_interfaces::msg::VehicleState;
using castor_supervisor::VehicleView;

class Supervisor;

// ---------------------------------------------------------------- BT leaves

// Placeholder: holds for `duration_s` while armed. Commands nothing.
class Hold : public BT::StatefulActionNode {
public:
  Hold(const std::string &name, const BT::NodeConfig &config) : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<double>("duration_s", 5.0, "seconds to hold")}; }
  BT::NodeStatus onStart() override {
    const double d = getInput<double>("duration_s").value_or(5.0);
    deadline_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                       std::chrono::duration<double>(d));
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override {
    return std::chrono::steady_clock::now() >= deadline_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point deadline_;
};

// ---------------------------------------------------------------- the node

class Supervisor : public rclcpp::Node {
public:
  Supervisor() : Node("supervisor") {
    robot_id_ = static_cast<uint32_t>(declare_parameter<int64_t>("robot_id", 0));
    const auto ns = declare_parameter<std::string>("robot_namespace", "");
    fc_enabled_ = declare_parameter<bool>("fc_enabled", false);
    required_ = declare_parameter<std::vector<std::string>>(
      "required_components", std::vector<std::string>{"vehicle", "localization", "planning"});
    heartbeat_timeout_s_ = declare_parameter<double>("heartbeat_timeout_s", 3.0);
    vehicle_timeout_s_ = declare_parameter<double>("vehicle_timeout_s", 1.0);
    preflight_timeout_s_ = declare_parameter<double>("preflight_timeout_s", 30.0);
    gate_path_ = declare_parameter<std::string>("update_gate_path", "/run/castor/update_gate");
    mission_tree_ = declare_parameter<std::string>("mission_tree", "");
    bt_tick_hz_ = declare_parameter<double>("bt_tick_hz", 10.0);

    const std::string root = ns.empty() ? "" : "/" + ns;
    vehicle_sub_ = create_subscription<VehicleState>(root + "/vehicle/state", 10, [this](const VehicleState &m) {
      std::lock_guard<std::mutex> lk(mu_);
      vehicle_ = m;
      vehicle_time_ = now();
    });
    for (const auto &c : required_) {
      hb_subs_.push_back(create_subscription<Heartbeat>(root + "/" + c + "/heartbeat", 10, [this, c](const Heartbeat &) {
        std::lock_guard<std::mutex> lk(mu_);
        heartbeats_[c] = now();
      }));
    }
    cmd_sub_ = create_subscription<TeamCommand>("/team/command", 10, [this](const TeamCommand &m) { on_command(m); });

    state_pub_ = create_publisher<SupervisorState>("state", 10);
    status_timer_ = create_wall_timer(500ms, [this] { publish_state(); });
    gate_timer_ = create_wall_timer(1s, [this] { write_gate(); });

    factory_.registerNodeType<Hold>("Hold");
    factory_.registerSimpleCondition("VehicleArmed", [this](BT::TreeNode &) {
      const auto v = vehicle_view();
      return v.fresh && v.fc_connected && v.armed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });

    RCLCPP_INFO(get_logger(), "robot %u, fc %s, waiting for: %s", robot_id_, fc_enabled_ ? "enabled" : "disabled",
                join(required_).c_str());
  }

  // --- used by the states (state-machine thread) ---

  VehicleView vehicle_view() {
    std::lock_guard<std::mutex> lk(mu_);
    VehicleView v;
    if (vehicle_) {
      v.fresh = (now() - vehicle_time_).seconds() < vehicle_timeout_s_;
      v.fc_connected = vehicle_->fc_connected;
      v.armed = vehicle_->armed;
      v.failsafe = vehicle_->failsafe;
      v.preflight_ok = vehicle_->preflight_checks_pass;
    }
    return v;
  }

  std::vector<std::string> missing_components() {
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<std::string> missing;
    for (const auto &c : required_) {
      auto it = heartbeats_.find(c);
      if (it == heartbeats_.end() || (now() - it->second).seconds() > heartbeat_timeout_s_) {
        missing.push_back(c);
      }
    }
    return missing;
  }

  std::optional<std::string> pop_command() {
    std::lock_guard<std::mutex> lk(mu_);
    if (commands_.empty()) {
      return std::nullopt;
    }
    auto c = commands_.front();
    commands_.pop_front();
    return c;
  }

  void clear_commands() {
    std::lock_guard<std::mutex> lk(mu_);
    commands_.clear();
  }

  void enter(const std::string &state) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (state_ == state) {
        return;
      }
      RCLCPP_INFO(get_logger(), "%s -> %s", state_.c_str(), state.c_str());
      state_ = state;
    }
    publish_state();
    write_gate();
  }

  void set_emergency(const std::string &reason) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!emergency_) {
      RCLCPP_ERROR(get_logger(), "EMERGENCY: %s", reason.c_str());
    }
    emergency_ = true;
    emergency_reason_ = reason;
  }

  void clear_emergency() {
    std::lock_guard<std::mutex> lk(mu_);
    emergency_ = false;
    emergency_reason_.clear();
  }

  bool fc_enabled() const { return fc_enabled_; }
  double preflight_timeout_s() const { return preflight_timeout_s_; }
  double bt_tick_hz() const { return bt_tick_hz_; }

  std::optional<BT::Tree> make_mission_tree() {
    if (mission_tree_.empty() || !std::filesystem::exists(mission_tree_)) {
      RCLCPP_ERROR(get_logger(), "mission tree '%s' not found", mission_tree_.c_str());
      return std::nullopt;
    }
    try {
      return factory_.createTreeFromFile(mission_tree_);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "mission tree %s failed to load: %s", mission_tree_.c_str(), e.what());
      return std::nullopt;
    }
  }

  bool running() const { return rclcpp::ok() && !stopping_; }
  void stop() { stopping_ = true; }

private:
  static std::string join(const std::vector<std::string> &v) {
    std::string s;
    for (const auto &x : v) {
      s += (s.empty() ? "" : ", ") + x;
    }
    return s.empty() ? "nothing" : s;
  }

  void on_command(const TeamCommand &m) {
    if (!m.robot_ids.empty() &&
        std::find(m.robot_ids.begin(), m.robot_ids.end(), robot_id_) == m.robot_ids.end()) {
      return;  // addressed to other robots
    }
    std::lock_guard<std::mutex> lk(mu_);
    commands_.push_back(m.command);
    while (commands_.size() > 8) {
      commands_.pop_front();
    }
  }

  void publish_state() {
    const auto missing = missing_components();
    const auto v = vehicle_view();
    SupervisorState s;
    s.header.stamp = now();
    {
      std::lock_guard<std::mutex> lk(mu_);
      s.state = state_;
      s.emergency = emergency_;
      s.emergency_reason = emergency_reason_;
      s.update_allowed = castor_supervisor::update_allowed(state_, fc_enabled_, v);
    }
    for (const auto &c : required_) {
      if (std::find(missing.begin(), missing.end(), c) == missing.end()) {
        s.components_up.push_back(c);
      }
    }
    state_pub_->publish(s);
  }

  // "allow|deny <unix seconds> <state>", written atomically. castor-update.sh
  // only proceeds on a fresh "allow".
  void write_gate() {
    std::string state;
    {
      std::lock_guard<std::mutex> lk(mu_);
      state = state_;
    }
    const bool allow = castor_supervisor::update_allowed(state, fc_enabled_, vehicle_view());
    const auto epoch = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now().time_since_epoch()).count();
    // Called from the gate timer and from state transitions on the FSM thread.
    std::lock_guard<std::mutex> gate_lock(gate_mu_);
    const std::string tmp = gate_path_ + ".tmp";
    {
      std::ofstream f(tmp, std::ios::trunc);
      f << (allow ? "allow" : "deny") << ' ' << epoch << ' ' << state << '\n';
      if (!f) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "cannot write update gate %s", tmp.c_str());
        return;
      }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, gate_path_, ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "cannot replace update gate %s: %s",
                           gate_path_.c_str(), ec.message().c_str());
    }
  }

  uint32_t robot_id_{0};
  bool fc_enabled_{false};
  std::vector<std::string> required_;
  double heartbeat_timeout_s_{3.0}, vehicle_timeout_s_{1.0}, preflight_timeout_s_{30.0}, bt_tick_hz_{10.0};
  std::string gate_path_, mission_tree_;

  std::mutex mu_, gate_mu_;
  std::optional<VehicleState> vehicle_;
  rclcpp::Time vehicle_time_;
  std::map<std::string, rclcpp::Time> heartbeats_;
  std::deque<std::string> commands_;
  std::string state_{"BOOT"};
  bool emergency_{false};
  std::string emergency_reason_;
  std::atomic<bool> stopping_{false};

  BT::BehaviorTreeFactory factory_;

  rclcpp::Subscription<VehicleState>::SharedPtr vehicle_sub_;
  std::vector<rclcpp::Subscription<Heartbeat>::SharedPtr> hb_subs_;
  rclcpp::Subscription<TeamCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<SupervisorState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_, gate_timer_;
};

// ---------------------------------------------------------------- states

// Every state polls at 20 Hz and leaves on its own outcomes, on "emergency"
// (unless it is EMERGENCY), or on "shutdown" when the node is stopping.
class PollingState : public yasmin::State {
public:
  PollingState(Supervisor *sup, std::string name, yasmin::Outcomes outcomes, bool armed_state = false)
    : yasmin::State(with_common(std::move(outcomes))), sup_(sup), name_(std::move(name)), armed_state_(armed_state) {}

  std::string execute(yasmin::Blackboard::SharedPtr) override {
    sup_->enter(name_);
    on_enter();
    while (sup_->running()) {
      std::string reason;
      if (name_ != "EMERGENCY" &&
          castor_supervisor::emergency(sup_->fc_enabled(), armed_state_, sup_->vehicle_view(), &reason)) {
        sup_->set_emergency(reason);
        on_exit();
        return "emergency";
      }
      if (auto outcome = poll()) {
        on_exit();
        return *outcome;
      }
      std::this_thread::sleep_for(50ms);
    }
    on_exit();
    return "shutdown";
  }

protected:
  virtual std::optional<std::string> poll() = 0;
  virtual void on_enter() {}
  virtual void on_exit() {}
  Supervisor *sup_;

private:
  static yasmin::Outcomes with_common(yasmin::Outcomes o) {
    o.insert("emergency");
    o.insert("shutdown");
    return o;
  }
  std::string name_;
  bool armed_state_;
};

class Boot : public PollingState {
public:
  explicit Boot(Supervisor *s) : PollingState(s, "BOOT", {"done"}) {}
  void on_enter() override { start_ = std::chrono::steady_clock::now(); }
  std::optional<std::string> poll() override {
    return std::chrono::steady_clock::now() - start_ > 1s ? std::optional<std::string>("done") : std::nullopt;
  }

private:
  std::chrono::steady_clock::time_point start_;
};

class WaitComponents : public PollingState {
public:
  explicit WaitComponents(Supervisor *s) : PollingState(s, "WAIT_COMPONENTS", {"ready"}) {}
  std::optional<std::string> poll() override {
    return sup_->missing_components().empty() ? std::optional<std::string>("ready") : std::nullopt;
  }
};

class Idle : public PollingState {
public:
  explicit Idle(Supervisor *s) : PollingState(s, "IDLE", {"preflight", "components_lost"}) {}
  void on_enter() override { sup_->clear_commands(); }
  std::optional<std::string> poll() override {
    if (!sup_->missing_components().empty()) {
      return "components_lost";
    }
    if (auto c = sup_->pop_command(); c && *c == TeamCommand::PREFLIGHT) {
      return "preflight";
    }
    return std::nullopt;
  }
};

class Preflight : public PollingState {
public:
  explicit Preflight(Supervisor *s) : PollingState(s, "PREFLIGHT", {"ready", "failed", "abort"}) {}
  void on_enter() override { start_ = std::chrono::steady_clock::now(); }
  std::optional<std::string> poll() override {
    if (auto c = sup_->pop_command(); c && *c == TeamCommand::ABORT) {
      return "abort";
    }
    const auto v = sup_->vehicle_view();
    const bool vehicle_ok = !sup_->fc_enabled() || (v.fresh && v.fc_connected && v.preflight_ok && !v.armed);
    if (vehicle_ok && sup_->missing_components().empty()) {
      return "ready";
    }
    if (std::chrono::steady_clock::now() - start_ > std::chrono::duration<double>(sup_->preflight_timeout_s())) {
      return "failed";
    }
    return std::nullopt;
  }

private:
  std::chrono::steady_clock::time_point start_;
};

class Ready : public PollingState {
public:
  explicit Ready(Supervisor *s) : PollingState(s, "READY", {"armed", "abort"}) {}
  std::optional<std::string> poll() override {
    if (auto c = sup_->pop_command(); c && *c == TeamCommand::ABORT) {
      return "abort";
    }
    if (!sup_->missing_components().empty()) {
      return "abort";
    }
    const auto v = sup_->vehicle_view();
    if (v.fresh && v.fc_connected && v.armed) {
      return "armed";  // armed by the operator; the supervisor never arms
    }
    return std::nullopt;
  }
};

class Mission : public PollingState {
public:
  explicit Mission(Supervisor *s) : PollingState(s, "MISSION", {"done", "failed", "disarmed"}, true) {}
  void on_enter() override {
    tree_ = sup_->make_mission_tree();
    period_ = std::chrono::duration<double>(1.0 / sup_->bt_tick_hz());
    last_tick_ = std::chrono::steady_clock::now() - std::chrono::duration_cast<std::chrono::steady_clock::duration>(period_);
  }
  void on_exit() override {
    if (tree_) {
      tree_->haltTree();
    }
    tree_.reset();
  }
  std::optional<std::string> poll() override {
    const auto v = sup_->vehicle_view();
    if (v.fresh && v.fc_connected && !v.armed) {
      return "disarmed";
    }
    if (!tree_) {
      return "failed";
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_tick_ < period_) {
      return std::nullopt;
    }
    last_tick_ = now;
    switch (tree_->tickOnce()) {
      case BT::NodeStatus::SUCCESS: return "done";
      case BT::NodeStatus::FAILURE: return "failed";
      default: return std::nullopt;
    }
  }

private:
  std::optional<BT::Tree> tree_;
  std::chrono::duration<double> period_{0.1};
  std::chrono::steady_clock::time_point last_tick_;
};

class Landing : public PollingState {
public:
  explicit Landing(Supervisor *s) : PollingState(s, "LANDING", {"landed"}, true) {}
  std::optional<std::string> poll() override {
    const auto v = sup_->vehicle_view();
    return v.fresh && v.fc_connected && !v.armed ? std::optional<std::string>("landed") : std::nullopt;
  }
};

class Emergency : public PollingState {
public:
  explicit Emergency(Supervisor *s) : PollingState(s, "EMERGENCY", {"reset"}) {}
  std::optional<std::string> poll() override {
    auto c = sup_->pop_command();
    if (c && *c == TeamCommand::RESET) {
      if (castor_supervisor::emergency_clearable(sup_->fc_enabled(), sup_->vehicle_view())) {
        sup_->clear_emergency();
        return "reset";
      }
      RCLCPP_WARN(sup_->get_logger(), "reset refused: vehicle is not disarmed and out of failsafe");
    }
    return std::nullopt;
  }
};

// ---------------------------------------------------------------- main

std::shared_ptr<yasmin::StateMachine> build_state_machine(Supervisor *s) {
  auto sm = std::make_shared<yasmin::StateMachine>(yasmin::Outcomes{"shutdown"});
  const auto with = [](std::map<std::string, std::string> t) {
    t.emplace("emergency", "EMERGENCY");
    t.emplace("shutdown", "shutdown");
    return yasmin::Transitions(t.begin(), t.end());
  };
  sm->add_state("BOOT", std::make_shared<Boot>(s), with({{"done", "WAIT_COMPONENTS"}}));
  sm->add_state("WAIT_COMPONENTS", std::make_shared<WaitComponents>(s), with({{"ready", "IDLE"}}));
  sm->add_state("IDLE", std::make_shared<Idle>(s),
                with({{"preflight", "PREFLIGHT"}, {"components_lost", "WAIT_COMPONENTS"}}));
  sm->add_state("PREFLIGHT", std::make_shared<Preflight>(s),
                with({{"ready", "READY"}, {"failed", "IDLE"}, {"abort", "IDLE"}}));
  sm->add_state("READY", std::make_shared<Ready>(s), with({{"armed", "MISSION"}, {"abort", "IDLE"}}));
  sm->add_state("MISSION", std::make_shared<Mission>(s),
                with({{"done", "LANDING"}, {"failed", "LANDING"}, {"disarmed", "IDLE"}}));
  sm->add_state("LANDING", std::make_shared<Landing>(s), with({{"landed", "IDLE"}}));
  // EMERGENCY never returns "emergency", but every outcome needs a transition.
  sm->add_state("EMERGENCY", std::make_shared<Emergency>(s), with({{"reset", "IDLE"}}));
  sm->set_start_state("BOOT");
  return sm;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Supervisor>();
  auto sm = build_state_machine(node.get());

  std::thread fsm([&] {
    try {
      const auto outcome = sm->execute();
      RCLCPP_INFO(node->get_logger(), "state machine finished: %s", outcome.c_str());
    } catch (const std::exception &e) {
      RCLCPP_FATAL(node->get_logger(), "state machine failed: %s", e.what());
      rclcpp::shutdown();
    }
  });

  rclcpp::spin(node);
  node->stop();
  fsm.join();
  rclcpp::shutdown();
  return 0;
}
