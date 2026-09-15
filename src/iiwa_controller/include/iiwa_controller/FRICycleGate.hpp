#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace iiwa_controller
{
// One SDK step -> one read/update/write -> next SDK step. No queued commands,
// no independent ROS timer and no overwrite of an unconsumed setpoint.
class FRICycleGate
{
public:
  void reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = available_ = acquired_ = completed_ = false;
  }
  void stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
    cv_.notify_all();
  }
  bool publishAndWait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) {return false;}
    available_ = true;
    completed_ = false;
    cv_.notify_all();
    if (!cv_.wait_for(lock, timeout_, [&] {return stopped_ || completed_;})) {
      stopped_ = true;
      cv_.notify_all();
    }
    return !stopped_;
  }
  bool waitReady(std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {return stopped_ || available_;}) && !stopped_;
  }
  bool acquire()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (acquired_ || !cv_.wait_for(lock, timeout_, [&] {return stopped_ || available_;}) ||
      stopped_) {return false;}
    available_ = false;
    acquired_ = true;
    return true;
  }
  bool complete()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopped_ || !acquired_) {return false;}
    acquired_ = false;
    completed_ = true;
    cv_.notify_all();
    return true;
  }
private:
  static constexpr std::chrono::milliseconds timeout_{100};
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopped_{true}, available_{false}, acquired_{false}, completed_{false};
};
}  // namespace iiwa_controller
