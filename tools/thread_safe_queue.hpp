#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <utility>

#include "tools/logger.hpp"

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
      size_t max_size, std::function<void(void)> full_handler = [] {})
      : max_size_(max_size), full_handler_(full_handler) {}

  void push(const T& value) { (void)try_push(value); }

  bool try_push(const T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (closed_) {
      return false;
    }

    if (queue_.size() >= max_size_) {
      if (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return false;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
    return true;
  }

  bool pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock,
                              [this] { return closed_ || !queue_.empty(); });

    if (queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  T pop() {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  bool try_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool wait_pop(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock,
                              [this] { return closed_ || !queue_.empty(); });

    if (queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  template <typename Rep, typename Period>
  bool wait_pop_for(T& value,
                    const std::chrono::duration<Rep, Period>& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!not_empty_condition_.wait_for(
            lock, timeout, [this] { return closed_ || !queue_.empty(); })) {
      return false;
    }

    if (queue_.empty()) {
      return false;
    }

    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

  bool front(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock,
                              [this] { return closed_ || !queue_.empty(); });

    if (queue_.empty()) {
      return false;
    }

    value = queue_.front();
    return true;
  }

  T front() {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    return queue_.front();
  }

  void back(T& value) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.empty()) {
      tools::logger()->error("Attempt to access the back of an empty queue.");
      return;
    }

    value = queue_.back();
  }

  bool empty() {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void clear() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

  void close() {
    std::unique_lock<std::mutex> lock(mutex_);
    closed_ = true;
    not_empty_condition_.notify_all();
  }

  void reopen() {
    std::unique_lock<std::mutex> lock(mutex_);
    closed_ = false;
  }

  bool closed() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return closed_;
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
  bool closed_{false};
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
