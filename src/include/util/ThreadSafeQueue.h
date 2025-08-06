#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

/**
 * @class ThreadSafeQueue
 * @brief A generic, thread-safe queue for passing data between threads.
 *
 * This template class wraps a standard std::queue with a mutex and a
 * condition variable to ensure that all operations are safe to call from
 * multiple threads simultaneously.
 */
template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;

  // Disable copy and move semantics
  ThreadSafeQueue(const ThreadSafeQueue&) = delete;
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  /**
   * @brief Pushes a new item onto the queue.
   * This method is thread-safe.
   * @param item The item to be added to the queue.
   */
  void push(const T& item) {
    // Lock the mutex to protect access to the queue
    std::lock_guard<std::mutex> lock(mutex);

    // Add the item to the queue
    if (queue.size() >= maxQueue && maxQueue != -1) {
      return;
    }
    queue.push(item);

    // Notify one waiting thread that an item is available
    condVar.notify_one();
  }

  /**
   * @brief Waits until an item is available and then pops it from the queue.
   * This method is thread-safe and will block until an item is ready.
   * @param[out] item A reference to store the popped item.
   * @return True if an item was successfully popped, false if the queue was
   * shut down while waiting.
   */
  bool waitAndPop(T& item) {
    // Create a unique_lock, which is necessary for condition variables
    std::unique_lock<std::mutex> lock(mutex);

    // Wait until the queue is not empty OR the shutdown flag is set.
    // The lambda function here prevents "spurious wakeups"
    condVar.wait(lock, [this] { return !queue.empty() || isShutdown; });

    // If we woke up because of shutdown and the queue is empty, return false.
    if (isShutdown && queue.empty()) {
      return false;
    }

    // We have the lock and the queue is not empty. Pop the item.
    item = queue.front();
    queue.pop();

    return true;
  }

  /**
   * @brief Waits for an item for a specific duration and then pops it.
   * This method is thread-safe. It will block until an item is ready,
   * the timeout is reached, or the queue is shut down.
   * @param[out] item A reference to store the popped item.
   * @param timeout The maximum time to wait for an item.
   * @return True if an item was successfully popped, false if the wait timed
   * out or the queue was shut down.
   */
  bool waitAndPopWithTimeout(T& item, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex);

    // Wait until the queue is not empty, the shutdown flag is set, or timeout
    if (!condVar.wait_for(
            lock, timeout, [this] { return !queue.empty() || isShutdown; })) {
      // wait_for returned false, meaning it timed out.
      return false;
    }

    // If we woke up because of shutdown and the queue is empty, return false.
    if (isShutdown && queue.empty()) {
      return false;
    }

    // We have the lock and the queue is not empty. Pop the item.
    item = queue.front();
    queue.pop();

    return true;
  }

  /**
   * @brief Gets the current number of items in the queue.
   * This method is thread-safe.
   * @return The number of items currently in the queue.
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex);
    return queue.size();
  }

  /**
   * @brief Shuts down the queue, unblocking any waiting threads.
   *
   * Call this before joining threads to ensure that threads waiting on an
   * empty queue can exit their loop.
   */
  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex);
    isShutdown = true;
    // Notify all waiting threads to wake up and check the shutdown flag.
    condVar.notify_all();
  }

  void setMaxQueue(const int max) { maxQueue = max; }

 private:
  std::queue<T> queue;
  mutable std::mutex mutex;
  std::condition_variable condVar;
  bool isShutdown = false;
  int maxQueue = -1;
};
