#include <gtest/gtest.h>
#include "util/ThreadSafeQueue.h"
#include <thread>
#include <chrono>
#include <vector>

TEST(ThreadSafeQueueTest, BasicPushPopSize) {
  ThreadSafeQueue<int> queue;
  EXPECT_EQ(queue.size(), 0u);

  queue.push(42);
  EXPECT_EQ(queue.size(), 1u);

  queue.push(100);
  EXPECT_EQ(queue.size(), 2u);

  int val = 0;
  bool pop_success = queue.waitAndPop(val);
  EXPECT_TRUE(pop_success);
  EXPECT_EQ(val, 42);
  EXPECT_EQ(queue.size(), 1u);

  pop_success = queue.waitAndPop(val);
  EXPECT_TRUE(pop_success);
  EXPECT_EQ(val, 100);
  EXPECT_EQ(queue.size(), 0u);
}

TEST(ThreadSafeQueueTest, MaxQueueLimit) {
  ThreadSafeQueue<int> queue;
  queue.setMaxQueue(2);

  queue.push(1);
  queue.push(2);
  // This push should be ignored because queue size is 2
  queue.push(3);

  EXPECT_EQ(queue.size(), 2u);

  int val = 0;
  EXPECT_TRUE(queue.waitAndPop(val));
  EXPECT_EQ(val, 1);

  EXPECT_TRUE(queue.waitAndPop(val));
  EXPECT_EQ(val, 2);

  // Since it was popped, we can push again
  queue.push(4);
  EXPECT_EQ(queue.size(), 1u);
  EXPECT_TRUE(queue.waitAndPop(val));
  EXPECT_EQ(val, 4);
}

TEST(ThreadSafeQueueTest, WaitAndPopWithTimeout) {
  ThreadSafeQueue<int> queue;

  int val = 0;
  // Queue is empty, should timeout
  auto start = std::chrono::high_resolution_clock::now();
  bool popped = queue.waitAndPopWithTimeout(val, std::chrono::milliseconds(50));
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  EXPECT_FALSE(popped);
  EXPECT_GE(duration, 45); // Should wait at least around 50ms

  // Now test popping when an item is pushed by another thread
  std::thread t([&queue]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    queue.push(99);
  });

  popped = queue.waitAndPopWithTimeout(val, std::chrono::milliseconds(200));
  EXPECT_TRUE(popped);
  EXPECT_EQ(val, 99);

  if (t.joinable()) {
    t.join();
  }
}

TEST(ThreadSafeQueueTest, ShutdownEmptyQueue) {
  ThreadSafeQueue<int> queue;

  std::thread t([&queue]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    queue.shutdown();
  });

  int val = 0;
  bool popped = queue.waitAndPop(val);
  EXPECT_FALSE(popped); // Should return false because queue shutdown

  if (t.joinable()) {
    t.join();
  }

  // Once shutdown, waitAndPop should immediately return false
  popped = queue.waitAndPop(val);
  EXPECT_FALSE(popped);
}

TEST(ThreadSafeQueueTest, ShutdownWithTimeout) {
  ThreadSafeQueue<int> queue;

  std::thread t([&queue]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    queue.shutdown();
  });

  int val = 0;
  bool popped = queue.waitAndPopWithTimeout(val, std::chrono::milliseconds(500));
  EXPECT_FALSE(popped);

  if (t.joinable()) {
    t.join();
  }
}

TEST(ThreadSafeQueueTest, ShutdownWithItemsRemaining) {
  ThreadSafeQueue<int> queue;
  queue.push(10);
  queue.push(20);
  queue.shutdown();

  // We should still be able to pop items that were already in the queue
  int val = 0;
  EXPECT_TRUE(queue.waitAndPop(val));
  EXPECT_EQ(val, 10);

  EXPECT_TRUE(queue.waitAndPopWithTimeout(val, std::chrono::milliseconds(10)));
  EXPECT_EQ(val, 20);

  // Now the queue is empty, so popping should return false
  EXPECT_FALSE(queue.waitAndPop(val));
  EXPECT_FALSE(queue.waitAndPopWithTimeout(val, std::chrono::milliseconds(10)));
}
