#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

class ThreadPool {
public:
    explicit ThreadPool(size_t num_threads = std::thread::hardware_concurrency());
    ~ThreadPool();

    void enqueue(std::function<void()> task);
    void wait();

private:
    // Vector to store worker threads
    std::vector<std::thread> threads_;

    // Queue of tasks
    std::queue<std::function<void()>> tasks_;

    // Mutex to synchronize access to shared data
    std::mutex queue_mutex_;

    // Condition variable to signal changes in the state of the tasks queue
    std::condition_variable cv_;

    // Flag to indicate whether the thread pool should stop or not
    bool stop_ = false;

    // Counter for tracking the number of enqueued tasks
    size_t tasks_count_ = 0;

    // Additional condition variable for waiting all tasks to be completed
    std::condition_variable all_tasks_done_cv_;
};

#endif // THREAD_POOL_H
