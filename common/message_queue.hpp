#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class MessageQueue
{
public:
    template <typename U>
    void Post(U &&msg)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(std::forward<U>(msg));
        cond_.notify_one();
    }
    T Wait()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this] { return !queue_.empty(); });
        T msg = std::move(queue_.front());
        queue_.pop();
        return msg;
    }
    void Clear()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_ = {};
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
};