#pragma once

#include <mutex>
#include <condition_variable>

class Semaphore
{
public:
    void release()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++count_;
    }

    bool try_acquire()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (count_)
        {
            --count_;
            return true;
        }
        return false;
    }

private:
    std::mutex mutex_;
    unsigned long count_ = 0; // Initialized as locked.
};