/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Chris Niessl, Hellbender Inc.
 *
 * file_output.hpp - send directly to file
 */

#pragma once

#include <netinet/in.h>
#include <sys/un.h>
#include <sys/time.h>

#include <queue>
#include <mutex>
#include <thread>
#include "output.hpp"
#include <condition_variable>
#include <semaphore>

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

struct MemoryWrapper {
    public:
        MemoryWrapper(void *mem, size_t memSize, void *exifMem, size_t exifMemSize)
        :memSize(memSize), exifMemSize(exifMemSize)
        {
            this->mem = malloc(memSize);
            if (this->mem == nullptr) {
                throw std::bad_alloc();
            }
            this->exifMem = malloc(exifMemSize);
            if (this->exifMem == nullptr) {
                this->~MemoryWrapper();
                throw std::bad_alloc();
            }
            memcpy(this->mem, mem, memSize);
            memcpy(this->exifMem, exifMem, exifMemSize);
        }
        ~MemoryWrapper()
        {
            free(this->mem);
            free(this->exifMem);
        }

        void *mem;
        size_t memSize;
        void *exifMem;
        size_t exifMemSize;
};

struct Work {
    timeval time;
    std::filesystem::path filePath;
    MemoryWrapper memWrapper;
    int index;
};

using namespace std::chrono_literals;
class Semaphore {
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned long count_ = 0; // Initialized as locked.


public:
    void release() {
        std::lock_guard<decltype(mutex_)> lock(mutex_);
        ++count_;
        condition_.notify_one();
    }

    bool acquire() {
        std::unique_lock<decltype(mutex_)> lock(mutex_);
        // while(!count_) // Handle spurious wake-ups.
        //     condition_.wait(lock);
        condition_.wait_for(lock, 1ms);
        if (!count_) {
            return false; 
        }
        --count_;
        return true;
    }

    bool try_acquire() {
        std::lock_guard<decltype(mutex_)> lock(mutex_);
        if(count_) {
            --count_;
            return true;
        }
        return false;
    }
};

class FileOutput : public Output
{
public:
    FileOutput(VideoOptions const *options);
    ~FileOutput();

    void checkGPSLock();

protected:

    void usbThreadLoop();

    void outputBuffer(void *mem,
                      size_t size,
                      void* prevMem,
                      size_t prevSize,
                      void *exifMem,
                      size_t exifSize,
                      int64_t timestamp_us,
                      uint32_t flags) override;

    struct timeval getAdjustedTime(int64_t timestamp_us);
    void wrapAndWrite(void *mem, std::string fullFileName, size_t size, void *exifMem, size_t exifSize, int index);
    void writeFile(std::string fullFileName, void *mem, size_t size, void *exifMem, size_t exifSize);
    void collectExistingFilenames();
    void removeLast(size_t numFiles);
    std::string currentDate();

private:

    bool verbose_;
    bool gpsLockAcq_;
    bool writeTempFile_;
    std::string latestDir_;
    std::string latestFileName_;
    std::string dir2K_;
    std::string dir4K_;
    std::filesystem::path dirUSB_;
    std::string prefix_;
    std::string postfix_;
    struct timeval baseTime_;
    std::mutex localtimeMutex_;
    std::mutex fileQueueMutex_;
    std::deque<std::filesystem::path> filesStoredOnUSB_;
    uint32_t minUSBFreeSpace_ = 0;
    uint32_t maxUSBFiles_ = 0;
    MessageQueue<Work> filesToTransfer_;

    std::thread usbThread_;
    Semaphore waitForUSB_;
};
