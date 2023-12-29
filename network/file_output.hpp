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
#include "output.hpp"

class FileOutput : public Output
{
public:
    FileOutput(VideoOptions const *options);
    ~FileOutput();

    void checkGPSLock();

protected:

    void outputBuffer(void *mem,
                      size_t size,
                      void* prevMem,
                      size_t prevSize,
                      void *exifMem,
                      size_t exifSize,
                      int64_t timestamp_us,
                      uint32_t flags) override;

    struct timeval getAdjustedTime(int64_t timestamp_us);
    std::string currentDate();
    void wrapAndWrite(void *mem, std::string fullFileName, size_t size, void *exifMem, size_t exifSize, int index);
    void writeFile(std::string fullFileName, void *mem, size_t size, void *exifMem, size_t exifSize);
    void collectExistingFilenames();
    void removeLast(size_t numFiles);

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

};
