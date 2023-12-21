/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Chris Niessl, Hellbender Inc.
 *
 * file_output.cpp - send directly to file.
 */
#include <iostream>
#include <iomanip>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>
#include <boost/filesystem.hpp>
#include <fmt/core.h>


#include "file_output.hpp"

static const unsigned char exif_header[] = {0xff, 0xd8, 0xff, 0xe1};


const std::string PARTITION_USB="/media/usb0";
// const int MIN_FREE_SPACE_USB=100000000;
// const uintmax_t MIN_FREE_SPACE_USB=124039233536L - 5000L;
// const int MAX_FILES=100;


FileOutput::FileOutput(VideoOptions const *options) : Output(options) {
    dir2K_ = options_->downsampleStreamDir;
    dir4K_ = options_->output;
    dirUSB_ = options_->output_2nd;

//    dir4K_ = options_->output;
//    dirUSB_ = options_->output_2nd;
//    directory_[2] = previewDir_;


    std::cerr << "Initializing sizes.." << std::endl;

    verbose_ = options_->verbose;
    prefix_ = options_->prefix;
    writeTempFile_ = options_->writeTmp;
    maxUSBUsage_ = options_->max_usb_usage;
    maxUSBFiles_ = options_->max_usb_files;

    //TODO - Assume jpeg format for now. Otherwise extract
    postfix_ = ".jpg";
    gpsLockAcq_ = false;

    //Check if directories exist, and if not then ignore them
    if (!boost::filesystem::exists(dir4K_)) {
        dir4K_ = "";
    }
    if (!boost::filesystem::exists(dirUSB_)) {
        dirUSB_ = "";
    }
    if (!boost::filesystem::exists(dir2K_)) {
        dir2K_ = "";
    }

    //Use stringstream to create latest file for picture
    std::stringstream fileNameGenerator;
    fileNameGenerator << latestDir_;
    fileNameGenerator << "latest.txt";
    latestFileName_ = fileNameGenerator.str();

    collectExistingFilenames();
}

FileOutput::~FileOutput() {
}

void FileOutput::collectExistingFilenames() {
    fileQueueMutex_.lock();

    for (const auto &entry : std::filesystem::directory_iterator(dirUSB_)) {
        filesStoredOnUSB_.push_back(entry.path());
    }
    std::sort(filesStoredOnUSB_.begin(), filesStoredOnUSB_.end());

    std::cerr << "files stored: " << filesStoredOnUSB_.size() << std::endl;
    if (filesStoredOnUSB_.size() > 0) {
        std::cerr << filesStoredOnUSB_.front() << std::endl;
    }

    fileQueueMutex_.unlock();
}

void FileOutput::removeLast(size_t numFiles) {
    fileQueueMutex_.lock();

    numFiles = std::min(numFiles, filesStoredOnUSB_.size());
    while (numFiles) {
        std::string fileName = filesStoredOnUSB_.front();
        filesStoredOnUSB_.pop_front();

        int status = std::remove(fileName.c_str());
        std::cerr << "removed " << fileName << std::endl;
        if (status != 0) {
            std::cerr << "Failed to delete file " << fileName << std::endl;
        }

        numFiles--;
    }

    fileQueueMutex_.unlock();
}

void FileOutput::outputBuffer(void *mem,
                              size_t size,
                              void *prevMem,
                              size_t prevSize,
                              void *exifMem,
                              size_t exifSize,
                              int64_t timestamp_us,
                              uint32_t /*flags*/) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    static int32_t frameNumTrun = 0;

    std::cerr << "filesStored: " << filesStoredOnUSB_.size() << std::endl;

    std::filesystem::space_info space = std::filesystem::space(PARTITION_USB);
    std::cerr << "space free: " << space.free << std::endl;
    

    std::cerr << "usb files: " << maxUSBFiles_ << std::endl;
    std::cerr << "usb usage: " << maxUSBUsage_ << std::endl;

    if ((maxUSBUsage_ > 0 && space.free < maxUSBUsage_)
        || (maxUSBFiles_ > 0 && filesStoredOnUSB_.size() > maxUSBFiles_)) {
        if (options_->verbose) {
            std::cerr << "Out of space, removing older image files." << std::endl;
        }
        removeLast(5);
    }

    try {
        tv = getAdjustedTime(timestamp_us);
    }
    catch (std::exception const &e) {
        std::cerr << "Time recording issues" << std::endl;
    }
    std::string primFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", dir4K_, prefix_, tv.tv_sec,
                                           tv.tv_usec, postfix_);
    if (!dir4K_.empty() && !options_->skip_4k) {
        wrapAndWrite(mem, primFileName, size, exifMem, exifSize, 0);
    }

    if (!dirUSB_.empty()) {
        std::string secFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", dirUSB_, prefix_, tv.tv_sec,
                                              tv.tv_usec, postfix_);
        if (!options_->skip_4k) {
            wrapAndWrite(mem, secFileName, size, exifMem, exifSize, 1);

            fileQueueMutex_.lock();
            filesStoredOnUSB_.push_back(secFileName);
            fileQueueMutex_.unlock();
        } else {
            if (!options_->skip_2k) {
                wrapAndWrite(prevMem, secFileName, prevSize, exifMem, exifSize, 1);
                fileQueueMutex_.lock();
                filesStoredOnUSB_.push_back(secFileName);
                fileQueueMutex_.unlock();
            }
        }
    }

    if (!dir2K_.empty() && !options_->skip_2k) {
        std::string prevFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", dir2K_, prefix_, tv.tv_sec,
                                               tv.tv_usec, postfix_);
        wrapAndWrite(prevMem, prevFileName, prevSize, exifMem, exifSize, 2);
    }
    //After files are written. Update the latest marker
    {
        int fd, ret;
        size_t latestSize = primFileName.size();
        fd = open(latestFileName_.c_str(), O_CREAT | O_WRONLY | O_TRUNC, 0644);
        if ((ret = write(fd, primFileName.c_str(), latestSize)) < 0) {
            throw std::runtime_error("failed to write latest data in file: " + latestFileName_);
        }
        close(fd);
    }

    frameNumTrun = (frameNumTrun + 1) % 1000;
}

struct timeval FileOutput::getAdjustedTime(int64_t timestamp_us) {
    static bool firstRun = false;
    struct timeval tv;
    time_t fullSec = timestamp_us / 1000000;
    long int microSec = timestamp_us % 1000000;

    if (!firstRun) {
        firstRun = true;
        gettimeofday(&baseTime_, NULL);
        if (baseTime_.tv_usec < microSec) {
            baseTime_.tv_usec = 1000000 + baseTime_.tv_usec - microSec;
            baseTime_.tv_sec = baseTime_.tv_sec - fullSec - 1;
        } else {
            baseTime_.tv_usec = baseTime_.tv_usec - microSec;
            baseTime_.tv_sec = baseTime_.tv_sec - fullSec;
        }
    }

    tv.tv_sec = baseTime_.tv_sec + fullSec;
    tv.tv_usec = baseTime_.tv_usec + microSec;
    if (tv.tv_usec > 1000000) {
        tv.tv_usec -= 1000000;
        tv.tv_sec += 1;
    }
    return tv;
}

void FileOutput::wrapAndWrite(void *mem, std::string fullFileName, size_t size,
                              void *exifMem, size_t exifSize, int index) {
    std::string tempFileName = fmt::format("{}.tmp", fullFileName);

    bool fileWritten = false;
    while (!fileWritten) {
        try {
            if (writeTempFile_) {
                writeFile(tempFileName, mem, size, exifMem, exifSize);
                boost::filesystem::rename(tempFileName, fullFileName);
            } else {
                writeFile(fullFileName, mem, size, exifMem, exifSize);
            }
        }
        catch (std::exception const &e) {
            std::cerr << "Failed to write file" << std::endl;
        }
        fileWritten = true;
    }
}

void FileOutput::writeFile(std::string fullFileName, void *mem, size_t size,
                           void *exifMem, size_t exifSize) {

    //open file name and assign fd
    size_t totalWritten = 0;
    int nowWritten = 0;
    int fd = open(fullFileName.c_str(), O_CREAT | O_WRONLY | O_TRUNC | O_NONBLOCK, 0644);
    uint8_t *writerIndex = (uint8_t *) mem;
    uint8_t exifLenBuff[2];

    //if we have an exif header then shift the writerIndex by 20
    if (exifSize > 0) {
        writerIndex += 20;
        size -= 20;

        exifLenBuff[0] = (exifSize + 2) >> 8;
        exifLenBuff[1] = (exifSize + 2) & 0xff;
        while (totalWritten < 4) {
            totalWritten += write(fd, exif_header, 4);
        }
        totalWritten = 0;
        while (totalWritten < 2) {
            totalWritten += write(fd, exifLenBuff, 2);
        }
        totalWritten = 0;
        while (totalWritten < exifSize) {
            totalWritten += write(fd, exifMem, exifSize);
        }
    }

    totalWritten = 0;
    while (totalWritten < size) {
        nowWritten = write(fd, writerIndex, size - totalWritten);
        if (nowWritten < 0) {
            throw std::runtime_error("failed to write data2");
        } else if (nowWritten == 0) {
            std::cerr << "no data written..." << std::endl;
        }
        writerIndex += nowWritten;
        totalWritten += nowWritten;
    }
    close(fd);

}
