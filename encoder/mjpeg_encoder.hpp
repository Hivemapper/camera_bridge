/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.hpp - mjpeg video encoder.
 */

#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

#include "encoder.hpp"
#include "../core/video_options.hpp"

struct jpeg_compress_struct;

class BufferPool {
public:
    BufferPool(size_t bufferCount, size_t bufferSize);
    ~BufferPool();
    uint8_t* GetBuffer();
    void ReleaseBuffer(uint8_t* buffer);

private:
    std::deque<uint8_t*> buffers_;
    std::mutex mutex_;
    std::condition_variable cond_;
};

struct EncodeItem {
    void *mem;
    size_t size;
    unsigned int width;
    unsigned int height;
    unsigned int stride;
    int64_t timestamp_us;
    int32_t expo_time;
    float alog_gain;
    float digi_gain;
    uint64_t index;
};

class MjpegEncoder : public Encoder {
public:
    MjpegEncoder(VideoOptions const *options);
    ~MjpegEncoder();

    // Encode the given buffer.
    void EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height, unsigned int stride,
                      int64_t timestamp_us, libcamera::ControlList metadata) override;
    void Stop() override;

private:
    static const int NUM_ENC_THREADS = 4;
    static const size_t MAX_BUFFER_SIZE = 30; // Adjust based on available memory

    void encodeThread(int num);
    void outputThread();
    void adjustThreadCount(); // Declaration for the new method

    bool abort_;
    bool doDownsample_;
    uint64_t index_;
    BufferPool bufferPool_;

    // Global frame buffer and mutex
    std::deque<EncodeItem> frame_buffer;
    std::mutex frame_buffer_mutex;

    std::mutex encode_mutex_;
    std::mutex stat_mutex_;
    std::condition_variable encode_cond_var_;
    std::vector<std::thread> encode_thread_; // Changed to vector to dynamically manage threads
    std::thread output_thread_;

    bool didInitDSI_;

    uint32_t frame_second_;
    unsigned int crop_width_;
    unsigned int crop_height_;
    unsigned int crop_stride_;
    unsigned int crop_half_height_;
    unsigned int crop_stride2_;
    unsigned int crop_y_size_;
    unsigned int crop_uv_size_;
    unsigned int crop_size_;
    uint8_t *cropBuffer_[NUM_ENC_THREADS];

    void initDownSampleInfo(EncodeItem &source);
    void createBuffer(struct jpeg_compress_struct &cinfo, EncodeItem &item, int num);
    void encodeJPEG(struct jpeg_compress_struct &cinfo, uint8_t *&encoded_buffer, size_t &buffer_len, int num);
    void encodeDownsampleJPEG(struct jpeg_compress_struct &cinfo, uint8_t *&encoded_buffer, size_t &buffer_len, int num);
};
