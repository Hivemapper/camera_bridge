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

    void encodeThread(int num);
    void outputThread();

    bool abort_;
    bool doDownsample_;
    uint64_t index_;
    BufferPool bufferPool_;

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

    std::queue<EncodeItem> encode_queue_;
    std::mutex encode_mutex_;
    std::condition_variable encode_cond_var_;
    std::thread encode_thread_[NUM_ENC_THREADS];

    bool didInitDSI_;

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
