/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * mjpeg_encoder.cpp - mjpeg video encoder.
 */

#include <chrono>
#include <iostream>

#include <jpeglib.h>
#include <libyuv.h>
#include <libexif/exif-data.h>
#include <libcamera/controls.h>
#include <map>
#include <stdexcept>
#include <cstring>
#include "mjpeg_encoder.hpp"

#if JPEG_LIB_VERSION_MAJOR > 9 || (JPEG_LIB_VERSION_MAJOR == 9 && JPEG_LIB_VERSION_MINOR >= 4)
typedef size_t jpeg_mem_len_t;
#else
typedef unsigned long jpeg_mem_len_t;
#endif


BufferPool::BufferPool(size_t bufferCount, size_t bufferSize) {
    for (size_t i = 0; i < bufferCount; ++i) {
        buffers_.emplace_back(new uint8_t[bufferSize]);
    }
}

BufferPool::~BufferPool() {
    for (auto& buffer : buffers_) {
        delete[] buffer;
    }
}

uint8_t* BufferPool::GetBuffer() {
    std::unique_lock<std::mutex> lock(mutex_);
    cond_.wait(lock, [this] { return !buffers_.empty(); });
    uint8_t* buffer = buffers_.front();
    buffers_.pop_front();
    return buffer;
}

void BufferPool::ReleaseBuffer(uint8_t* buffer) {
    std::unique_lock<std::mutex> lock(mutex_);
    buffers_.push_back(buffer);
    cond_.notify_one();
}


MjpegEncoder::MjpegEncoder(VideoOptions const *options)
    : Encoder(options), abort_(false), doDownsample_(false), index_(0),
      bufferPool_(NUM_ENC_THREADS, 2028 * 1024 * 1.5), didInitDSI_(false) {
    for (int i = 0; i < NUM_ENC_THREADS; ++i) {
        encode_thread_[i] = std::thread(&MjpegEncoder::encodeThread, this, i);
    }
    output_thread_ = std::thread(&MjpegEncoder::outputThread, this);
}

MjpegEncoder::~MjpegEncoder() {
    Stop();
    for (auto &thread : encode_thread_) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    if (output_thread_.joinable()) {
        output_thread_.join();
    }
}

void MjpegEncoder::EncodeBuffer(int fd, size_t size, void *mem, unsigned int width, unsigned int height,
                                unsigned int stride, int64_t timestamp_us, libcamera::ControlList metadata) {
    int32_t newExpoTime = *metadata.get(libcamera::controls::ExposureTime);
    float   newAlogGain = *metadata.get(libcamera::controls::AnalogueGain);
    float   newDigiGain = *metadata.get(libcamera::controls::DigitalGain);

    EncodeItem item = { mem,
                        size,
                        width,
                        height,
                        stride,
                        timestamp_us,
                        newExpoTime,
                        newAlogGain,
                        newDigiGain,
                        index_++ };

    std::lock_guard<std::mutex> lock(encode_mutex_);
    if (!didInitDSI_) {
        initDownSampleInfo(item);
    }
    encode_queue_.push(item);
    encode_cond_var_.notify_all();
}

void MjpegEncoder::initDownSampleInfo(EncodeItem &source) {
    if (options_->verbose) {
        std::cout << "Initializing downsample structures" << std::endl;
    }

    crop_width_ = options_->crop_width;
    crop_height_ = options_->crop_height;

    crop_stride_ = crop_width_;

    crop_half_height_ = (crop_height_ + 1) / 2;
    crop_stride2_ = crop_width_ / 2;

    crop_y_size_ = crop_stride_ * crop_height_;
    crop_uv_size_ = crop_stride2_ * crop_half_height_;

    crop_size_ = crop_y_size_ + crop_uv_size_ * 2;

    for (int ii = 0; ii < NUM_ENC_THREADS; ii += 1) {
        cropBuffer_[ii] = (uint8_t *) malloc(crop_size_);
    }

    didInitDSI_ = true;
}

void MjpegEncoder::createBuffer(struct jpeg_compress_struct &cinfo, EncodeItem &item, int num) {
    (void) num;

    //----------------------------------------------
    //----------------------------------------------
    // SRC
    //----------------------------------------------
    //----------------------------------------------
    uint8_t *src_i420 = (uint8_t *) item.mem;

//    unsigned int src_width = item.width;
    unsigned int src_height = item.height;
    unsigned int src_stride = item.stride;

//    if (options_->verbose) {
//        std::cout << "create buffer source: " <<
//                  " width:" << item.width <<
//                  " height:" << item.height <<
//                  std::endl;
//    }

    unsigned int src_half_height = (src_height + 1) / 2;
    unsigned int src_stride2 = item.stride / 2;

    unsigned int src_y_size = src_stride * src_height;
    unsigned int src_uv_size = src_stride2 * src_half_height;

    int src_U_stride = src_stride2;
    int src_V_stride = src_stride2;

    uint8_t *src_Y = (uint8_t *) src_i420;
    uint8_t *src_U = (uint8_t *) src_Y + src_y_size;
    uint8_t *src_V = (uint8_t *) src_U + src_uv_size;

    int crop_U_stride = crop_stride2_;
    int crop_V_stride = crop_stride2_;

    uint8_t *crop_Y = (uint8_t *) cropBuffer_[num];
    uint8_t *crop_U = (uint8_t *) crop_Y + crop_y_size_;
    uint8_t *crop_V = (uint8_t *) crop_U + crop_uv_size_;

    libyuv::I420Rotate(
            src_i420, src_stride,
            src_U, src_U_stride,
            src_V, src_V_stride,
            cropBuffer_[num], crop_stride_,
            crop_U, crop_U_stride,
            crop_V, crop_V_stride,
            crop_width_, crop_height_, libyuv::kRotate0);

}

void
MjpegEncoder::encodeJPEG(struct jpeg_compress_struct &cinfo, uint8_t *&encoded_buffer, size_t &buffer_len, int num) {
    (void) num;

    //----------------------------------------------
    //----------------------------------------------
    // OUT
    //----------------------------------------------
    //----------------------------------------------
    uint8_t *out_Y = (uint8_t *) cropBuffer_[num];
    unsigned int out_stride = crop_stride_;
    int out_half_stride = crop_stride2_;

    uint8_t *out_U = (uint8_t *) out_Y + crop_y_size_;
    uint8_t *out_V = (uint8_t *) out_U + crop_uv_size_;

    uint8_t *Y_max = out_Y + crop_y_size_ - 1;
    uint8_t *U_max = out_U + crop_uv_size_ - 1;
    uint8_t *V_max = out_V + crop_uv_size_ - 1;

    cinfo.image_width = crop_width_;
    cinfo.image_height = crop_height_;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;
    cinfo.restart_interval = 0;

    jpeg_set_defaults(&cinfo);
    cinfo.raw_data_in = TRUE;
    jpeg_set_quality(&cinfo, options_->quality, TRUE);
    encoded_buffer = nullptr;
    buffer_len = 0;
    jpeg_mem_len_t jpeg_mem_len;
    jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW y_rows[16];
    JSAMPROW u_rows[8];
    JSAMPROW v_rows[8];


    for (uint8_t *Y_row = out_Y, *U_row = out_U, *V_row = out_V; cinfo.next_scanline < crop_height_;) {
        for (int i = 0; i < 16; i++, Y_row += out_stride)
            y_rows[i] = std::min(Y_row, Y_max);
        for (int i = 0; i < 8; i++, U_row += out_half_stride, V_row += out_half_stride)
            u_rows[i] = std::min(U_row, U_max), v_rows[i] = std::min(V_row, V_max);

        JSAMPARRAY rows[] = {y_rows, u_rows, v_rows};
        jpeg_write_raw_data(&cinfo, rows, 16);
    }

    jpeg_finish_compress(&cinfo);
    buffer_len = jpeg_mem_len;
//    free(crop_i420_c);
}


void
MjpegEncoder::encodeDownsampleJPEG(struct jpeg_compress_struct &cinfo, uint8_t *&encoded_buffer, size_t &buffer_len,
                                   int num) {
    (void) num;

    uint8_t *scaleBuffer = bufferPool_.GetBuffer();
    
    uint8_t *crop_Y = (uint8_t *) cropBuffer_[num];
    uint8_t *crop_U = (uint8_t *) crop_Y + crop_y_size_;
    uint8_t *crop_V = (uint8_t *) crop_U + crop_uv_size_;

    unsigned int scale_width = options_->scale_width;
    unsigned int scale_height = options_->scale_height;

    unsigned int scale_y_stride = scale_width;
    unsigned int scale_uv_stride = scale_width / 2;

    unsigned int scale_y_size = scale_y_stride * scale_height;
    unsigned int scale_uv_size = scale_uv_stride * scale_height;

    uint8_t *scale_Y = (uint8_t *) scaleBuffer;
    uint8_t *scale_U = (uint8_t *) scale_Y + scale_y_size;
    uint8_t *scale_V = (uint8_t *) scale_U + scale_uv_size;

    uint8_t *scale_Y_max = scale_Y + scale_y_size - 1;
    uint8_t *scale_U_max = scale_U + scale_uv_size - 1;
    uint8_t *scale_V_max = scale_V + scale_uv_size - 1;


    libyuv::I420Scale(
            crop_Y, crop_stride_,
            crop_U, crop_stride2_,
            crop_V, crop_stride2_,
            crop_width_, crop_height_,
            scale_Y, scale_y_stride,
            scale_U, scale_uv_stride,
            scale_V, scale_uv_stride,
            scale_width, scale_height,
            libyuv::kFilterBilinear
    );

    cinfo.image_width = scale_width;
    cinfo.image_height = scale_height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_YCbCr;
    cinfo.restart_interval = 0;

    jpeg_set_defaults(&cinfo);
    cinfo.raw_data_in = TRUE;
    jpeg_set_quality(&cinfo, options_->scale_quality, TRUE);
    encoded_buffer = nullptr;
    buffer_len = 0;
    jpeg_mem_len_t jpeg_mem_len;
    jpeg_mem_dest(&cinfo, &encoded_buffer, &jpeg_mem_len);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW y_rows[16];
    JSAMPROW u_rows[8];
    JSAMPROW v_rows[8];

    for (uint8_t *Y_row = scale_Y, *U_row = scale_U, *V_row = scale_V; cinfo.next_scanline < scale_height;) {
        for (int i = 0; i < 16; i++, Y_row += scale_y_stride)
            y_rows[i] = std::min(Y_row, scale_Y_max);
        for (int i = 0; i < 8; i++, U_row += scale_uv_stride, V_row += scale_uv_stride) {
            u_rows[i] = std::min(U_row, scale_U_max);
            v_rows[i] = std::min(V_row, scale_V_max);
        }

        JSAMPARRAY rows[] = {y_rows, u_rows, v_rows};
        jpeg_write_raw_data(&cinfo, rows, 16);
    }

    jpeg_finish_compress(&cinfo);
    buffer_len = jpeg_mem_len;
    bufferPool_.ReleaseBuffer(scaleBuffer);
}

void MjpegEncoder::encodeThread(int num) {
    struct jpeg_compress_struct cinfoMain;
    struct jpeg_compress_struct cinfoPrev;

    struct jpeg_error_mgr jerr;

    cinfoMain.err = jpeg_std_error(&jerr);
    cinfoPrev.err = jpeg_std_error(&jerr);

    jpeg_create_compress(&cinfoMain);
    jpeg_create_compress(&cinfoPrev);
    typedef std::chrono::duration<float, std::milli> duration;

    duration buffer_time(0);
    duration encoding_time(0);
    duration scaling_time(0);
    duration output_time(0);
    duration total_time(0);

    EncodeItem encode_item;


//    uint8_t *encoded_buffer = (uint8_t *) malloc(options_->crop_width * options_->crop_height);
//    uint8_t *encoded_prev_buffer = (uint8_t *) malloc(options_->scale_width * options_->scale_height);
//        uint8_t *exif_buffer = nullptr;

    while (true) {
        {
            std::unique_lock<std::mutex> lock(encode_mutex_);
            while (true) {
                using namespace std::chrono_literals;
                if (abort_) {
                    std::cout << "aborting mpeg encoder: " << num  << std::endl;
                    jpeg_destroy_compress(&cinfoMain);
                    return;
                }
                if (!encode_queue_.empty()) {
                    encode_item = encode_queue_.front();
                    encode_queue_.pop();
                    break;
                } else {
                    encode_cond_var_.wait_for(lock, 200ms);
                }
            }
        }

        uint8_t *exif_buffer = nullptr;
        uint8_t *encoded_buffer = nullptr;
        uint8_t *encoded_prev_buffer = nullptr;

        size_t buffer_len = 0;
        size_t buffer_prev_len = 0;
        size_t exif_buffer_len = 0;

        auto start_buffer_time = std::chrono::high_resolution_clock::now();
        {
            createBuffer(cinfoMain, encode_item, num);
            buffer_time = (std::chrono::high_resolution_clock::now() - start_buffer_time);

            auto start_encoding_time = std::chrono::high_resolution_clock::now();
            if (!options_->skip_4k) {
                encodeJPEG(cinfoMain, encoded_buffer, buffer_len, num);
            }
            encoding_time = (std::chrono::high_resolution_clock::now() - start_encoding_time);

            auto start_scaling_time = std::chrono::high_resolution_clock::now();
            if (!options_->skip_2k) {
                encodeDownsampleJPEG(cinfoPrev, encoded_prev_buffer, buffer_prev_len, num);
            }
            scaling_time = (std::chrono::high_resolution_clock::now() - start_scaling_time);
        }

        // Don't return buffers until the output thread as that's where they're
        // in order again.
        // We push this encoded buffer to another thread so that our
        // application can take its time with the data without blocking the
        // encode process.

        auto start_output_time = std::chrono::high_resolution_clock::now();
        input_done_callback_(nullptr);


        output_ready_callback_(
                encoded_buffer, buffer_len,
                encoded_prev_buffer, buffer_prev_len,
                exif_buffer, exif_buffer_len,
                encode_item.timestamp_us,
                true);

        output_time = (std::chrono::high_resolution_clock::now() - start_output_time);
        total_time = (std::chrono::high_resolution_clock::now() - start_buffer_time);

        if (options_->verbose) {
            std::cout << "Frame processed in: " << total_time.count()
                      << " buffer: " << buffer_time.count()
                      << " 4k: " << encoding_time.count()
                      << " 2k: " << scaling_time.count()
                      << " out: " << output_time.count()
                      << std::endl;
        }



        free(exif_buffer);
        free(encoded_buffer);
        free(encoded_prev_buffer);

        if (options_->verbose) {
            stat_mutex_.lock();
            frame_second_++;
            stat_mutex_.unlock();
        }
    }
}


void MjpegEncoder::outputThread() {
    if (options_->verbose) {
        while (true) {
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                stat_mutex_.lock();
                std::cout << "Frame / sec: " << frame_second_ << std::endl;
                frame_second_ = 0;
                stat_mutex_.unlock();
            }
        }
    }
}

void MjpegEncoder::Stop() {
    abort_ = true;
    encode_cond_var_.notify_all();
}
