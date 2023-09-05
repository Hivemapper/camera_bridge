//
// Created by Eduard Voiculescu on 2023-08-16.
//

#ifndef CAMERA_BRIDGE_ONXX_STAGE_H
#define CAMERA_BRIDGE_ONXX_STAGE_H

#include "inference.h"

struct OnnxConfig {
    int number_of_threads = 3;
    int refresh_rate = 5;
    std::string model_file;
    bool verbose = false;
};

class OnxxStage : public PostProcessingStage {
    public:
        // The TfStage provides implementations of the PostProcessingStage functions except for
        // Name(), which derived classes must still provide.

        // The constructor supplies the width and height that TFLite wants.
        OnxxStage(LibcameraApp *app, int tf_w, int tf_h);

        //char const *Name() const override;

        void Read(boost::property_tree::ptree const &params) override;

        void Configure() override;

        bool Process(CompletedRequestPtr &completed_request) override;

        void Stop() override;

    protected:
        OnnxConfig *config() const { return config_.get(); }

        // Instead of redefining the above public interface, derived class should implement
        // the following four virtual methods.

        // Read additional parameters required by the stage. Can also do some model checking.
        virtual void readExtras(boost::property_tree::ptree const &params) {}

        // Check the stream and image configuration. Here the stage should report any errors
        // and/or fail.
        virtual void checkConfiguration() {}

        // This runs asynchronously from the main thread right after the model has run. The
        // outputs should be processed into a form where applyResults can make use of them.
        virtual void interpretOutputs() {}

        // Here we run synchronously again and so should not take too long. The results
        // produced by interpretOutputs can be used now, for example as metadata to attach
        // to the image, or even drawn onto the image itself.
        virtual void applyResults(CompletedRequestPtr &completed_request) {}

        std::unique_ptr<OnnxConfig> config_;

        // The width and height that Onnx wants.
        unsigned int onnx_w_, onnx_h_;

        // We run Onnx on the low resolution image, details of which are here.
        libcamera::Stream *lores_stream_;
        unsigned int lores_w_, lores_h_, lores_stride_;

        // TODO: check if we need this ??
        // The stage may or may not make use of the larger or "main" image stream.
        libcamera::Stream *main_stream_;
        unsigned int main_w_, main_h_, main_stride_;

        inference::Inference inf;

    private:
        void initialise();
        void runInference();

        std::mutex future_mutex_;
        std::unique_ptr<std::future<void>> future_;
        std::vector<uint8_t> lores_copy_;
        std::mutex output_mutex_;
};


#endif //CAMERA_BRIDGE_ONXX_STAGE_H
