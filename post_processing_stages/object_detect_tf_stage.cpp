/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi (Trading) Limited
 *
 * object_detect_tf_stage.cpp - object detector
 */

#include "./object_detect.hpp"
#include "./tf_stage.hpp"

using Rectangle = libcamera::Rectangle;

// TODO: get this data from the config files
constexpr int WIDTH = 640; // 300 is for detect.tflite
constexpr int HEIGHT = 640; // 300 is for detect.tflite

struct ObjectDetectTfConfig : public TfConfig {
	float confidence_threshold;
	float overlap_threshold;
    float tensor_width;
    float tensor_height;
    int coordinates_detected_objects;
    int class_labels;
    int confidence_score;
};

#define NAME "object_detect_tf"

class ObjectDetectTfStage : public TfStage {
    public:
        // The model we use expects 224x224 images. The detect.tflite expects 300x300 images
        ObjectDetectTfStage(LibcameraApp *app) : TfStage(app, WIDTH, HEIGHT) {
            config_ = std::make_unique<ObjectDetectTfConfig>();
        }
        
        char const *Name() const override { 
            return NAME; 
        }

    protected:
        ObjectDetectTfConfig *config() const { 
            return static_cast<ObjectDetectTfConfig *>(config_.get()); 
        }

        // Read the label file, plus some thresholds.
        void readExtras(boost::property_tree::ptree const &params) override;

        void checkConfiguration() override;

        // Retrieve the top-n most likely results.
        void interpretOutputs() override;

        // Attach the results as metadata; optionally write the labels too for the annotate_cv
        // stage to pick up.
        void applyResults(CompletedRequestPtr &completed_request) override;

    private:
        void readLabelsFile(const std::string &file_name);

        std::vector<Detection> output_results_;
        std::vector<std::string> labels_;
        size_t label_count_;
};

void ObjectDetectTfStage::readExtras(boost::property_tree::ptree const &params) {
	config()->confidence_threshold = params.get<float>("confidence_threshold", 0.5f);
	config()->overlap_threshold = params.get<float>("overlap_threshold", 0.5f);
    config()->coordinates_detected_objects = params.get<int>("coordinates_detected_objects", 1);
    config()->class_labels = params.get<int>("class_labels", 6);
    config()->confidence_score = params.get<int>("confidence_score", 8400);
    config()->tensor_height = params.get<float>("tensor_height", 640);
    config()->tensor_width = params.get<float>("tensor_width", 640);

	std::string labels_file = params.get<std::string>("labels_file", "");
	readLabelsFile(labels_file);
	if (config()->verbose) {
        std::cout << "Read " << label_count_ << " labels" << std::endl;
        std::cout << "Tensor height: " << config()->tensor_height << " width: " << config()->tensor_width << std::endl;
        std::cout << "Coordinates detected Objects: " << config()->coordinates_detected_objects << std::endl;
        std::cout << "Class labels: " << config()->class_labels << std::endl;
        std::cout << "Confidence score: " << config()->confidence_score << std::endl;
    }

	// Check that the tensor outputs and label classes match up.
	int output = interpreter_->outputs()[0];

	TfLiteIntArray *output_dims = interpreter_->tensor(output)->dims;

    // From the privacy7000 model spec: Coordinates of detected objects, class labels, and confidence score
    // int data0 = 1; // orignally: 1
    // int data1 = 6; // orignally: 10
    // int data2 = 8400; // orignally: 4

    // Causes might include loading the wrong model.
	if (output_dims->size != 3 || 
        output_dims->data[0] != config()->coordinates_detected_objects || 
        output_dims->data[1] != config()->class_labels || 
        output_dims->data[2] != config()->confidence_score) {

        throw std::runtime_error("ObjectDetectTfStage: unexpected output dimensions");
    }
}

void ObjectDetectTfStage::readLabelsFile(const std::string &file_name) {
	std::ifstream file(file_name);
	if (!file)
		throw std::runtime_error("ObjectDetectTfStage: Failed to load labels file");

	std::string line;
	std::getline(file, line); // discard first line of ???
	while (std::getline(file, line))
		labels_.push_back(line);

	label_count_ = labels_.size();
}

void ObjectDetectTfStage::checkConfiguration() {
	if (!main_stream_)
		throw std::runtime_error("ObjectDetectTfStage: Main stream is required");
}

void ObjectDetectTfStage::applyResults(CompletedRequestPtr &completed_request) {
	completed_request->post_process_metadata.Set("object_detect.results", output_results_);
}

static unsigned int area(const Rectangle &r) {
	return r.width * r.height;
}

void ObjectDetectTfStage::interpretOutputs() {
    if (config()->verbose) {
        std::cout << "Interpreting outputs" << std::endl;
    }

    int box_index = interpreter_->outputs()[0];
    int class_index = interpreter_->outputs()[1];
    int score_index = interpreter_->outputs()[2];

    std::cout << "box_index: " << box_index << std::endl;
    std::cout << "class_index: " << class_index << std::endl;
    std::cout << "score_index: " << score_index << std::endl;

    int num_detections = interpreter_->tensor(box_index)->dims->data[1];
    float *boxes = interpreter_->tensor(box_index)->data.f;
    float *scores = interpreter_->tensor(score_index)->data.f;
    float *classes = interpreter_->tensor(class_index)->data.f;

    std::cout << "boxes: " << boxes << std::endl;
    std::cout << "scores: " << scores << std::endl;
    std::cout << "num_detections: " << num_detections << std::endl;

    output_results_.clear();

    float tensor_h = config()->tensor_height;
    float tensor_w = config()->tensor_width;

    for (int i = 0; i < num_detections; i++) {
        float s = scores[i];
        std::cout << " score: " << s << std::endl;

        if (scores[i] < config()->confidence_threshold) {
            continue;
        }

        // The coords in the WIDTH x HEIGHT image fed to the network are:
        int y = std::clamp<int>(tensor_h * boxes[i * 4 + 0], 0, tensor_h);
        int x = std::clamp<int>(tensor_w * boxes[i * 4 + 1], 0, tensor_w);
        int h = std::clamp<int>(tensor_h * boxes[i * 4 + 2] - y, 0, tensor_h);
        int w = std::clamp<int>(tensor_w * boxes[i * 4 + 3] - x, 0, tensor_w);
        // The network is fed a crop from the lores (if that was too large), so the coords
        // in the full lores image are:
        y += (lores_h_ - tensor_h) / 2;
        x += (lores_w_ - tensor_w) / 2;
        // The lores is a pure scaling of the main image (squishing if the aspect ratios
        // don't match), so:
        y = y * main_h_ / lores_h_;
        x = x * main_w_ / lores_w_;
        h = h * main_h_ / lores_h_;
        w = w * main_w_ / lores_w_;

        int c = classes[i];
        Detection detection(c, labels_[c], scores[i], x, y, w, h);

        // Before adding this detection to the results, see if it overlaps an existing one.
        bool overlapped = false;
        for (auto &prev_detection : output_results_) {
            if (prev_detection.category == c) {
                unsigned int prev_area = area(prev_detection.box);
                unsigned int new_area = area(detection.box);
                unsigned int overlap = area(prev_detection.box.boundedTo(detection.box));

                if (overlap > config()->overlap_threshold * prev_area ||
                    overlap > config()->overlap_threshold * new_area) {

                    // Take the box with the higher confidence.
                    if (detection.confidence > prev_detection.confidence) {
                        prev_detection = detection;   
                    }

                    overlapped = true;
                    std::cout << "image overlaps!!!" << std::endl;
                    break;
                }
            }
        }
        if (!overlapped)
            output_results_.push_back(detection);
    }

    if (config()->verbose) {
        for (auto &detection : output_results_) {
            std::cout << "Detected: " << detection.toString() << std::endl;
        }
    }
}

//void ObjectDetectTfStage::interpretOutputs()
//{
//    std::cout << " ObjectDetectTfStage::interpretOutputs" << std::endl;
//
//    //todo: work with silvio to decode output, need tenser flow model
//
//	int box_index = interpreter_->outputs()[0];
//	int class_index = interpreter_->outputs()[1];
//	int score_index = interpreter_->outputs()[2];
//	int num_detections = interpreter_->tensor(box_index)->dims->data[1];
//	float *boxes = interpreter_->tensor(box_index)->data.f;
//	float *scores = interpreter_->tensor(score_index)->data.f;
//	float *classes = interpreter_->tensor(class_index)->data.f;
//
//	output_results_.clear();
//
//	for (int i = 0; i < num_detections; i++)
//	{
//		if (scores[i] < config()->confidence_threshold)
//			continue;
//
//		// The coords in the WIDTH x HEIGHT image fed to the network are:
//		int y = std::clamp<int>(HEIGHT * boxes[i * 4 + 0], 0, HEIGHT);
//		int x = std::clamp<int>(WIDTH * boxes[i * 4 + 1], 0, WIDTH);
//		int h = std::clamp<int>(HEIGHT * boxes[i * 4 + 2] - y, 0, HEIGHT);
//		int w = std::clamp<int>(WIDTH * boxes[i * 4 + 3] - x, 0, WIDTH);
//		// The network is fed a crop from the lores (if that was too large), so the coords
//		// in the full lores image are:
//		y += (lores_h_ - HEIGHT) / 2;
//		x += (lores_w_ - WIDTH) / 2;
//		// The lores is a pure scaling of the main image (squishing if the aspect ratios
//		// don't match), so:
//		y = y * main_h_ / lores_h_;
//		x = x * main_w_ / lores_w_;
//		h = h * main_h_ / lores_h_;
//		w = w * main_w_ / lores_w_;
//
//		int c = classes[i];
//		Detection detection(c, labels_[c], scores[i], x, y, w, h);
//
//		// Before adding this detection to the results, see if it overlaps an existing one.
//		bool overlapped = false;
//		for (auto &prev_detection : output_results_)
//		{
//			if (prev_detection.category == c)
//			{
//				unsigned int prev_area = area(prev_detection.box);
//				unsigned int new_area = area(detection.box);
//				unsigned int overlap = area(prev_detection.box.boundedTo(detection.box));
//				if (overlap > config()->overlap_threshold * prev_area ||
//					overlap > config()->overlap_threshold * new_area)
//				{
//					// Take the box with the higher confidence.
//					if (detection.confidence > prev_detection.confidence)
//						prev_detection = detection;
//					overlapped = true;
//					break;
//				}
//			}
//		}
//		if (!overlapped)
//			output_results_.push_back(detection);
//	}
//
//	if (config()->verbose)
//	{
//		for (auto &detection : output_results_)
//			std::cerr << detection.toString() << std::endl;
//	}
//}

static PostProcessingStage *Create(LibcameraApp *app) {
    return new ObjectDetectTfStage(app);
}

static RegisterStage reg(NAME, &Create);
