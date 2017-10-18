#include <HDRMerger.h>
#include <iostream>

HDRMerger::HDRMerger() {}
HDRMerger::~HDRMerger() {}

/*
(input)  src : a sequence images to HDR merging
(output) dst : the merged result
(return)     : whether merging successfully
*/
bool HDRMerger::operator ()(std::vector<cv::Mat> &src, cv::Mat &dst)
{
	float wcon = 1.0;
	float wsat = 1.0;
	float wexp = 0.0;

	if (src.size() == 0)return false;

	int channels = src[0].channels();
	if (channels != 1 && channels != 3) return false;
	cv::Size size = src[0].size();
	int CV_32FCC = CV_MAKETYPE(CV_32F, channels);

	for (size_t i = 0; i < src.size(); i++)
	{
		if (src[i].channels() != channels || src[i].size() != size)
			return false;
	}


	double megapix = 0.3;
	double weight_scale = std::min(1.0, sqrt(megapix * 1e6 / src[0].size().area()));
	int src_w = src[0].cols, src_h = src[0].rows;
	cv::Size weight_size(src_w*weight_scale, src_h*weight_scale);

	std::vector<cv::Mat> weights(src.size());
	cv::Mat weight_sum = cv::Mat::zeros(size, CV_32F);

	for (size_t i = 0; i < src.size(); i++) {
		cv::Mat img, gray, contrast, saturation, wellexp;
		std::vector<cv::Mat> splitted(channels);

		cv::Mat weight_img;
		if (weight_scale == 1.0)
			weight_img = src[i];
		else cv::resize(src[i], weight_img, weight_size);


		weight_img.convertTo(img, CV_32F, 1.0f / 255.0f);
		if (channels == 3) {
			cvtColor(img, gray, cv::COLOR_RGB2GRAY);
		}
		else {
			img.copyTo(gray);
		}
		split(img, splitted);

		Laplacian(gray, contrast, CV_32F);
		contrast = cv::abs(contrast);

		cv::Mat mean = cv::Mat::zeros(weight_size, CV_32F);
		for (int c = 0; c < channels; c++) {
			mean += splitted[c];
		}
		mean /= channels;

		saturation = cv::Mat::zeros(weight_size, CV_32F);
		for (int c = 0; c < channels; c++) {
			cv::Mat deviation = splitted[c] - mean;
			pow(deviation, 2.0f, deviation);
			saturation += deviation;
		}
		sqrt(saturation, saturation);

		weights[i] = contrast;
		if (channels == 3) {
			weights[i] = weights[i].mul(saturation);
		}
		weights[i] = weights[i]/*.mul(wellexp)*/ + 1e-12f;
		cv::resize(weights[i], weights[i], size);
		weight_sum += weights[i];
	}

	int maxlevel = static_cast<int>(logf(static_cast<float>(std::min(size.width, size.height))) / logf(2.0f));

	std::vector<cv::Mat> res_pyr(maxlevel + 1);
	for (size_t i = 0; i < src.size(); i++) {
		weights[i] /= weight_sum;
		cv::Mat img;
		src[i].convertTo(img, CV_32F, 1.0f / 255.0f);

		double duration1 = static_cast<double>(cv::getTickCount());
		std::vector<cv::Mat> img_pyr, weight_pyr;

		cv::buildPyramid(img, img_pyr, maxlevel);
		cv::buildPyramid(weights[i], weight_pyr, maxlevel);


		for (int lvl = 0; lvl < maxlevel; lvl++) {
			cv::Mat up;
			pyrUp(img_pyr[lvl + 1], up, img_pyr[lvl].size());
			img_pyr[lvl] -= up;
		}
		for (int lvl = 0; lvl <= maxlevel; lvl++) {
			std::vector<cv::Mat> splitted(channels);
			split(img_pyr[lvl], splitted);
			for (int c = 0; c < channels; c++) {
				splitted[c] = splitted[c].mul(weight_pyr[lvl]);
			}
			merge(splitted, img_pyr[lvl]);
			if (res_pyr[lvl].empty()) {
				res_pyr[lvl] = img_pyr[lvl];
			}
			else {
				res_pyr[lvl] += img_pyr[lvl];
			}
		}

	}

	for (int lvl = maxlevel; lvl > 0; lvl--) {
		cv::Mat up;
		pyrUp(res_pyr[lvl], up, res_pyr[lvl - 1].size());
		res_pyr[lvl - 1] += up;
	}
	dst.create(size, CV_32FCC);
	res_pyr[0].copyTo(dst);
	return true;
}

/*
(input)  src : a array of sequences images to HDR merging
(output) dst : a array of the merged result
(return)     : whether merging successfully
*/
bool HDRMerger::operator ()(std::vector<std::vector<cv::Mat> > &src, std::vector<cv::Mat> &dst)
{
	for (size_t i = 0; i < src.size(); i++)
	{
		cv::Mat fusion;
		if (!(*this)(src[i], fusion))
		{
			std::cout << i << "th sequence images failed to HDRMerger" << std::endl;
			return false;
		}

		cv::Mat result_temp = fusion * 255;
		result_temp.convertTo(result_temp, CV_8UC3);
		dst.push_back(result_temp);
	}
	return true;
}
