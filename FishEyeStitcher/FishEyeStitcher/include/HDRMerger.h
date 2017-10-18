#pragma once
#include <OpencvCommon.h>


class HDRMerger
{
public:
	HDRMerger();
	~HDRMerger();


	/*
	(input)  src : a sequence images to HDR merging
	(output) dst : the merged result
	(return)     : whether merging successfully
	*/
	bool operator ()(std::vector<cv::Mat> &src, cv::Mat &dst);

	/*
	(input)  src : a array of sequences images to HDR merging
	(output) dst : a array of the merged result
	(return)     : whether merging successfully
	*/
	bool operator ()(std::vector<std::vector<cv::Mat> > &src, std::vector<cv::Mat> &dst);

private:

};

