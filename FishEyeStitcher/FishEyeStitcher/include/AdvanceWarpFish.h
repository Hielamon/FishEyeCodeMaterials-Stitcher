#pragma once
#include <FishCamera.h>
#include <FineMapping.h>
#include <map>

namespace CircleFish
{
	class AdvanceWarpFish
	{
	public:
		AdvanceWarpFish(int sphere_height, bool is_tunning = true);
		~AdvanceWarpFish();

		void process(const std::vector<cv::Mat> &src_arr, std::vector<FishCamera> &cameras,
			std::vector<int> &index, bool is_ring, std::vector<cv::Mat> &blend_warpeds,
			std::vector<cv::Mat> &blend_warped_masks, std::vector<cv::Point>& blend_corners);

		void getoverlap(std::vector<std::vector<cv::Rect> > &rois,
			std::vector<std::pair<cv::Rect, cv::Rect> > &lap_rois,
			std::vector<std::pair<int, int> > &lap_idx, bool is_ring);

		void getoverlap(std::vector<std::vector<cv::Rect2d> > &rois,
			std::vector<std::pair<cv::Rect2d, cv::Rect2d> > &lap_rois,
			std::vector<std::pair<int, int> > &lap_idx, bool is_ring);

		std::vector<std::vector<cv::Rect> > result_warped_rois;

		std::vector<std::pair<cv::Rect, cv::Rect> > init_lap_rois;
		std::vector<std::pair<int, int> > init_lap_idx;
		bool m_is_ring;

	private:

		void _initWarp(const std::vector<cv::Mat> &src_arr, std::vector<FishCamera> &cameras,
			std::vector<int> &index, std::vector<cv::Mat> &blend_warpeds,
			std::vector<cv::Mat> &blend_warped_masks, std::vector<cv::Point>& blend_corners,
			std::vector<std::vector<cv::Rect> > &rois);

		//The height of Spere Warp
		//Here it's always the height of stitching result
		int m_sphere_height;

		//Whether to do the final FineMapping operator
		//For the outdoor scene, Mostly, this need to be false
		//For the indoor scene, Mostly, this need to be true, and default to be true
		bool m_is_tunning;
	};
}
