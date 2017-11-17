#pragma once

#include <Ransac.h>
#include <FishCamera.h>
#include <AdvanceWarpFish.h>
#include <EstimateFish.h>

#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/util.hpp>
#include <opencv2/stitching/detail/motion_estimators.hpp>

namespace CircleFish
{
	class FishStitcher
	{
	public:
		FishStitcher(const size_t result_height = 4000);
		~FishStitcher();

		/*
		(input)  images : a sequence images to Stitching
		(output) dst    : the stitched result
		(return)        : whether stitching successfully
		*/
		bool operator ()(std::vector<cv::Mat> images, bool do_fine_tune, cv::Mat &result);
		

	private:
		
		/*
		(input)  blend_warpeds      : warped images for blending
		(input)  blend_warped_masks : warped image masks for blending
		(input)  blend_corners      : the top-left coordinate of warped images
		(input)  blend_size         : the size of warped images
		(output) result             : the blended result
		*/
		void _blendCompute(std::vector<cv::Mat>& blend_warpeds, std::vector<cv::Mat>& blend_warped_masks,
			std::vector<cv::Point> &blend_corners, cv::Size blend_size, cv::Mat &result);

		/*
		(input)  img    : a circle fisheye image for extract the cicle
		(output) center : the circle region's center point
		(output) radius : the circle region's radius
		*/
		void _getCircleRegion(cv::Mat &img, cv::Point2d &center, double &radius);

		/*
		(input)  img           : a circle fisheye image for extract the cicle edge points
		(output) circle_points : the circle edge poins
		*/
		void _getCircleEdgePoints(const cv::Mat &img, std::vector<std::vector<int> >&circle_points);

		void _checkSize(cv::Mat &result);

		//Used in _getCircleEdgePoints for determining the no-black threshold
		uchar m_black;

		cv::Size m_findSeam_size, m_blend_size;
		cv::Size m_result_size;

		int m_expos_comp_type;
		int m_blend_type;
		double m_blend_strength;
	};
}
