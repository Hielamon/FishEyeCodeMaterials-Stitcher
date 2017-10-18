#pragma once
#include <OpencvCommon.h>
#include <Ransac.h>
#include <ORBextractor.h>
#include <ORBMatcher.h>
#include <sstream>
#include <cmath>

#ifndef MYMAX
#define MYMAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MYMIN
#define MYMIN(a,b) (((a) < (b)) ? (a) : (b))
#endif


namespace CircleFish
{
	//Motion Search in the square region
	//(input-output)motion : input the initmotion ,output the result motion
	bool CompareTemplate(cv::Mat &src, cv::Mat &aim, cv::Vec2i &motion, int square_size);

	void BinaryByGridient(cv::Mat &src, cv::Mat &result);

	inline bool CheckValid(const cv::Mat &src);

	inline bool CheckMask(cv::Mat &src);

	class FineMapping
	{
	public:

		typedef std::vector<std::vector<size_t> > GridIndex;
		typedef std::vector<cv::Vec2f> GridShift;
		typedef std::vector<bool> GridFlag;

		typedef std::pair<cv::Mat, cv::Vec2f> MapAndX;

		enum TempStat
		{
			NONE, UP_EMPTY, DOWN_EMPTY, ALL_EMPTY
		};

		FineMapping(ORBextractor *pORBextractor);

		~FineMapping();

		/*
		(input)  img1  , img2    : same size images which are overlap region to be Finemapping
		(output) mAndx1, mAndx2  : the projection map , min and max motion in x direction for
		two input images
		(return)				 : whether operation successfully
		*/
		bool operator ()(cv::Mat &img1, cv::Mat &img2, MapAndX &mAndx1, MapAndX &mAndx2, cv::Mat &mask1, cv::Mat &mask2);

	private:

		void _assignToGrid(std::vector<cv::KeyPoint> &vkpts, GridIndex &aGridIndex);

		void _fillIndexContainer(ORBMatcher::IndexContainer &aidxcontainer12, std::vector<cv::KeyPoint> &vkpts1,
			std::vector<cv::KeyPoint> &vkpts2);

		//Get the average shift in every aGridShift£¬in the sign of Test£¬meanwhile refining the vnMatches12
		void _fillGridShift(GridShift &aGridShift, GridFlag &aGridFlag, std::vector<cv::KeyPoint> &vkpts1,
			std::vector<cv::KeyPoint> &vkpts2, std::vector<int> &vnMatches12);

		bool _fillWarpMap(GridShift &aGridShift1, MapAndX &mAndx1, MapAndX &mAndx2);

		void _templateMatchGrid(int r, GridShift &aGridShift);

		ORBextractor *mpORBextractor;

		size_t mnRowNum;
		size_t mnWindowSize;

		cv::Mat mimg1, mimg2;
		cv::Mat mimg1_grey, mimg2_grey;
		cv::Mat mmask1, mmask2;

		GridIndex maGridIndex1, maGridIndex2;
		std::vector<TempStat> mvStat;

		//changed in every process
		float mfGridElementWidthInv, mfGridElementHeightInv;
		int mnGridElementWidth, mnGridElementHeight;

	};
}
