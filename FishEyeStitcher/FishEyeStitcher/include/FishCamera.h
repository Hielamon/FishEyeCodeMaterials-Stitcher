#pragma once
#include <OpencvCommon.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <sstream>
#include <string>
#include <numeric>
#include <fstream>

#include "CameraModel.h"
#include "Rotation.h"


namespace CircleFish
{
	class FishCamera
	{
	public:
		FishCamera(const std::shared_ptr<CameraModel> & _pModel, const std::shared_ptr<Rotation> & _pRot);

		~FishCamera();

		std::shared_ptr<CameraModel> pModel;
		std::shared_ptr<Rotation> pRot;

		bool SaveToXML(cv::FileStorage &fs);

		bool LoadFromXML(cv::FileStorage &fs);

	private:
		FishCamera();
	};

	void mapSP2I(cv::Point2d &sp_pt, cv::Point2d &img_pt, int sphere_height, const FishCamera &C);

	void buildMap(cv::Rect roi, int sphere_height, const FishCamera &C, cv::Mat &map);

	
}
