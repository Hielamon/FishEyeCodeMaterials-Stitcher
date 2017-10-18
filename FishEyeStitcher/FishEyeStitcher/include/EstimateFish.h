#pragma once
#include <FishCamera.h>
#include <FishOptimizer.h>
#include <SequenceMatcher.h>
#include <Ransac.h>

namespace CircleFish
{

	class EstimateFish
	{
	public:
		EstimateFish();
		~EstimateFish();

		void operator ()(std::vector<cv::Mat> &images, std::vector<FishCamera> &cameras, std::vector<int> &index);
		bool m_is_ring;

	private:

		//初始化计算相机相对姿态R矩阵，如果鱼眼图像覆盖360以上，或者说满足大致的4张90度转条件，返回true，否则返回false
		bool _initCompute(std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, std::vector<int> &index);

		void _ransacRotation(PairInfo &pairinfo, FishCamera &camera, double bad_thres);

		void _stretchCameras(std::vector<FishCamera> &cameras);

		void _alignCameras(std::vector<FishCamera> &cameras, std::vector<int> &index);

		void _showPairInfo(std::vector<cv::Mat> &images, std::list<PairInfo> &pairinfos, double scale);

		double _getBiggestError(std::vector<cv::Mat> &images, std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, bool is_draw, double scale);

		double bad_threshold;
	};

}
