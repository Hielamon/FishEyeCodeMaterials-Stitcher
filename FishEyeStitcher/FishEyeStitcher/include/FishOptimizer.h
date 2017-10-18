#pragma once

#include <FishCamera.h>
#include <SequenceMatcher.h>

namespace CircleFish
{
	void calcRotation(PairInfo &pairinfo, FishCamera &camera);

	void calcDeriv(const cv::Mat &err1, const cv::Mat &err2, double h, cv::Mat res);

	class FishOptimizer
	{
	public:

		FishOptimizer();
		~FishOptimizer();

		void setMask(std::vector<bool> &mask);

		bool setParameters(std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, bool sparse = false);
		void optimizeProcess();

		void getCameras(std::vector<FishCamera> &cameras);

	private:
		std::list<PairInfo> m_pairinfos;
		std::vector<FishCamera> m_cameras;
		std::vector<double> m_pairweight;

		std::vector<bool> m_mask;
		int m_num_param;

		double m_ori_u0, m_ori_v0;

		//是否形成闭环
		bool m_is_ring;

		void _calcError(cv::Mat &err);

		void _calcJacobian(cv::Mat &jac);
	};
}