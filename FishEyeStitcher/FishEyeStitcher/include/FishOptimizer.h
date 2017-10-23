#pragma once

#include <FishCamera.h>
#include <SequenceMatcher.h>
#include <precomp.hpp>

namespace CircleFish
{
	inline void calcRotation(PairInfo &pairinfo, FishCamera &camera)
	{

		double s[9] = { 0 };
		cv::Mat S(3, 3, CV_64FC1, s);
		for (size_t i = 0; i < pairinfo.pairs_num; i++)
		{
			if (pairinfo.mask[i] != 1)continue;

			cv::Point3d spherePt1, spherePt2;
			camera.pModel->mapI2S(pairinfo.points1[i], spherePt1);
			camera.pModel->mapI2S(pairinfo.points2[i], spherePt2);

			s[0] += (spherePt1.x * spherePt2.x);
			s[1] += (spherePt1.x * spherePt2.y);
			s[2] += (spherePt1.x * spherePt2.z);
			s[3] += (spherePt1.y * spherePt2.x);
			s[4] += (spherePt1.y * spherePt2.y);
			s[5] += (spherePt1.y * spherePt2.z);
			s[6] += (spherePt1.z * spherePt2.x);
			s[7] += (spherePt1.z * spherePt2.y);
			s[8] += (spherePt1.z * spherePt2.z);
		}

		cv::Mat w, u, vt;
		cv::SVD::compute(S, w, u, vt);
		cv::Mat I = cv::Mat::eye(3, 3, CV_64FC1);
		I.at<double>(2, 2) = cv::determinant(vt.t() * u.t());
		cv::Mat R = vt.t() * I * u.t();
		camera.pRot->updataRotation(R);
	}

	inline void calcDeriv(const cv::Mat &err1, const cv::Mat &err2, double h, cv::Mat &res)
	{
		for (int i = 0; i < err1.rows; ++i)
			res.at<double>(i, 0) = (err2.at<double>(i, 0) - err1.at<double>(i, 0)) / h;
	}

	class FishCameraRefineCallback : public cv::LMSolver::Callback
	{
	public:
		
		FishCameraRefineCallback(std::list<PairInfo> & lPairInfos,
								std::vector<FishCamera> & vFishCameras,
								const std::vector<uchar> &vCameraMask, bool bRotationScheme = true) :
			mlPairInfos(lPairInfos), mvFishCameras(vFishCameras), mbRotationScheme(bRotationScheme)
		{
			mTotalPairs = 0;
			for (auto iter = lPairInfos.begin(); iter != lPairInfos.end(); iter++)
			{
				mTotalPairs += iter->pairs_num;
				mpEndPair = iter;
			}

			mbRing = lPairInfos.size() == vFishCameras.size();
			if (!mbRing)mpEndPair = lPairInfos.end();

			//check the validation of vFishCameras
			{
				bool valid = true;
				for (size_t i = 0; i < vFishCameras.size(); i++)
				{
					valid &= vFishCameras[i].pModel.use_count() != 0;
					valid &= vFishCameras[i].pRot.use_count() != 0;
				}

				assert(valid);
			}

			assert(vFishCameras.size() > 0);
			std::shared_ptr<CameraModel> pModel = vFishCameras[0].pModel;
			assert(vCameraMask.size() == pModel->vpParameter.size());

			for (size_t i = 0; i < vCameraMask.size(); i++)
			{
				if (vCameraMask[i] != 0)
				{
					mvpParameter.push_back(pModel->vpParameter[i]);
					mvRotMask.push_back(false);
					mvParamCameraIdx.push_back(0);
				}
			}

			if (!mbRotationScheme)
			{
				for (auto iter = lPairInfos.begin(); iter != mpEndPair; iter++)
				{
					int index2 = iter->index2;
					const std::shared_ptr<Rotation> & pRot = vFishCameras[index2].pRot;
					for (size_t j = 0; j < 3; j++)
					{
						mvpParameter.push_back(&(pRot->axisAngle[j]));
						mvRotMask.push_back(true);
						mvParamCameraIdx.push_back(index2);
					}
				}
			}
		}

		bool compute(cv::InputArray _param, cv::OutputArray _err, cv::OutputArray _Jac) const
		{
			cv::Mat param = _param.getMat();

			for (size_t i = 0; i < mvpParameter.size(); i++)
			{
				*(mvpParameter[i]) = param.at<double>(i, 0);
			}

			_err.create(mTotalPairs * 3, 1, CV_64F);
			cv::Mat err = _err.getMat();
			if (!_calcError(err))return false;

			cv::Mat err2 = _err.getMat();

			if (_Jac.needed())
			{
				_Jac.create(mTotalPairs * 3, mvpParameter.size(), CV_64F);
				cv::Mat J = _Jac.getMat();
				if (!_calcJacobian(J))return false;
			}

			/*std::cout << "average error = " << norm(err) << std::endl;
			std::cout << param.at<double>(0, 0) << std::endl;
			std::cout << param.at<double>(1, 0) << " " << param.at<double>(2, 0) << std::endl;
			std::cout << param.at<double>(3, 0) << " " << param.at<double>(4, 0) << " " << param.at<double>(5, 0) << std::endl;*/
			return true;
		}

	private:

		bool _calcError(cv::Mat &err) const
		{
			//err.create(pairNum * 3, 1, CV_64F);
			err.setTo(0);
			bool valid = true;

			cv::Mat ringR = cv::Mat::eye(3, 3, CV_64F);

			int idx = 0;
			for (auto iter = mlPairInfos.begin(); iter != mpEndPair; iter++)
			{
				int index1 = iter->index1, index2 = iter->index2;

				if (mbRotationScheme)
					calcRotation(*iter, mvFishCameras[index2]);

				ringR = mvFishCameras[index2].pRot->R * ringR;
				for (size_t i = 0; i < iter->pairs_num; i++)
				{
					if (iter->mask[i] != 1)continue;

					cv::Point3d spherePt1, spherePt2;
					valid &= mvFishCameras[index1].pModel->mapI2S(iter->points1[i], spherePt1);
					valid &= mvFishCameras[index2].pModel->mapI2S(iter->points2[i], spherePt2);

					if (!valid)return false;

					cv::Point3d spherePt1ByRot = RotatePoint(spherePt1, *(mvFishCameras[index2].pRot));
					err.at<double>(idx, 0) = spherePt1ByRot.x - spherePt2.x;
					err.at<double>(idx + 1, 0) = spherePt1ByRot.y - spherePt2.y;
					err.at<double>(idx + 2, 0) = spherePt1ByRot.z - spherePt2.z;

					idx += 3;
				}
			}

			if (mbRing && mpEndPair != mlPairInfos.end())
			{
				int index1 = mpEndPair->index1, index2 = mpEndPair->index2;
				for (size_t i = 0; i < mpEndPair->pairs_num; i++)
				{
					if (mpEndPair->mask[i] != 1)continue;
					cv::Point3d spherePt1, spherePt2;

					valid &= mvFishCameras[index1].pModel->mapI2S(mpEndPair->points1[i], spherePt1);
					valid &= mvFishCameras[index2].pModel->mapI2S(mpEndPair->points2[i], spherePt2);

					if (!valid)return false;

					Rotation ringRotation(ringR);
					cv::Point3d spherePt1ByRot = RotatePoint(spherePt1, ringRotation);
					err.at<double>(idx, 0) = spherePt1ByRot.x - spherePt2.x;
					err.at<double>(idx + 1, 0) = spherePt1ByRot.y - spherePt2.y;
					err.at<double>(idx + 2, 0) = spherePt1ByRot.z - spherePt2.z;

					idx += 3;
				}
			}

			return true;
		}

		bool _calcJacobian(cv::Mat &jac) const
		{

			//jac.create(pairNum * 3, activeParamNum, CV_64F);
			jac.setTo(0);

			const double step = 1e-6;
			cv::Mat err1, err2;
			err1.create(mTotalPairs * 3, 1, CV_64F);
			err2.create(mTotalPairs * 3, 1, CV_64F);
			bool valid = true;

			for (size_t i = 0; i < mvpParameter.size(); i++)
			{
				double originValue = *(mvpParameter[i]);

				*(mvpParameter[i]) = originValue - step;
				_updateParameters(i);

				valid &= _calcError(err1);

				*(mvpParameter[i]) = originValue + step;
				_updateParameters(i);

				valid &= _calcError(err2);

				calcDeriv(err1, err2, 2 * step, jac.col(i));

				*(mvpParameter[i]) = originValue;
				_updateParameters(i);

				if (!valid)return false;
			}

			return true;
		}

		void _updateParameters(const int& idx) const
		{
			int cameraIdx = mvParamCameraIdx[idx];
			if (mvRotMask[idx])
			{
				std::shared_ptr<Rotation> &pRot = mvFishCameras[cameraIdx].pRot;
				pRot->updataRotation(pRot->axisAngle);
			}
			else
			{
				mvFishCameras[cameraIdx].pModel->updateFov();
			}
		}

		std::list<PairInfo> &mlPairInfos;
		std::list<PairInfo>::iterator mpEndPair;
		std::vector<FishCamera> &mvFishCameras;
		std::vector<double *> mvpParameter;
		std::vector<int> mvParamCameraIdx;
		std::vector<bool> mvRotMask;

		int mTotalPairs;

		//bRotationScheme is design to control the way to refine the Rotation
		//bRotationScheme == false means to optimize the rotation as ordinary parameters
		//bRotationScheme == true means to optimize the rotation by least-squared method
		bool mbRotationScheme;

		bool mbRing;
	};
	
}