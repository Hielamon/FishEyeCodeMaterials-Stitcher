#include <EstimateFish.h>

namespace CircleFish
{

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

	EstimateFish::EstimateFish() {}
	EstimateFish::~EstimateFish() {}

	void EstimateFish::operator ()(std::vector<cv::Mat> &images, std::vector<FishCamera> &cameras, std::vector<int> &index)
	{
		int num_images = images.size();
		assert(num_images >= 2);
		double megapix = 1.0;
		double match_scale = min(1.0, sqrt(megapix * 1e6 / images[0].size().area()));
		cv::Size match_size(images[0].size().width*match_scale, images[0].size().height*match_scale);

		std::vector<cv::Mat> match_images(num_images);
		for (int i = 0; i < num_images; i++)
			cv::resize(images[i], match_images[i], match_size);

		sequencematcher = std::make_shared<SequenceMatcher>(SequenceMatcher::F_SIFT, 1.0 / match_scale);
		sequencematcher->process(match_images, pairinfos);
		index.resize(pairinfos.size() + 1);

		bad_threshold = 0.05;

		bool is_ring = _initCompute(pairinfos, cameras, index);
		//_showPairInfo(images, pairinfos, 1.0);
		m_is_ring = is_ring;
		if (is_ring)
			sequencematcher->getRingPair(pairinfos);

		if (is_ring)
		{
			std::list<PairInfo>::iterator ring_iter;
			for (std::list<PairInfo>::iterator iter = pairinfos.begin(); iter != pairinfos.end(); iter++)
				ring_iter = iter;
			FishCamera ring_camera(cameras[index[0]].pModel, std::make_shared<Rotation>(0, 0));
			_ransacRotation(*ring_iter, ring_camera, bad_threshold);
		}
		//_showPairInfo(images, pairinfos, 1.0);
		

		std::string logFileName;
		std::shared_ptr<CameraModel> &pModel = cameras[index[0]].pModel;
		std::vector<uchar> vMask(pModel->vpParameter.size(), 1);
		vMask[0] = vMask[1] = 0;

		_runOptimizer(pairinfos, cameras, vMask, index, true, logFileName, is_ring);

		//Optimizing togather with Rotation
		vMask[0] = vMask[1] = 1;
		_runOptimizer(pairinfos, cameras, vMask, index, false, logFileName, is_ring);
		
		_alignCameras(cameras, index);
		_stretchCameras(cameras);
		//std::cout << "radius = " << cameras[0].radius << std::endl;
		//std::cout << "a = " << cameras[0].a << std::endl;
		//std::cout << "b = " << cameras[0].b << std::endl;
		//std::cout << "c = " << cameras[0].c << std::endl;
		//std::cout << "fov = " << cameras[0].fov << std::endl;
		DrawPairInfos(images, pairinfos, true, 1.0);
		//_showPairInfo(images, pairinfos, 1.0);
	}

	//初始化计算相机相对姿态R矩阵，如果鱼眼图像覆盖360以上，或者说满足大致的4张90度转条件，返回true，否则返回false
	bool EstimateFish::_initCompute(std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, std::vector<int> &index)
	{
		assert(index.size() == cameras.size());
		assert(pairinfos.size() == cameras.size() || cameras.size() == pairinfos.size() + 1);
		index[0] = pairinfos.begin()->index1;
		int count = 1;
		//cv::Mat pre_R = cameras[index[0]].R.clone();

		double total_theta = 0;

		for (std::list<PairInfo>::iterator iter = pairinfos.begin(); iter != pairinfos.end(); iter++, count++)
		{
			int index2 = iter->index2;

			//for the stituation of close ring
			if (count == index.size())
			{
				//_ransacRotation(*iter, common_c, bad_threshold);
				break;
			}

			index[count] = index2;
			_ransacRotation(*iter, cameras[index2], bad_threshold);

			total_theta += cameras[index2].pRot->axisAngle[1];
		}

		return std::abs(total_theta) > CV_PI*1.1 ? true : false;
	}

	void EstimateFish::_ransacRotation(PairInfo &pairinfo, FishCamera &camera, double bad_thres)
	{
		std::vector<std::vector<double> > csp(pairinfo.pairs_num);
		std::vector<cv::Point3d> threed1(pairinfo.pairs_num);
		std::vector<cv::Point3d> threed2(pairinfo.pairs_num);
		for (size_t i = 0; i < pairinfo.pairs_num; i++)
		{
			camera.pModel->mapI2S(pairinfo.points1[i], threed1[i]);
			camera.pModel->mapI2S(pairinfo.points2[i], threed2[i]);
			csp[i].resize(6);
			csp[i][0] = threed1[i].x; csp[i][1] = threed1[i].y; csp[i][2] = threed1[i].z;
			csp[i][3] = threed2[i].x; csp[i][4] = threed2[i].y; csp[i][5] = threed2[i].z;
		}
		std::vector<char> inliers(pairinfo.pairs_num, 1);
		RansacRotation rr(bad_thres, 0.999, 2000, false);
		double inlier_rate = rr.run(csp, inliers);
		cv::Mat R;
		rr.getRotation(R);
		camera.pRot->updataRotation(R);
		
		pairinfo.mask = inliers;

		cv::Mat H = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
		double *H_ptr = reinterpret_cast<double *>(H.data);

		int total_inliers = 0;
		for (size_t i = 0; i < pairinfo.pairs_num; i++)
		{
			if (inliers[i] == 0)continue;
			H_ptr[0] += threed1[i].x * threed2[i].x;
			H_ptr[1] += threed1[i].x * threed2[i].y;
			H_ptr[2] += threed1[i].x * threed2[i].z;
			H_ptr[3] += threed1[i].y * threed2[i].x;
			H_ptr[4] += threed1[i].y * threed2[i].y;
			H_ptr[5] += threed1[i].y * threed2[i].z;
			H_ptr[6] += threed1[i].z * threed2[i].x;
			H_ptr[7] += threed1[i].z * threed2[i].y;
			H_ptr[8] += threed1[i].z * threed2[i].z;
			total_inliers++;
		}
		pairinfo.inliers_num = total_inliers;

		cv::SVD svd;
		cv::Mat w, u, vt;
		svd.compute(H, w, u, vt);

		R = vt.t()*u.t();
		camera.pRot->updataRotation(R);
	}

	void EstimateFish::_stretchCameras(std::vector<FishCamera> &cameras)
	{
		cv::Mat moment = cv::Mat::zeros(3, 3, CV_64F);
		cv::Mat img_k = cv::Mat::zeros(3, 1, CV_64F);
		img_k.at<double>(0, 0) = 1.0;
		for (size_t i = 0; i < cameras.size(); ++i)
		{
			cv::Mat R_temp = cameras[i].pRot->R.t();
			cv::Mat col = R_temp.col(2);

			moment += col * col.t();
			//img_k += R_temp.col(1);
		}

		cv::Mat eigen_vals, eigen_vecs;
		cv::eigen(moment, eigen_vals, eigen_vecs);

		cv::Mat rg1 = eigen_vecs.row(2).t();
		cv::Mat rg0 = rg1.cross(img_k);
		double rg0_norm = norm(rg0);
		if (rg0_norm <= DBL_MIN)
		{
			return;
		}
		rg0 /= rg0_norm;
		cv::Mat rg2 = rg0.cross(rg1);

		cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
		cv::Mat tmp = R.row(0);
		cv::Mat(rg0.t()).copyTo(tmp);
		tmp = R.row(1);
		cv::Mat(rg1.t()).copyTo(tmp);
		tmp = R.row(2);
		cv::Mat(rg2.t()).copyTo(tmp);


		R = R.t();
		for (size_t i = 0; i < cameras.size(); i++)
		{
			cameras[i].pRot->updataRotation(cameras[i].pRot->R * R);
		}
	}

	void EstimateFish::_alignCameras(std::vector<FishCamera> &cameras, std::vector<int> &index)
	{
		cv::Vec3d rvec(0.0, 1.0, 0.0);
		rvec *= (-CV_PI*0.5);
		cv::Mat rotate90;
		cv::Rodrigues(rvec, rotate90);
		cameras[index[0]].pRot->updataRotation(cameras[index[0]].pRot->R * rotate90);

		cv::Mat pre_R = cameras[index[0]].pRot->R.clone();

		for (size_t i = 1; i < index.size(); i++)
		{
			cameras[index[i]].pRot->updataRotation(cameras[index[i]].pRot->R * pre_R);
			pre_R = cameras[index[i]].pRot->R.clone();
		}
	}

	void EstimateFish::_showPairInfo(std::vector<cv::Mat> &images, std::list<PairInfo> &pairinfos, double scale)
	{
		size_t pair_num = pairinfos.size();
		std::string name = "pairs_";
		std::stringstream ss;
		int index = 0;

		for (std::list<PairInfo>::iterator iter = pairinfos.begin(); iter != pairinfos.end(); iter++)
		{
			cv::Mat result;
			cv::Mat left, right;
			images[iter->index1].copyTo(left);
			images[iter->index2].copyTo(right);
			cv::hconcat(left, right, result);

			for (size_t i = 0; i < iter->pairs_num; i++)
			{
				if (iter->mask[i] != 1)continue;
				uchar r = rand() % 255;
				uchar g = rand() % 255;
				uchar b = rand() % 255;
				cv::Scalar color(b, g, r);
				cv::circle(result, iter->points1[i] * scale, 6, color, -1);
				cv::Point2d pt2 = iter->points2[i] * scale;
				pt2.x += left.cols;
				cv::line(result, iter->points1[i] * scale, pt2, color, 3);
				cv::circle(result, pt2, 6, color, -1);
			}
			ss << index;
			cv::imwrite(name + ss.str() + ".jpg", result);
			ss.str("");
			index++;
		}
	}
	
	void EstimateFish::_runOptimizer(std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, std::vector<uchar> &vMask, std::vector<int> &index,
									 bool bRotationScheme, std::string &logFileName, bool is_ring)
	{
		cv::Ptr<FishCameraRefineCallback> cb = new FishCameraRefineCallback(pairinfos, cameras, vMask, bRotationScheme);
		cv::Ptr<cv::LMSolver> levmarpPtr = customCreateLMSolver(cb, 200, 1e-15, 1e-15, logFileName);
		
		std::shared_ptr<CameraModel> &pModel = cameras[index[0]].pModel;
		assert(vMask.size() == pModel->vpParameter.size());


		int rotParamNum = bRotationScheme ? 0 : 3 * (index.size() - 1);
		int cameraParamNum = 0;
		std::for_each(vMask.begin(), vMask.end(), [&](uchar &maski) {
			if (maski == 1)cameraParamNum++;
		});

		cv::Mat param(cameraParamNum + rotParamNum, 1, CV_64FC1);
		{
			int idx = 0; 
			for (size_t i = 0; i < vMask.size(); i++)
			{
				if (vMask[i] == 1)
				{
					param.at<double>(idx++, 0) = *(pModel->vpParameter[i]);
				}
			}

			if (!bRotationScheme)
			{
				std::list<PairInfo>::iterator pEndPair;
				for (auto iter = pairinfos.begin(); iter != pairinfos.end(); iter++)
					pEndPair = iter;
				if (!is_ring)pEndPair = pairinfos.end();
				
				for (auto iter = pairinfos.begin(); iter != pEndPair; iter++)
				{
					int index2 = iter->index2;
					const std::shared_ptr<Rotation> & pRot = cameras[index2].pRot;
					for (size_t j = 0; j < 3; j++)
					{
						param.at<double>(idx + j, 0) = pRot->axisAngle[j];
					}
					idx += 3;
				}
			}
		}

		levmarpPtr->run(param);
	}
}

