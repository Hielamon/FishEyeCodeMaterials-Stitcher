#include <FishOptimizer.h>

namespace CircleFish
{
	void calcRotation(PairInfo &pairinfo, FishCamera &camera)
	{
		int inliers_num = pairinfo.inliers_num, count = 0;
		std::vector<cv::Point3d> threed1(inliers_num);
		std::vector<cv::Point3d> threed2(inliers_num);
		for (size_t i = 0; i < pairinfo.pairs_num; i++)
		{
			if (pairinfo.mask[i] == 0)continue;
			mapI2S(pairinfo.points1[i], camera, threed1[count]);
			mapI2S(pairinfo.points2[i], camera, threed2[count]);
			count++;
		}

		assert(inliers_num == count);

		cv::Mat H = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
		double *H_ptr = reinterpret_cast<double *>(H.data);

		for (size_t i = 0; i < inliers_num; i++)
		{
			H_ptr[0] += threed1[i].x * threed2[i].x;
			H_ptr[1] += threed1[i].x * threed2[i].y;
			H_ptr[2] += threed1[i].x * threed2[i].z;
			H_ptr[3] += threed1[i].y * threed2[i].x;
			H_ptr[4] += threed1[i].y * threed2[i].y;
			H_ptr[5] += threed1[i].y * threed2[i].z;
			H_ptr[6] += threed1[i].z * threed2[i].x;
			H_ptr[7] += threed1[i].z * threed2[i].y;
			H_ptr[8] += threed1[i].z * threed2[i].z;
		}

		cv::SVD svd;
		cv::Mat w, u, vt;
		svd.compute(H, w, u, vt);

		camera.R = vt.t()*u.t();
	}

	void calcDeriv(const cv::Mat &err1, const cv::Mat &err2, double h, cv::Mat res)
	{
		for (int i = 0; i < err1.rows; ++i)
			res.at<double>(i, 0) = (err2.at<double>(i, 0) - err1.at<double>(i, 0)) / h;
	}

	FishOptimizer::FishOptimizer()
	{
		m_mask.resize(7, true);
		m_num_param = 7;
	}
	
	FishOptimizer::~FishOptimizer() {}

	void FishOptimizer::setMask(std::vector<bool> &mask)
	{
		if (mask.size() != 6)
		{
			std::cout << "mask.size() != " << 6 << std::endl;
			return;
		}
		m_mask = mask;
		m_num_param = 0;
		for (size_t i = 0; i < m_mask.size(); i++)
			if (m_mask[i])m_num_param++;

	}

	bool FishOptimizer::setParameters(std::list<PairInfo> &pairinfos, std::vector<FishCamera> &cameras, bool sparse)
	{
		if (pairinfos.size() == cameras.size())
			m_is_ring = true;
		else if (pairinfos.size() == cameras.size() - 1)
			m_is_ring = false;
		else
			return false;

		double refuze_d;

		m_pairinfos = pairinfos;
		double total_weights = 0;
		{
			double refuse_r = cameras[0].radius * 0.02;
			for (std::list<PairInfo>::iterator iter = m_pairinfos.begin(), iter_end = m_pairinfos.end(); iter != iter_end; iter++)
			{
				if (sparse && iter->inliers_num > 30)
				{
					for (size_t j = 0; j < iter->pairs_num; j++)
					{
						if (iter->mask[j] != 1)continue;
						for (size_t k = j + 1; k < iter->pairs_num; k++)
						{
							if (iter->mask[k] != 1)continue;
							double dist = cv::norm(iter->points1[j] - iter->points1[k]);
							if (dist < refuse_r)
							{
								iter->mask[k] = 0;
								iter->inliers_num--;
							}
						}
					}
				}

				double pairs_weight = 1.0 / (iter->inliers_num);
				total_weights += pairs_weight;
				m_pairweight.push_back(pairs_weight);
			}
		}


		for (size_t i = 0; i < m_pairweight.size(); i++)
			m_pairweight[i] = 1.0;
		//m_pairweight[i] /= total_weights;

		m_cameras = cameras;
		m_ori_u0 = cameras[0].u0;
		m_ori_v0 = cameras[0].v0;

		//m_images = images;
		return true;
	}
	
	void FishOptimizer::optimizeProcess()
	{
		int total_pairs = 0;
		for (std::list<PairInfo>::iterator iter = m_pairinfos.begin(); iter != m_pairinfos.end(); iter++)
			total_pairs += iter->inliers_num;


		cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, DBL_EPSILON);

		std::vector<double> param_vect;
		if (m_mask[0])param_vect.push_back(m_cameras[0].fov);
		if (m_mask[1])param_vect.push_back(m_cameras[0].a);
		if (m_mask[2])param_vect.push_back(m_cameras[0].b);
		if (m_mask[3])param_vect.push_back(m_cameras[0].c);
		if (m_mask[4])param_vect.push_back(m_cameras[0].u0);
		if (m_mask[5])param_vect.push_back(m_cameras[0].v0);

		CvMat matParams = cvMat(m_num_param, 1, CV_64F, &param_vect[0]);

		CvLevMarq solver(m_num_param, 2 * (total_pairs)+2, term_criteria);
		cv::Mat err, jac;
		cvCopy(&matParams, solver.param);
		int iter = 0;
		for (;;)
		{
			const CvMat* _param = 0;
			CvMat* _jac = 0;
			CvMat* _err = 0;

			bool proceed = solver.update(_param, _jac, _err);

			cvCopy(_param, &matParams);
			for (size_t i = 0; i < m_cameras.size(); i++)
			{
				int idx = 0;
				if (m_mask[0])
				{
					m_cameras[i].fov = param_vect[idx];
					idx++;
				}
				if (m_mask[1])
				{
					m_cameras[i].a = param_vect[idx];
					idx++;
				}
				if (m_mask[2])
				{
					m_cameras[i].b = param_vect[idx];
					idx++;
				}
				if (m_mask[3])
				{
					m_cameras[i].c = param_vect[idx];
					idx++;
				}
				if (m_mask[4])
				{
					m_cameras[i].u0 = param_vect[idx];
					idx++;
				}
				if (m_mask[5])
				{
					m_cameras[i].v0 = param_vect[idx];
					idx++;
				}


				assert(idx = param_vect.size());

			}

			if (!proceed || !_err)
				break;

			if (_jac)
			{
				_calcJacobian(jac);
				CvMat tmp = jac;
				cvCopy(&tmp, _jac);
			}

			if (_err)
			{
				_calcError(err);
				iter++;
				CvMat tmp = err;
				cvCopy(&tmp, _err);
			}
		}
	}

	void FishOptimizer::getCameras(std::vector<FishCamera> &cameras)
	{
		cameras = m_cameras;
	}

	void FishOptimizer::_calcError(cv::Mat &err)
	{
		int total_pairs = 0;
		std::list<PairInfo>::iterator ring_iter;
		for (std::list<PairInfo>::iterator iter = m_pairinfos.begin(); iter != m_pairinfos.end(); iter++)
		{
			total_pairs += iter->inliers_num;
			ring_iter = iter;
		}
		if (!m_is_ring) ring_iter = m_pairinfos.end();

		FishCamera C = m_cameras[0];
		err.create(total_pairs * 2 + 2, 1, CV_64F);

		cv::Mat ring_R = cv::Mat::eye(3, 3, CV_64F);
		int idx = 0, pair_idx = 0;
		for (std::list<PairInfo>::iterator iter = m_pairinfos.begin(); iter != ring_iter; iter++)
		{
			int index2 = iter->index2;
			calcRotation(*iter, m_cameras[index2]);
			ring_R = m_cameras[index2].R * ring_R;

			double *R_temp = reinterpret_cast<double *>(m_cameras[index2].R.data);
			double pair_weight = m_pairweight[pair_idx];
			for (size_t i = 0; i < iter->pairs_num; i++)
			{
				if (iter->mask[i] != 1)continue;
				cv::Point3d threed1, threedR_2;
				mapI2S(iter->points1[i], C, threed1);

				threedR_2.x = R_temp[0] * threed1.x + R_temp[1] * threed1.y + R_temp[2] * threed1.z;
				threedR_2.y = R_temp[3] * threed1.x + R_temp[4] * threed1.y + R_temp[5] * threed1.z;
				threedR_2.z = R_temp[6] * threed1.x + R_temp[7] * threed1.y + R_temp[8] * threed1.z;

				cv::Point2d img2;
				mapS2I(threedR_2, C, img2);

				double dx = iter->points2[i].x - img2.x;
				double dy = iter->points2[i].y - img2.y;

				err.at<double>(idx, 0) = std::abs(dx)*pair_weight;
				err.at<double>(idx + 1, 0) = std::abs(dy)*pair_weight;

				idx += 2;
			}
			pair_idx++;
		}

		if (m_is_ring)
		{
			double weight = 1, pair_weight = m_pairweight[pair_idx];
			double *R_ptr = reinterpret_cast<double *>(ring_R.data);
			for (size_t i = 0; i < ring_iter->pairs_num; i++)
			{
				if (ring_iter->mask[i] != 1)continue;
				cv::Point3d threed1, threedR_2;
				mapI2S(ring_iter->points1[i], C, threed1);

				threedR_2.x = R_ptr[0] * threed1.x + R_ptr[1] * threed1.y + R_ptr[2] * threed1.z;
				threedR_2.y = R_ptr[3] * threed1.x + R_ptr[4] * threed1.y + R_ptr[5] * threed1.z;
				threedR_2.z = R_ptr[6] * threed1.x + R_ptr[7] * threed1.y + R_ptr[8] * threed1.z;

				cv::Point2d img2;
				mapS2I(threedR_2, C, img2);


				double dx = ring_iter->points2[i].x - img2.x;
				double dy = ring_iter->points2[i].y - img2.y;

				err.at<double>(idx, 0) = std::abs(dx*weight)*pair_weight;
				err.at<double>(idx + 1, 0) = std::abs(dy*weight)*pair_weight;

				idx += 2;
			}
		}


		assert(idx == total_pairs * 2);
		err.at<double>(idx, 0) = (C.u0 - m_ori_u0)/**(ring_inliers + origin_pairs)*1*/;
		err.at<double>(idx + 1, 0) = (C.v0 - m_ori_v0)/**(ring_inliers + origin_pairs)*1*/;
	}

	void FishOptimizer::_calcJacobian(cv::Mat &jac)
	{
		int total_pairs = 0;
		for (std::list<PairInfo>::iterator iter = m_pairinfos.begin(); iter != m_pairinfos.end(); iter++)
			total_pairs += iter->inliers_num;

		jac.create(total_pairs * 2 + 2, m_num_param, CV_64F);
		jac.setTo(0);

		FishCamera C = m_cameras[0];

		const double step = 1e-5;
		const double step_image = 1;
		cv::Mat err1, err2;
		int idx = 0;

		if (m_mask[0])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].fov = C.fov - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].fov = C.fov + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].fov = C.fov;

			idx++;

		}

		if (m_mask[1])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].a = C.a - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].a = C.a + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].a = C.a;

			idx++;
		}

		if (m_mask[2])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].b = C.b - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].b = C.b + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].b = C.b;

			idx++;
		}

		if (m_mask[3])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].c = C.c - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].c = C.c + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].c = C.c;

			idx++;
		}

		if (m_mask[4])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].u0 = C.u0 - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].u0 = C.u0 + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].u0 = C.u0;

			idx++;
		}

		if (m_mask[5])
		{
			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].v0 = C.v0 - step;

			_calcError(err1);

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].v0 = C.v0 + step;

			_calcError(err2);
			calcDeriv(err1, err2, 2 * step, jac.col(idx));

			for (size_t i = 0; i < m_cameras.size(); i++)
				m_cameras[i].v0 = C.v0;

			idx++;
		}

	}
}
