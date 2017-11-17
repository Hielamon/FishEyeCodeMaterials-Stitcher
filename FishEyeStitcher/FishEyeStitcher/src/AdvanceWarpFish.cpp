#include <AdvanceWarpFish.h>

namespace CircleFish
{
	AdvanceWarpFish::AdvanceWarpFish(int sphere_height, bool is_tunning)
	{
		m_sphere_height = sphere_height;
		//std::cout << "m_sphere_height = " << m_sphere_height << std::endl;
		m_is_tunning = is_tunning;
	};
	AdvanceWarpFish::~AdvanceWarpFish() {}

	void AdvanceWarpFish::process(const std::vector<cv::Mat> &src_arr, std::vector<FishCamera> &cameras,
		std::vector<int> &index, bool is_ring, std::vector<cv::Mat> &blend_warpeds,
		std::vector<cv::Mat> &blend_warped_masks, std::vector<cv::Point>& blend_corners)
	{
		assert(!cameras.empty() && !index.empty());
		assert(cameras.size() == index.size());
		assert(src_arr.size() > 1 && src_arr.size() == cameras.size() && src_arr.size() == index.size());
		if (!blend_warpeds.empty())blend_warpeds.clear();
		if (!blend_warped_masks.empty())blend_warped_masks.clear();
		if (!blend_corners.empty())blend_corners.clear();

		m_is_ring = is_ring;

		_initWarp(src_arr, cameras, index, blend_warpeds, blend_warped_masks, blend_corners, result_warped_rois);

		getoverlap(result_warped_rois, init_lap_rois, init_lap_idx, is_ring);

		if (!m_is_tunning)return;
		std::vector<std::vector<cv::Rect>> resize_rois(blend_warpeds.size());

		ORBextractor orbextractor(2000, 1.2, 1, 20, 5);

		std::stringstream ss;
		for (size_t i = 0; i < init_lap_rois.size(); i++)
		{
			int idx1 = init_lap_idx[i].first;
			int idx2 = init_lap_idx[i].second;
			cv::Rect &roi1 = init_lap_rois[i].first;
			cv::Rect &roi2 = init_lap_rois[i].second;
			cv::Mat roi1_img = blend_warpeds[idx1](roi1);
			cv::Mat roi2_img = blend_warpeds[idx2](roi2);

			cv::Mat roi1_mask = blend_warped_masks[idx1](roi1);
			cv::Mat roi2_mask = blend_warped_masks[idx2](roi2);

			size_t nRowNum = 2 * roi1.height / roi1.width;
			FineMapping finemapping(&orbextractor);
			FineMapping::MapAndX mAndx1, mAndx2;
			finemapping(roi1_img, roi2_img, mAndx1, mAndx2,roi1_mask,roi2_mask);

			bool first_right = roi1.br().x == blend_warpeds[idx1].cols;
			cv::Rect &right_roi = first_right ? roi1 : roi2;
			int &right_idx = first_right ? idx1 : idx2;
			FineMapping::MapAndX &right_mAndx = first_right ? mAndx1 : mAndx2;

			bool second_left = roi2.x == 0;
			cv::Rect &left_roi = second_left ? roi2 : roi1;
			int &left_idx = second_left ? idx2 : idx1;
			FineMapping::MapAndX &left_mAndx = second_left ? mAndx2 : mAndx1;

			if (!right_mAndx.first.empty())
			{
				int add_width = right_roi.x >= right_roi.width ? right_roi.width : right_roi.x;
				cv::Rect right_roi_add(right_roi.x - add_width, right_roi.y, right_roi.width + add_width, right_roi.height);
				cv::Mat cp1 = blend_warpeds[right_idx](right_roi_add), cp1_mask = blend_warped_masks[right_idx](right_roi_add);
				cv::Mat cp1_t(cp1.size(), cp1.type()), cp1_mask_t(cp1_mask.size(), cp1_mask.type());

				int max_black_x = -right_mAndx.second[0];
				cv::Mat map_add(right_roi_add.size(), CV_32FC2), map1, map2;

				cv::Vec2f shift1;
				for (int m = 0; m < cp1_t.rows; m++)
				{
					cv::Vec2f *pmapRow = (cv::Vec2f*)right_mAndx.first.ptr(m);
					cv::Vec2f *pmapRow_add = (cv::Vec2f*)map_add.ptr(m);
					for (int n = cp1_t.cols - 1; n >= 0; n--)
					{
						cv::Vec2f aim_point;
						if (n >= right_roi.width)
						{
							shift1 = pmapRow[n - right_roi.width];
							aim_point = cv::Vec2f(n + shift1[0], m + shift1[1]);
						}
						else
						{
							double ratio = n / (double)(right_roi_add.width - right_roi.width);
							aim_point = cv::Vec2f(n + shift1[0] * ratio, m + shift1[1] * ratio);
						}

						pmapRow_add[n] = aim_point;
					
					}
				}



				if (i == 3 && 0)
				{
					ss.str("");
					ss << "overlap_right_origin_" << i << ".jpg";
					cv::imwrite(ss.str(), cp1);

					//draw grid
					int colDiv = 8;
					int widthDiv = cp1.cols / colDiv;
					int rowDiv = cp1.rows / widthDiv;
					cv::Scalar color(0, 0, 255);
					int thickness = 1;
					for (size_t j = 1; j <= colDiv; j++)
					{
						cv::line(cp1, cv::Point(j*widthDiv, 0), cv::Point(j*widthDiv, cp1.rows - 1), color, thickness);
					}
					for (size_t j = 1; j <= rowDiv; j++)
					{
						cv::line(cp1, cv::Point(0, j*widthDiv), cv::Point(cp1.cols - 1, j*widthDiv), color, thickness);
					}

				}

				cv::convertMaps(map_add, cv::Mat(), map1, map2, CV_16SC2);
				cv::remap(cp1, cp1_t, map1, map2, CV_INTER_LINEAR);
				cv::remap(cp1_mask, cp1_mask_t, map1, map2, CV_INTER_LINEAR);

				if (i == 3)
				{
					ss.str("");
					ss << "overlap_right_" << i << ".jpg";
					cv::imwrite(ss.str(), cp1);
					ss.str("");
					ss << "overlap_right_warped_" << i << ".jpg";
					cv::imwrite(ss.str(), cp1_t);
				}
				

				cp1_t.copyTo(blend_warpeds[right_idx](right_roi_add));
				cp1_mask_t.copyTo(blend_warped_masks[right_idx](right_roi_add));

				if (max_black_x > 0)
				{
					cv::Rect valid_roi(0, 0, blend_warpeds[right_idx].cols - 1 - max_black_x, blend_warpeds[right_idx].rows);
					resize_rois[right_idx].push_back(valid_roi);
				}
			}

			if (!left_mAndx.first.empty())
			{
				int add_width = left_roi.width * 2 <= blend_warpeds[left_idx].cols ? left_roi.width : blend_warpeds[left_idx].cols - left_roi.width;
				cv::Rect left_roi_add(left_roi.x, left_roi.y, left_roi.width + add_width, left_roi.height);
				cv::Mat cp2 = blend_warpeds[left_idx](left_roi_add), cp2_mask = blend_warped_masks[left_idx](left_roi_add);
				cv::Mat cp2_t(cp2.size(), cp2.type()), cp2_mask_t(cp2_mask.size(), cp2_mask.type());

				int min_black_x = -left_mAndx.second[1];

				cv::Mat map_add(left_roi_add.size(), CV_32FC2), map1, map2;

				cv::Vec2f shift2;
				for (int m = 0; m < cp2_t.rows; m++)
				{
					cv::Vec2f *pmapRow = (cv::Vec2f*)left_mAndx.first.ptr(m);
					cv::Vec2f *pmapRow_add = (cv::Vec2f*)map_add.ptr(m);
					for (int n = 0; n < cp2_t.cols; n++)
					{
						cv::Vec2f aim_point;
						if (n < left_roi.width)
						{
							shift2 = pmapRow[n];
							aim_point = cv::Vec2f(n + shift2[0], m + shift2[1]);
						}
						else
						{
							double ratio = (left_roi_add.width - n - 1) / (double)(left_roi_add.width - left_roi.width);
							aim_point = cv::Vec2f(n + shift2[0] * ratio, m + shift2[1] * ratio);
						}

						pmapRow_add[n] = aim_point;
					}
				}

				if (i == 3 && 0)
				{
					ss.str("");
					ss << "overlap_left_origin_" << i << ".jpg";
					cv::imwrite(ss.str(), cp2);

					//draw grid
					int colDiv = 8;
					int widthDiv = cp2.cols / colDiv;
					int rowDiv = cp2.rows / widthDiv;
					cv::Scalar color(0, 0, 255);
					int thickness = 1;
					for (size_t j = 1; j <= colDiv; j++)
					{
						cv::line(cp2, cv::Point(j*widthDiv, 0), cv::Point(j*widthDiv, cp2.rows - 1), color, thickness);
					}
					for (size_t j = 1; j <= rowDiv; j++)
					{
						cv::line(cp2, cv::Point(0, j*widthDiv), cv::Point(cp2.cols - 1, j*widthDiv), color, thickness);
					}

				}

				cv::convertMaps(map_add, cv::Mat(), map1, map2, CV_16SC2);
				cv::remap(cp2, cp2_t, map1, map2, CV_INTER_LINEAR);
				cv::remap(cp2_mask, cp2_mask_t, map1, map2, CV_INTER_LINEAR);

				if (i == 3)
				{
					ss.str("");
					ss << "overlap_left_" << i << ".jpg";
					cv::imwrite(ss.str(), cp2);
					ss.str("");
					ss << "overlap_left_warped_" << i << ".jpg";
					cv::imwrite(ss.str(), cp2_t);
				}
				

				cp2_t.copyTo(blend_warpeds[left_idx](left_roi_add));
				cp2_mask_t.copyTo(blend_warped_masks[left_idx](left_roi_add));

				if (min_black_x < 0)
				{
					cv::Rect valid_roi(-min_black_x, 0, blend_warpeds[left_idx].cols - 1 + min_black_x, blend_warpeds[left_idx].rows);
					resize_rois[left_idx].push_back(valid_roi);
						
				}
			}



		}

			
		std::map<size_t, cv::Vec2i> idxPairs;
		size_t idx_count = 0;
		for (size_t i = 0; i < result_warped_rois.size(); i++)
		{
			for (size_t j = 0; j < result_warped_rois[i].size(); j++)
			{
				idxPairs[idx_count] = cv::Vec2i(i, j);
				idx_count++;
			}
		}
		assert(idx_count == resize_rois.size());

		//retain the valid regions
		for (size_t i = 0; i < resize_rois.size(); i++)
		{
			if (resize_rois[i].size() == 0)continue;
			cv::Point tl_temp = resize_rois[i][0].tl(), tl = tl_temp;
			cv::Point br_temp = resize_rois[i][0].br(), br = br_temp;
			for (size_t j = 1; j < resize_rois[i].size(); j++)
			{
				tl_temp = resize_rois[i][j].tl();
				br_temp = resize_rois[i][j].br();

				if (tl_temp.x > tl.x)tl.x = tl_temp.x;
				if (tl_temp.y > tl.y)tl.y = tl_temp.y;
				if (br_temp.x < br.x)br.x = br_temp.x;
				if (br_temp.y < br.y)br.y = br_temp.y;
			}

			blend_corners[i] += tl;
			cv::Rect valid_roi(tl, br);
			blend_warpeds[i] = blend_warpeds[i](valid_roi);
			blend_warped_masks[i] = blend_warped_masks[i](valid_roi);
				
			cv::Rect result_roi(blend_corners[i], blend_corners[i] + cv::Point(valid_roi.width, valid_roi.height));
			cv::Vec2i idx_pos = idxPairs[i];
			result_warped_rois[idx_pos[0]][idx_pos[1]] = result_roi;
		}
	}

	void AdvanceWarpFish::getoverlap(std::vector<std::vector<cv::Rect>> &rois,
		std::vector<std::pair<cv::Rect, cv::Rect>> &lap_rois,
		std::vector<std::pair<int, int>> &lap_idx, bool is_ring)
	{

		int idx_count = 0;

		for (size_t i = 1, prev_idx = 0; i < rois.size(); i++, prev_idx++)
		{
			std::vector<cv::Rect> roi_pair_temp;
			std::vector<std::pair<int, int>> idx;
			int idx_choose;
			for (size_t j = 0; j < rois[prev_idx].size(); j++)
			{
				for (size_t k = 0; k < rois[i].size(); k++)
				{
					cv::Point tl, br;
					cv::Rect &ori1 = rois[prev_idx][j];
					cv::Rect &ori2 = rois[i][k];
					tl.x = ori1.x > ori2.x ? ori1.x : ori2.x;
					tl.y = ori1.y > ori2.y ? ori1.y : ori2.y;

					br.x = ori1.br().x < ori2.br().x ? ori1.br().x : ori2.br().x;
					br.y = ori1.br().y < ori2.br().y ? ori1.br().y : ori2.br().y;

					if (tl.x >= br.x || tl.y >= br.y)continue;

					roi_pair_temp.push_back(cv::Rect(tl, br));
					idx.push_back(std::pair<int, int>(j, k));
				}
			}
			if (roi_pair_temp.size() > 2)
			{
				std::cout << "It's not reasonable when there are more than 2 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 0)
			{
				std::cout << "It's not reasonable when there is 0  overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 2)
			{
				if (roi_pair_temp[0].area() > roi_pair_temp[1].area())
					idx_choose = 0;
				else
					idx_choose = 1;
			}
			else
				idx_choose = 0;

			cv::Rect &ori1 = rois[prev_idx][idx[idx_choose].first];
			cv::Rect &ori2 = rois[i][idx[idx_choose].second];
			lap_idx.push_back(std::pair<int, int>(idx[idx_choose].first + idx_count, rois[prev_idx].size() + idx[idx_choose].second + idx_count));
			cv::Point tl1 = roi_pair_temp[idx_choose].tl() - ori1.tl();
			cv::Point tl2 = roi_pair_temp[idx_choose].tl() - ori2.tl();
			cv::Rect roi1_(tl1, roi_pair_temp[idx_choose].size());
			cv::Rect roi2_(tl2, roi_pair_temp[idx_choose].size());
			lap_rois.push_back(std::pair<cv::Rect, cv::Rect>(roi1_, roi2_));
			idx_count += rois[prev_idx].size();
		}



		if (is_ring)
		{
			int last_idx = rois.size() - 1;
			std::vector<cv::Rect> roi_pair_temp;
			std::vector<std::pair<int, int>> idx;
			int idx_choose;
			for (size_t j = 0; j < rois[0].size(); j++)
			{
				for (size_t k = 0; k < rois[last_idx].size(); k++)
				{
					cv::Point tl, br;
					cv::Rect &ori1 = rois[0][j];
					cv::Rect &ori2 = rois[last_idx][k];
					tl.x = ori1.x > ori2.x ? ori1.x : ori2.x;
					tl.y = ori1.y > ori2.y ? ori1.y : ori2.y;

					br.x = ori1.br().x < ori2.br().x ? ori1.br().x : ori2.br().x;
					br.y = ori1.br().y < ori2.br().y ? ori1.br().y : ori2.br().y;

					if (tl.x >= br.x || tl.y >= br.y)continue;

					roi_pair_temp.push_back(cv::Rect(tl, br));
					idx.push_back(std::pair<int, int>(j, k));
				}
			}
			if (roi_pair_temp.size() > 2)
			{
				std::cout << "It's not reasonable when there are more than 2 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 0)
			{
				std::cout << "It's not reasonable when there is 0 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 2)
			{
				if (roi_pair_temp[0].area() > roi_pair_temp[1].area())
					idx_choose = 0;
				else
					idx_choose = 1;
			}
			else
				idx_choose = 0;

			cv::Rect &ori1 = rois[0][idx[idx_choose].first];
			cv::Rect &ori2 = rois[last_idx][idx[idx_choose].second];
			lap_idx.push_back(std::pair<int, int>(idx[idx_choose].first, idx[idx_choose].second + idx_count));
			cv::Point tl1 = roi_pair_temp[idx_choose].tl() - ori1.tl();
			cv::Point tl2 = roi_pair_temp[idx_choose].tl() - ori2.tl();
			cv::Rect roi1_(tl1, roi_pair_temp[idx_choose].size());
			cv::Rect roi2_(tl2, roi_pair_temp[idx_choose].size());
			lap_rois.push_back(std::pair<cv::Rect, cv::Rect>(roi1_, roi2_));
		}
	}

	void AdvanceWarpFish::getoverlap(std::vector<std::vector<cv::Rect2d>> &rois,
		std::vector<std::pair<cv::Rect2d, cv::Rect2d>> &lap_rois,
		std::vector<std::pair<int, int>> &lap_idx, bool is_ring)
	{

		int idx_count = 0;

		for (size_t i = 1, prev_idx = 0; i < rois.size(); i++, prev_idx++)
		{
			std::vector<cv::Rect2d> roi_pair_temp;
			std::vector<std::pair<int, int>> idx;
			int idx_choose;
			for (size_t j = 0; j < rois[prev_idx].size(); j++)
			{
				for (size_t k = 0; k < rois[i].size(); k++)
				{
					cv::Point2d tl, br;
					cv::Rect2d &ori1 = rois[prev_idx][j];
					cv::Rect2d &ori2 = rois[i][k];
					tl.x = ori1.x > ori2.x ? ori1.x : ori2.x;
					tl.y = ori1.y > ori2.y ? ori1.y : ori2.y;

					br.x = ori1.br().x < ori2.br().x ? ori1.br().x : ori2.br().x;
					br.y = ori1.br().y < ori2.br().y ? ori1.br().y : ori2.br().y;

					if (tl.x >= br.x || tl.y >= br.y)continue;

					roi_pair_temp.push_back(cv::Rect2d(tl, br));
					idx.push_back(std::pair<int, int>(j, k));
				}
			}
			if (roi_pair_temp.size() > 2)
			{
				std::cout << "It's not reasonable when there are more than 2 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 0)
			{
				std::cout << "It's not reasonable when there is 0 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 2)
			{
				if (roi_pair_temp[0].area() > roi_pair_temp[1].area())
					idx_choose = 0;
				else
					idx_choose = 1;
			}
			else
				idx_choose = 0;

			cv::Rect2d &ori1 = rois[prev_idx][idx[idx_choose].first];
			cv::Rect2d &ori2 = rois[i][idx[idx_choose].second];
			lap_idx.push_back(std::pair<int, int>(idx[idx_choose].first + idx_count, rois[prev_idx].size() + idx[idx_choose].second + idx_count));
			cv::Point2d tl1 = roi_pair_temp[idx_choose].tl() - ori1.tl();
			cv::Point2d tl2 = roi_pair_temp[idx_choose].tl() - ori2.tl();
			cv::Rect2d roi1_(tl1, roi_pair_temp[idx_choose].size());
			cv::Rect2d roi2_(tl2, roi_pair_temp[idx_choose].size());
			lap_rois.push_back(std::pair<cv::Rect2d, cv::Rect2d>(roi1_, roi2_));
			idx_count += rois[prev_idx].size();
		}



		if (is_ring)
		{
			int last_idx = rois.size() - 1;
			std::vector<cv::Rect2d> roi_pair_temp;
			std::vector<std::pair<int, int>> idx;
			int idx_choose;
			for (size_t j = 0; j < rois[0].size(); j++)
			{
				for (size_t k = 0; k < rois[last_idx].size(); k++)
				{
					cv::Point2d tl, br;
					cv::Rect2d &ori1 = rois[0][j];
					cv::Rect2d &ori2 = rois[last_idx][k];
					tl.x = ori1.x > ori2.x ? ori1.x : ori2.x;
					tl.y = ori1.y > ori2.y ? ori1.y : ori2.y;

					br.x = ori1.br().x < ori2.br().x ? ori1.br().x : ori2.br().x;
					br.y = ori1.br().y < ori2.br().y ? ori1.br().y : ori2.br().y;

					if (tl.x >= br.x || tl.y >= br.y)continue;

					roi_pair_temp.push_back(cv::Rect2d(tl, br));
					idx.push_back(std::pair<int, int>(j, k));
				}
			}
			if (roi_pair_temp.size() > 2)
			{
				std::cout << "It's not reasonable when there are more than 2 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 0)
			{
				std::cout << "It's not reasonable when there is 0 overlap regions between two frames" << std::endl;
				exit(0);
			}
			else if (roi_pair_temp.size() == 2)
			{
				if (roi_pair_temp[0].area() > roi_pair_temp[1].area())
					idx_choose = 0;
				else
					idx_choose = 1;
			}
			else
				idx_choose = 0;

			cv::Rect2d &ori1 = rois[0][idx[idx_choose].first];
			cv::Rect2d &ori2 = rois[last_idx][idx[idx_choose].second];
			lap_idx.push_back(std::pair<int, int>(idx[idx_choose].first, idx[idx_choose].second + idx_count));
			cv::Point2d tl1 = roi_pair_temp[idx_choose].tl() - ori1.tl();
			cv::Point2d tl2 = roi_pair_temp[idx_choose].tl() - ori2.tl();
			cv::Rect2d roi1_(tl1, roi_pair_temp[idx_choose].size());
			cv::Rect2d roi2_(tl2, roi_pair_temp[idx_choose].size());
			lap_rois.push_back(std::pair<cv::Rect2d, cv::Rect2d>(roi1_, roi2_));
		}
	}

	void AdvanceWarpFish::_initWarp(const std::vector<cv::Mat> &src_arr, std::vector<FishCamera> &cameras,
		std::vector<int> &index, std::vector<cv::Mat> &blend_warpeds,
		std::vector<cv::Mat> &blend_warped_masks, std::vector<cv::Point>& blend_corners,
		std::vector<std::vector<cv::Rect>> &rois)
	{
		int num_image = src_arr.size();
		int sphere_mini = 500, half_shm = sphere_mini * 0.5;
		double sphere_mini_aspect = m_sphere_height / (double)sphere_mini;

		cv::Point2i center(cameras[0].pModel->u0, cameras[0].pModel->v0);
		double radius = cameras[0].pModel->maxRadius;
		cv::Mat origin_blend_mask;
		{
			origin_blend_mask = cv::Mat(src_arr[0].size(), CV_8UC1, cv::Scalar(0));
			cv::circle(origin_blend_mask, center, radius, cv::Scalar(255), -1);
		}

		double half_fov = cameras[0].pModel->fov * 0.5;
		int src_width = src_arr[0].cols;
		
		double r_theta = src_width - center.x> radius ? half_fov : half_fov*(src_width - center.x) / radius;
		double l_theta = center.x > radius ? half_fov : half_fov*center.x / radius;

		cv::Point3d right_pt(sin(r_theta), 0, cos(r_theta));
		cv::Point3d left_pt(-sin(l_theta), 0, cos(l_theta));
		cv::Point3d border_pt[3];
		cv::Point2d border_img[3];
		double shm_pi = sphere_mini / CV_PI;
			
		std::vector<std::vector<cv::Rect2d>> biggest_rois;

		for (size_t i = 0; i < num_image; i++)
		{
			int index_temp = index[i];
			{
				std::stringstream ss;
				std::string name = "src_";
				ss << name << i << ".jpg";
				cv::imwrite(ss.str(), src_arr[index_temp]);
			}
			
			double *R_ptr = (double *)cameras[index_temp].pRot->R.data;

			border_pt[0] = cv::Point3d(R_ptr[0] * right_pt.x + R_ptr[6] * right_pt.z, R_ptr[1] * right_pt.x + R_ptr[7] * right_pt.z, R_ptr[2] * right_pt.x + R_ptr[8] * right_pt.z);
			border_pt[1] = cv::Point3d(R_ptr[6], R_ptr[7], R_ptr[8]);
			border_pt[2] = cv::Point3d(R_ptr[0] * left_pt.x + R_ptr[6] * left_pt.z, R_ptr[1] * left_pt.x + R_ptr[7] * left_pt.z, R_ptr[2] * left_pt.x + R_ptr[8] * left_pt.z);
			for (size_t j = 0; j < 3; j++)
			{
				double theta = atan2(border_pt[j].x, border_pt[j].z);
				double x2_z2 = border_pt[j].x * border_pt[j].x + border_pt[j].z * border_pt[j].z;
				double phi = atan2(border_pt[j].y, sqrt(x2_z2));
				border_img[j].x = theta * shm_pi + half_shm;
				border_img[j].y = -phi * shm_pi + half_shm;
			}

			std::vector<cv::Rect2d> roi_temp;

			if (border_img[0].x < border_img[2].x)
			{
				double min_x = border_img[0].x, max_x = border_img[2].x;
				double roi_width = min_x + sphere_mini*0.5;
				if (roi_width > sphere_mini * 0.05)
				{
					roi_temp.push_back(cv::Rect2d(-sphere_mini*0.5, 0, roi_width, sphere_mini));
				}
				roi_width = sphere_mini*1.5 - max_x;
				if (roi_width > sphere_mini * 0.05)
				{
					roi_temp.push_back(cv::Rect2d(max_x, 0, roi_width, sphere_mini));
				}
			}
			else
			{
				roi_temp.push_back(cv::Rect2d(border_img[2].x, 0, border_img[0].x - border_img[2].x, sphere_mini));
			}

			biggest_rois.push_back(roi_temp);
			
			for(size_t j = 0; j < roi_temp.size(); j++)
			{
				cv::Mat map,map1,map2;
				cv::Mat warped_temp;
				buildMap(roi_temp[j], sphere_mini, cameras[index_temp], map);
				int n_width = std::ceil(roi_temp[j].width * sphere_mini_aspect);
				int n_lp = roi_temp[j].x * sphere_mini_aspect;
				cv::resize(map, map, cv::Size(n_width, roi_temp[j].height * sphere_mini_aspect));
				cv::convertMaps(map, cv::Mat(), map1, map2, CV_16SC2);
				cv::remap(src_arr[index_temp], warped_temp, map1, map2, cv::INTER_LINEAR);
				
				std::stringstream ss;
				std::string name = "warped_";
				ss << name << i << "_"<< j<<".jpg";
				cv::imwrite(ss.str(),warped_temp);
			}
		}

		std::vector<std::pair<cv::Rect2d, cv::Rect2d>> lap_rois;
		std::vector<std::pair<int, int>> lap_idx;
		getoverlap(biggest_rois, lap_rois, lap_idx, m_is_ring);

		std::map<size_t, cv::Vec2i> idxPairs;
		size_t idx_count = 0;
		for (size_t i = 0; i < biggest_rois.size(); i++)
		{
				
			for (size_t j = 0; j < biggest_rois[i].size(); j++, idx_count++)
			{
				idxPairs[idx_count] = cv::Vec2i(i, j);
			}
		}

		assert(lap_rois.size() == lap_idx.size());
		assert(lap_rois.size() == biggest_rois.size());

		double max_lap_width = (250.0 / 4000) * sphere_mini;

		std::vector<bool> valid_mask(idx_count, false);
		std::vector<std::vector<cv::Rect2d>> biggest_rois_bak = biggest_rois;

		for (size_t i = 0; i < lap_rois.size(); i++)
		{
			int first_idx = lap_idx[i].first;
			int second_idx = lap_idx[i].second;
			if (!valid_mask[first_idx])valid_mask[first_idx] = true;
			if (!valid_mask[second_idx])valid_mask[second_idx] = true;

			double cur_roi_width = lap_rois[i].first.width;
			if (cur_roi_width <= max_lap_width)continue;

			cv::Vec2i ft_cd = idxPairs[first_idx];
			cv::Vec2i sd_cd = idxPairs[second_idx];
			bool first_right = lap_rois[i].first.br().x == biggest_rois_bak[ft_cd[0]][ft_cd[1]].width;
			cv::Rect2d &right_roi = first_right ? biggest_rois[ft_cd[0]][ft_cd[1]] : biggest_rois[sd_cd[0]][sd_cd[1]];
			bool second_left = lap_rois[i].second.tl().x == 0;
			cv::Rect2d &left_roi = second_left ? biggest_rois[sd_cd[0]][sd_cd[1]] : biggest_rois[ft_cd[0]][ft_cd[1]];

			double reduce_x = (cur_roi_width - max_lap_width) * 0.5;
			right_roi.width -= reduce_x;
			left_roi.x += reduce_x;
			left_roi.width -= reduce_x;
		}

		idx_count = 0;
		for (size_t i = 0; i < biggest_rois.size(); i++)
		{
			cv::Mat warped_temp;
			cv::Mat map, map1, map2;
			int index_temp = index[i];
			std::vector<cv::Rect> nroi_temp;
			std::vector<cv::Rect2d> &roi_temp = biggest_rois[i];
			for (size_t j = 0; j < roi_temp.size(); j++, idx_count++)
			{
				if (!valid_mask[idx_count])continue;

				buildMap(roi_temp[j], sphere_mini, cameras[index_temp], map);
				int n_width = std::ceil(roi_temp[j].width * sphere_mini_aspect);
				int n_lp = roi_temp[j].x * sphere_mini_aspect;
				cv::resize(map, map, cv::Size(n_width, roi_temp[j].height * sphere_mini_aspect));
				cv::convertMaps(map, cv::Mat(), map1, map2, CV_16SC2);
				cv::remap(src_arr[index_temp], warped_temp, map1, map2, cv::INTER_LINEAR);
				blend_warpeds.push_back(warped_temp);

				std::stringstream ss;
				std::string name = "warped_clipped_";
				ss << name << i << "_" << j << ".jpg";
				cv::imwrite(ss.str(), warped_temp);

				cv::remap(origin_blend_mask, warped_temp, map1, map2, cv::INTER_NEAREST);
				blend_warped_masks.push_back(warped_temp);
				cv::Point tl = cv::Point(n_lp, 0), br = tl + cv::Point(warped_temp.cols, warped_temp.rows);
				blend_corners.push_back(tl);
				nroi_temp.push_back(cv::Rect(tl, br));
			}
			rois.push_back(nroi_temp);
		}
	}

}
