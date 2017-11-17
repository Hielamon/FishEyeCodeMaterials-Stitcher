#include <FishStitcher.h>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace FishEye;

namespace CircleFish
{
	bool SaveWarpedInfos(const std::vector<cv::Mat>& blend_warpeds, const std::vector<cv::Mat>& blend_warped_masks,
						const std::vector<cv::Point> &blend_corners, const std::string &fName)
	{
		assert(blend_corners.size() == blend_warpeds.size() && blend_corners.size() == blend_warped_masks.size());
		std::ofstream fs(fName, std::ios::out);
		if (!fs.is_open())return false;
		
		std::stringstream ioStr;
		std::string prefix = "Warped_", prefix_mask = "WarpedMask_", name;
		for (size_t i = 0; i < blend_warpeds.size(); i++)
		{
			ioStr.str("");
			ioStr << prefix << std::setw(4) << std::setfill('0') << i << ".jpg";
			name = ioStr.str();
			fs << name << " ";
			cv::imwrite(name, blend_warpeds[i]);

			ioStr.str("");
			ioStr << prefix_mask << std::setw(4) << std::setfill('0') << i << ".jpg";
			name = ioStr.str();
			fs << name << " ";
			cv::imwrite(name, blend_warped_masks[i]);

			fs << blend_corners[i].x << " " << blend_corners[i].y << std::endl;
		}

		fs.close();
	
		return true;
	}

	FishStitcher::FishStitcher(const size_t result_height)
	{
		m_black = 15;
		m_result_size = cv::Size(result_height*2, result_height);
		m_findSeam_size = cv::Size(300, 300);
		m_blend_size = cv::Size(result_height, result_height);

		m_expos_comp_type = cv::detail::ExposureCompensator::GAIN_BLOCKS;
		m_blend_type = cv::detail::Blender::MULTI_BAND;
		m_blend_strength = 10;
	}

	FishStitcher::~FishStitcher() {}

	bool FishStitcher::operator ()(std::vector<cv::Mat> images, bool do_fine_tune, cv::Mat &result)
	{
#if COST_TIME
		double duration;
		duration = static_cast<double>(cv::getTickCount());
#endif

		int num_images = images.size();
		if (num_images < 2)
		{
			std::cout << "the number of images for FishStitcher least than 2" << std::endl;
			return false;
		}

		std::cout << "\n.............Start calib cameras............." << std::endl;

		//Get the circle fisheye image's circle region
		cv::Point2d circle_center;
		double radius;
		_getCircleRegion(images[0], circle_center, radius);

		//Initialize the cameras
		std::shared_ptr<Equidistant> baseModel = std::make_shared<Equidistant>(0, 0, 1, CV_PI);
		double f = radius / baseModel->maxRadius;
		cv::Vec2d args(0.976517, 1.743803);
		std::string modelName = "GeyerModel";
		/*cv::Vec2d args(1.000000, 0.000000);
		std::string modelName = "PolynomialAngle";*/

		std::shared_ptr<CameraModel> pModel = std::static_pointer_cast<CameraModel>(
			createCameraModel(modelName, circle_center.x, circle_center.y, f, 0, radius, args[0], args[1]));
		std::vector<FishCamera> cameras;
		for (size_t i = 0; i < num_images; i++)
		{
			FishCamera camera(pModel, std::make_shared<Rotation>(0, 0));
			cameras.push_back(camera);
		}

		//Estimate the intrinsic and external cameras' parameters
		EstimateFish estiamte;
		std::vector<int> index;
		estiamte(images, cameras, index);

#if COST_TIME
		duration = static_cast<double>(cv::getTickCount()) - duration;
		std::cout << "Camera calibration cost time : " << 1000 * (duration / cv::getTickFrequency()) << " ms " << std::endl;
		duration = static_cast<double>(cv::getTickCount());
#endif

		std::cout << "\n.............Warp and FineMapping............." << std::endl;

		//Declare variable for blending
		std::vector<cv::Mat> blend_warpeds;
		std::vector<cv::Mat> blend_warped_masks;
		std::vector<cv::Point> blend_corners;


		//directly get the warped images for blending
		AdvanceWarpFish advancewarpfish(m_blend_size.height, do_fine_tune);
		advancewarpfish.process(images, cameras, index, estiamte.m_is_ring, blend_warpeds, blend_warped_masks, blend_corners);


#if COST_TIME
		duration = static_cast<double>(cv::getTickCount()) - duration;
		std::cout << "Warp and FineMapping cost time : " << 1000 * (duration / cv::getTickFrequency()) << " ms " << std::endl;
		duration = static_cast<double>(cv::getTickCount());
#endif

		//SaveWarpedInfos(blend_warpeds, blend_warped_masks, blend_corners, "warpedInfos.txt");

		//Finish the finally blend manipulation
		_blendCompute(blend_warpeds, blend_warped_masks, blend_corners, m_blend_size, result);

		//Clear images for save memory
		images.clear();

		_checkSize(result);

		return true;
	}

	void FishStitcher::_blendCompute(std::vector<cv::Mat>& blend_warpeds, std::vector<cv::Mat>& blend_warped_masks,
		std::vector<cv::Point> &blend_corners, cv::Size blend_size, cv::Mat &result)
	{
#if COST_TIME
		double duration = static_cast<double>(cv::getTickCount());
#endif
		std::cout << "\n.............Start blend process............." << std::endl;
		int num_images = blend_warpeds.size();
		double seam_aspect_blend = m_findSeam_size.height / (double)blend_size.height;
		std::vector<cv::Size> blend_sizes;
		for (size_t i = 0; i < num_images; i++)
			blend_sizes.push_back(blend_warpeds[i].size());

		std::vector<cv::UMat> seam_warped(num_images);
		std::vector<cv::UMat> seam_warped_f(num_images);
		std::vector<cv::UMat> seam_masks_warped(num_images);
		std::vector<cv::Point> seam_corners;

		for (size_t i = 0; i < num_images; i++)
		{
			m_findSeam_size = cv::Size(blend_warpeds[i].cols*seam_aspect_blend, blend_warpeds[i].rows*seam_aspect_blend);

			cv::resize(blend_warpeds[i], seam_warped[i], m_findSeam_size);
			cv::resize(blend_warped_masks[i], seam_masks_warped[i], m_findSeam_size);
			seam_warped[i].convertTo(seam_warped_f[i], CV_32F);
			seam_corners.push_back(blend_corners[i] * seam_aspect_blend);
		}

		cv::Ptr<cv::detail::ExposureCompensator> compensator = cv::detail::ExposureCompensator::createDefault(m_expos_comp_type);
		compensator->feed(seam_corners, seam_warped, seam_masks_warped);

		cv::Ptr<cv::detail::SeamFinder> seam_finder = cv::makePtr<cv::detail::GraphCutSeamFinder>(cv::detail::GraphCutSeamFinderBase::COST_COLOR);
		seam_finder->find(seam_warped_f, seam_corners, seam_masks_warped);

		std::vector<cv::Mat> seam_masks_mat(num_images);
		for (size_t i = 0; i < num_images; i++)
		{
			seam_masks_warped[i].copyTo(seam_masks_mat[i]);
		}

		seam_warped.clear();
		seam_warped_f.clear();

		cv::Ptr<cv::detail::Blender> blender = cv::detail::Blender::createDefault(m_blend_type);
		{
			cv::Size dst_sz = cv::detail::resultRoi(blend_corners, blend_sizes).size();
			double blend_width = sqrt(static_cast<double>(dst_sz.area())) * m_blend_strength / 100.f;
			if (blend_width < 1.f)
				blender = cv::detail::Blender::createDefault(cv::detail::Blender::NO);
			else if (m_blend_type == cv::detail::Blender::MULTI_BAND)
			{
				cv::detail::MultiBandBlender* mb = dynamic_cast<cv::detail::MultiBandBlender*>(blender.get());
				mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
			}
			else if (m_blend_type == cv::detail::Blender::FEATHER)
			{
				cv::detail::FeatherBlender* fb = dynamic_cast<cv::detail::FeatherBlender*>(blender.get());
				fb->setSharpness(1.f / blend_width);
			}
			blender->prepare(blend_corners, blend_sizes);
		}

		cv::Mat blend_mask_warped, blend_warped_s;
		cv::Mat dilated_mask, seam_mask;
		for (size_t i = 0; i < num_images; i++)
		{
			blend_mask_warped = blend_warped_masks[i];
			compensator->apply(i, blend_corners[i], blend_warpeds[i], blend_mask_warped);
			blend_warpeds[i].convertTo(blend_warped_s, CV_16S);
			dilate(seam_masks_warped[i], dilated_mask, cv::Mat());
			resize(dilated_mask, seam_mask, blend_mask_warped.size());
			blend_mask_warped = seam_mask/* & blend_mask_warped*/;
			blender->feed(blend_warped_s, blend_mask_warped, blend_corners[i]);
		}
		cv::Mat result_mask;
		blender->blend(result, result_mask);

#if COST_TIME
		duration = static_cast<double>(cv::getTickCount()) - duration;
		std::cout << "Blending cost time : " << 1000 * (duration / cv::getTickFrequency()) << " ms " << std::endl;
#endif
	}

	void FishStitcher::_getCircleRegion(cv::Mat &img, cv::Point2d &center, double &radius)
	{
		std::vector<std::vector<int> > circle_points;
		_getCircleEdgePoints(img, circle_points);

		RansacCircle ransaccircle(10, 0.99, 2000, false);
		std::vector<char> inlier_mask;
		double interior_rate = ransaccircle.run(circle_points, inlier_mask);

		double x, y, radius_s;
		ransaccircle.getCircle(x, y, radius_s);

		radius = sqrt(radius_s);
		center = cv::Point2d(x, y);
		circle_points.clear();
	}

	void FishStitcher::_getCircleEdgePoints(const cv::Mat &img, std::vector<std::vector<int> >&circle_points)
	{
		if (!circle_points.empty())circle_points.clear();
		double megapix = 0.5;
		int edge_black = 25;
		int big_black = 100;
		int shift_w_ratio = 50;
		int shift_h_ratio = 60;
		int window_windth_raio = 6;
		int grad_threshold_ratio = 20;
		int minX_threshold_ratio = 100;

		double work_scale = /*std::*/MYMIN(1.0, sqrt(megapix * 1e6 / img.size().area()));
		int step = 1.0 / work_scale;

		int W = img.cols;
		int H = img.rows;
		int half_H = H / 2;
		int half_W = W / 2;

		int shift_w = half_W / shift_w_ratio;
		int shift_h = half_H / shift_h_ratio;
		int window_width = half_W / window_windth_raio;
		int grad_threshold = grad_threshold_ratio * window_width;
		int minX_threshold = half_W / minX_threshold_ratio;

		int x_l, x_h, y_l, y_h;

		int channel_step = img.type() == CV_8UC3 ? 3 : 1;

		int c_half_W = channel_step * half_W;

		//设置大阈值找到边界框,和中心边界
		{
			//the coefficient that convert the bgr to grey value
			double coeffs[3] = { 0.114f, 0.587f, 0.299f };

			//得到最大最小X：minX ，maxX
			int minX = -1, maxX = W;
			const uchar * median_row_ptr = img.ptr(half_H);
			int temp_value;

			for (int i = 0; i < c_half_W; i += channel_step)
			{
				if (channel_step == 1)
				{
					temp_value = median_row_ptr[i];
				}
				else temp_value = coeffs[0] * median_row_ptr[i] + coeffs[1] * median_row_ptr[i + 1] + coeffs[2] * median_row_ptr[i + 2];
				if (temp_value <= edge_black)
					minX++;
				else break;
			}
			for (int i = channel_step * (img.cols - 1); i >= c_half_W; i -= channel_step)
			{
				if (channel_step == 1)
				{
					temp_value = median_row_ptr[i];
				}
				else temp_value = coeffs[0] * median_row_ptr[i] + coeffs[1] * median_row_ptr[i + 1] + coeffs[2] * median_row_ptr[i + 2];
				if (temp_value <= edge_black)
					maxX--;
				else break;
			}

			//得到最大最小Y：minY ，maxY
			int minY = -1, maxY = img.rows;
			for (size_t i = 0; i < half_H; i++)
			{
				const uchar * row_ptr = img.ptr(i);
				if (channel_step == 1)
				{
					temp_value = row_ptr[c_half_W];
				}
				else temp_value = coeffs[0] * row_ptr[c_half_W] + coeffs[1] * row_ptr[c_half_W + 1] + coeffs[2] * row_ptr[c_half_W + 2];
				if (temp_value <= edge_black)
					minY++;
			}
			for (size_t i = img.rows - 1; i > half_H; i--)
			{
				const uchar * row_ptr = img.ptr(i);
				if (channel_step == 1)
				{
					temp_value = row_ptr[c_half_W];
				}
				else temp_value = coeffs[0] * row_ptr[c_half_W] + coeffs[1] * row_ptr[c_half_W + 1] + coeffs[2] * row_ptr[c_half_W + 2];
				if (temp_value <= edge_black)
					maxY--;
			}

			x_l = minX == -1 ? 0 : minX;
			x_h = maxX == img.cols ? img.cols - 1 : maxX;
			y_l = minY == -1 ? 0 : minY;
			y_h = maxY == img.rows ? img.rows - 1 : maxY;

			y_l += shift_h;
			y_h -= shift_h;

			assert(x_l < half_W && x_h > half_W && y_l < half_H && y_h > half_H);
		}

		std::vector<int> row_integral(half_W + window_width + 1, 0);

		{
			double coeffs[3] = { 0.114f, 0.587f, 0.299f };
			assert(img.type() == CV_8UC3 || img.type() == CV_8UC1);


			//对下半图进行操作
			for (int j = half_H; j < y_h; j += step)
			{
				const uchar * down_row_ptr = img.ptr(j);

				//对左半图进行操作
				int max_length = 0;
				int minX1 = 0;
				row_integral[0] = 0;
				bool mini_clock = false;
				int temp_value;
				for (int i = x_l * channel_step; i < c_half_W; i += channel_step, max_length++)
				{
					if (channel_step == 1)
					{
						temp_value = down_row_ptr[i];
					}
					else temp_value = coeffs[0] * down_row_ptr[i] + coeffs[1] * down_row_ptr[i + 1] + coeffs[2] * down_row_ptr[i + 2];

					row_integral[max_length + 1] = row_integral[max_length] + temp_value;
					if (temp_value <= m_black && !mini_clock)minX1++;
					if (temp_value > m_black)mini_clock = true;
					if (temp_value >= big_black)
					{
						max_length++;
						break;
					}
				}
				if (max_length != 1 && minX1 != half_W - x_l && minX1 > minX_threshold)
				{
					int start = max_length + x_l;
					int end = start + window_width;
					start *= channel_step;
					end *= channel_step;
					int max_length_temp = max_length;
					for (int i = start; i < end; i += channel_step, max_length_temp++)
					{
						if (channel_step == 1)
						{
							temp_value = down_row_ptr[i];
						}
						else temp_value = coeffs[0] * down_row_ptr[i] + coeffs[1] * down_row_ptr[i + 1] + coeffs[2] * down_row_ptr[i + 2];

						row_integral[max_length_temp + 1] = row_integral[max_length_temp] + temp_value;
					}

					int max_diff = 0;
					int max_index = minX1;
					for (int i = minX1; i <= max_length; i++)
					{
						int left_value, right_value;
						if (i > window_width)
						{
							left_value = row_integral[i] - row_integral[i - window_width];
						}
						else left_value = row_integral[i];

						right_value = row_integral[i + window_width] - row_integral[i];

						double diff_temp = right_value - left_value;
						if (diff_temp > max_diff)
						{
							max_diff = diff_temp;
							max_index = i;
						}
						else if (max_diff > grad_threshold)break;
					}

					max_index += x_l;
					if (max_index >= shift_w)
					{
						max_index -= shift_w;
						std::vector<int> point(2);
						point[0] = max_index;
						point[1] = j;
						circle_points.push_back(point);
					}
				}

				//对右半图进行操作
				max_length = 0;
				row_integral[0] = 0;
				int minX2 = 0;
				mini_clock = false;
				for (int i = x_h * channel_step; i > c_half_W; i -= channel_step, max_length++)
				{
					if (channel_step == 1)
					{
						temp_value = down_row_ptr[i];
					}
					else temp_value = coeffs[0] * down_row_ptr[i] + coeffs[1] * down_row_ptr[i + 1] + coeffs[2] * down_row_ptr[i + 2];

					row_integral[max_length + 1] = row_integral[max_length] + temp_value;
					if (temp_value <= m_black && !mini_clock)minX2++;
					if (temp_value > m_black)mini_clock = true;
					if (temp_value >= big_black)
					{
						max_length++;
						break;
					}
				}
				if (max_length != 1 && minX2 != x_h - half_W && minX2 > minX_threshold)
				{
					int start = x_h - max_length;
					int end = start - window_width;
					start *= channel_step;
					end *= channel_step;
					int max_length_temp = max_length;
					for (int i = start; i > end; i -= channel_step, max_length_temp++)
					{
						if (channel_step == 1)
						{
							temp_value = down_row_ptr[i];
						}
						else temp_value = coeffs[0] * down_row_ptr[i] + coeffs[1] * down_row_ptr[i + 1] + coeffs[2] * down_row_ptr[i + 2];
						row_integral[max_length_temp + 1] = row_integral[max_length_temp] + temp_value;
					}

					int max_diff = 0;
					int max_index = minX2;
					for (int i = minX2; i <= max_length; i++)
					{
						int left_value, right_value;
						if (i > window_width)
						{
							left_value = row_integral[i] - row_integral[i - window_width];
						}
						else left_value = row_integral[i];

						right_value = row_integral[i + window_width] - row_integral[i];

						double diff_temp = right_value - left_value;
						if (diff_temp > max_diff)
						{
							max_diff = diff_temp;
							max_index = i;
						}
						else if (max_diff > grad_threshold)break;
					}

					max_index = x_h - max_index;
					if (W - max_index > shift_w)
					{
						max_index += shift_w;
						std::vector<int> point(2);
						point[0] = max_index;
						point[1] = j;
						circle_points.push_back(point);
					}
				}

				if (minX1 == half_W - x_l && minX2 == x_h - half_W)
					break;
			}

			//对上半图进行操作
			for (int j = half_H; j > y_l; j -= step)
			{
				const uchar * up_row_ptr = img.ptr(j);

				//对左半图进行操作
				int max_length = 0;
				int minX1 = 0;
				row_integral[0] = 0;
				int temp_value;
				bool mini_clock = false;
				for (int i = x_l * channel_step; i < c_half_W; i += channel_step, max_length++)
				{
					if (channel_step == 1)
					{
						temp_value = up_row_ptr[i];
					}
					else temp_value = coeffs[0] * up_row_ptr[i] + coeffs[1] * up_row_ptr[i + 1] + coeffs[2] * up_row_ptr[i + 2];
					row_integral[max_length + 1] = row_integral[max_length] + temp_value;
					if (temp_value <= m_black && !mini_clock)minX1++;
					if (temp_value > m_black)mini_clock = true;
					if (temp_value >= big_black)
					{
						max_length++;
						break;
					}
				}
				if (max_length != 1 && minX1 != half_W - x_l&& minX1 > minX_threshold)
				{
					int start = max_length + x_l;
					int end = start + window_width;
					start *= channel_step;
					end *= channel_step;
					int max_length_temp = max_length;
					for (int i = start; i < end; i += channel_step, max_length_temp++)
					{
						if (channel_step == 1)
						{
							temp_value = up_row_ptr[i];
						}
						else temp_value = coeffs[0] * up_row_ptr[i] + coeffs[1] * up_row_ptr[i + 1] + coeffs[2] * up_row_ptr[i + 2];
						row_integral[max_length_temp + 1] = row_integral[max_length_temp] + temp_value;
					}

					int max_diff = 0;
					int max_index = minX1;
					for (int i = minX1; i <= max_length; i++)
					{
						int left_value, right_value;
						if (i > window_width)
						{
							left_value = row_integral[i] - row_integral[i - window_width];
						}
						else left_value = row_integral[i];

						right_value = row_integral[i + window_width] - row_integral[i];

						double diff_temp = right_value - left_value;
						if (diff_temp > max_diff)
						{
							max_diff = diff_temp;
							max_index = i;
						}
						else if (max_diff > grad_threshold)break;
					}

					max_index += x_l;
					if (max_index >= shift_w)
					{
						max_index -= shift_w;
						std::vector<int> point(2);
						point[0] = max_index;
						point[1] = j;
						circle_points.push_back(point);
					}
				}

				//对右半图进行操作
				max_length = 0;
				row_integral[0] = 0;
				int minX2 = 0;
				mini_clock = false;
				for (int i = x_h * channel_step; i > c_half_W; i -= channel_step, max_length++)
				{
					if (channel_step == 1)
					{
						temp_value = up_row_ptr[i];
					}
					else temp_value = coeffs[0] * up_row_ptr[i] + coeffs[1] * up_row_ptr[i + 1] + coeffs[2] * up_row_ptr[i + 2];
					row_integral[max_length + 1] = row_integral[max_length] + temp_value;
					if (temp_value <= m_black && !mini_clock)minX2++;
					if (temp_value > m_black)mini_clock = true;
					if (temp_value >= big_black)
					{
						max_length++;
						break;
					}
				}
				if (max_length != 1 && minX2 != x_h - half_W && minX2 > minX_threshold)
				{
					int start = x_h - max_length;
					int end = start - window_width;
					start *= channel_step;
					end *= channel_step;
					int max_length_temp = max_length;
					for (int i = start; i > end; i -= channel_step, max_length_temp++)
					{
						if (channel_step == 1)
						{
							temp_value = up_row_ptr[i];
						}
						else temp_value = coeffs[0] * up_row_ptr[i] + coeffs[1] * up_row_ptr[i + 1] + coeffs[2] * up_row_ptr[i + 2];
						row_integral[max_length_temp + 1] = row_integral[max_length_temp] + temp_value;
					}

					int max_diff = 0;
					int max_index = minX2;
					for (int i = minX2; i <= max_length; i++)
					{
						int left_value, right_value;
						if (i > window_width)
						{
							left_value = row_integral[i] - row_integral[i - window_width];
						}
						else left_value = row_integral[i];

						right_value = row_integral[i + window_width] - row_integral[i];

						double diff_temp = right_value - left_value;
						if (diff_temp > max_diff)
						{
							max_diff = diff_temp;
							max_index = i;
						}
						else if (max_diff > grad_threshold)break;
					}

					max_index = x_h - max_index;
					if (W - max_index > shift_w)
					{
						max_index += shift_w;
						std::vector<int> point(2);
						point[0] = max_index;
						point[1] = j;
						circle_points.push_back(point);
					}
				}

				if (minX1 == half_W - x_l && minX2 == x_h - half_W)
					break;
			}
		}
	}

	void FishStitcher::_checkSize(cv::Mat &result)
	{
		if (result.size() == m_result_size)return;
		if (std::abs(result.cols - m_result_size.width) > 5)return;
		if (std::abs(result.rows - m_result_size.height) > 5)return;

		cv::resize(result, result, m_result_size);
	}
}
