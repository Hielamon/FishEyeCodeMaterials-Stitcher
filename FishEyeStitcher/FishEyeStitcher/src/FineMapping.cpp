#include <FineMapping.h>

#ifndef MYISNAN
#if defined(WIN32) || defined(_WIN32)
#define MYISNAN _isnan
#else
#define MYISNAN std::isnan
#endif
#endif


namespace CircleFish
{

	//Motion Search in the square region
	//(input-output)motion : input the initmotion ,output the result motion
	bool CompareTemplate(cv::Mat &src, cv::Mat &aim, cv::Vec2i &motion, int square_size)
	{
		//Todo more check
		cv::Vec2i bestMotion = motion;
		double bestDist1 = DBL_MAX;
		double bestDist2 = DBL_MAX;
		double worstDist = 0;
		int half_ss = square_size*0.5;
		double nnratio = 0.9;

		cv::Point tl_aim(0, 0);
		cv::Point br_aim(aim.cols - 1, aim.rows - 1);

		for (size_t i = 0; i < square_size; i++)
		{
			for (size_t j = 0; j < square_size; j++)
			{
				cv::Point tl(i - half_ss + motion[0], j - half_ss + motion[1]);
				cv::Point br(tl.x + src.cols - 1, tl.y + src.rows - 1);

				cv::Point tl_lap, br_lap;
				tl_lap.x = tl.x > tl_aim.x ? tl.x : tl_aim.x;
				tl_lap.y = tl.y > tl_aim.y ? tl.y : tl_aim.y;

				br_lap.x = br.x < br_aim.x ? br.x : br_aim.x;
				br_lap.y = br.y < br_aim.y ? br.y : br_aim.y;

				cv::Rect roi_lap_src(tl_lap - tl, br_lap - tl);
				cv::Rect roi_lap_aim(tl_lap/* - tl_aim*/, br_lap/* - tl_aim*/);

				double dist = cv::norm(src(roi_lap_src), aim(roi_lap_aim));

				dist /= roi_lap_aim.area();

				if (dist > worstDist)worstDist = dist;


				if (dist < bestDist1)
				{
					bestDist2 = bestDist1;
					bestDist1 = dist;
					bestMotion[0] = tl.x;
					bestMotion[1] = tl.y;
				}
				else if(dist < bestDist2)
				{
					bestDist2 = dist;
				}
			}
		}

		if (bestDist1 < 0.4 * worstDist/* && bestDist1 < nnratio * bestDist2*/)
		{
			motion = bestMotion;
			return true;
		}
		else
			return false;
	}

	void BinaryByGridient(cv::Mat &src, cv::Mat &result)
	{
		cv::Mat sobelX, sobelY, sobelX_abs, sobelY_abs, sobel;
		cv::Sobel(src, sobelX, CV_16S, 1, 0, 3, 0.25, 0);
		cv::Sobel(src, sobelY, CV_16S, 0, 1, 3, 0.25, 0);
		cv::convertScaleAbs(sobelX, sobelX_abs);
		cv::convertScaleAbs(sobelY, sobelY_abs);

		cv::Mat sobelX_abs_b, sobelY_abs_b;
		cv::threshold(sobelX_abs, sobelX_abs_b, 10, 255, CV_THRESH_BINARY);
		cv::threshold(sobelY_abs, sobelY_abs_b, 10, 255, CV_THRESH_BINARY);

		result = sobelY_abs_b | sobelX_abs_b;
	}

	inline bool CheckValid(const cv::Mat &src)
	{
		assert(src.type() == CV_8UC1);
		for (size_t i = 0; i < src.rows; i++)
		{
		    const uchar *row_ptr = src.ptr(i);
			for (size_t j = 0; j < src.cols; j++)
				if (row_ptr[j])return true;
		}
		return false;
	}

	inline bool CheckMask(cv::Mat &src)
	{
		assert(src.type() == CV_8UC1);
		for (size_t i = 0; i < src.rows; i++)
		{
			uchar *row_ptr = src.ptr(i);
			for (size_t j = 0; j < src.cols; j++)
				if (!row_ptr[j])return true;
		}
		return false;
	}

	FineMapping::FineMapping(ORBextractor *pORBextractor)
	{
		mpORBextractor = pORBextractor;
	}

	FineMapping::~FineMapping() {}

	bool FineMapping::operator ()(cv::Mat &img1, cv::Mat &img2, MapAndX &mAndx1, MapAndX &mAndx2, cv::Mat &mask1, cv::Mat &mask2)
	{
		assert(img1.size() == img2.size());

		ORBMatcher::KeyAndDescriptor kap1, kap2;
		mimg1 = img1;
		mimg2 = img2;

		mmask1 = mask1;
		mmask2 = mask2;

		if (img1.type() == CV_8UC3)cv::cvtColor(img1, mimg1_grey, cv::COLOR_BGR2GRAY);
		else if (img1.type() == CV_8UC1)img1.copyTo(mimg1_grey);
		else return false;

		if (img2.type() == CV_8UC3)cv::cvtColor(img2, mimg2_grey, cv::COLOR_BGR2GRAY);
		else if (img2.type() == CV_8UC1)img2.copyTo(mimg2_grey);
		else return false;

		(*mpORBextractor)(mimg1_grey, cv::Mat(), kap1.first, kap1.second);
		(*mpORBextractor)(mimg2_grey, cv::Mat(), kap2.first, kap2.second);

		mnRowNum = 2 * img1.rows / img1.cols;
		if (!maGridIndex1.empty())maGridIndex1.clear();
		if (!maGridIndex2.empty())maGridIndex2.clear();
		if (!mvStat.empty())mvStat.clear();
		maGridIndex1.resize(mnRowNum);
		maGridIndex2.resize(mnRowNum);
		mvStat.resize(mnRowNum, NONE);

		mnGridElementWidth = img1.cols;
		mnGridElementHeight = img1.rows / mnRowNum;
		mfGridElementWidthInv = 1.0 / mnGridElementWidth;
		mfGridElementHeightInv = 1.0 / mnGridElementHeight;
		mnWindowSize = mnGridElementWidth*0.3;

		_assignToGrid(kap1.first, maGridIndex1);
		_assignToGrid(kap2.first, maGridIndex2);

		ORBMatcher::IndexContainer aidxcontainer12(kap1.first.size());
		_fillIndexContainer(aidxcontainer12, kap1.first, kap2.first);

		ORBMatcher matcher(0.9, true);
		std::vector<int> vnMatches12, vnMatches21;
		matcher.SearchIndex(kap1, kap2, aidxcontainer12, vnMatches12, vnMatches21);

		//Used for storage the block motion
		GridShift aGridShift1(mnRowNum);

		//Used for record the block which get its motion from template matching
		GridFlag aGridFlag(mnRowNum,false);

		_fillGridShift(aGridShift1, aGridFlag, kap1.first, kap2.first, vnMatches12);

		mAndx1.first.create(img1.size(), CV_32FC2);
		mAndx2.first.create(img2.size(), CV_32FC2);

		if (!_fillWarpMap(aGridShift1, mAndx1, mAndx2))
		{
			mAndx1.first.release();
			mAndx2.first.release();
		}

		return true;
	}

	void FineMapping::_assignToGrid(std::vector<cv::KeyPoint> &vkpts, GridIndex &aGridIndex)
	{
		assert(aGridIndex.size() == mnRowNum);
		size_t N = vkpts.size();
		for (size_t i = 0; i < N; i++)
		{
			const cv::KeyPoint &kpt = vkpts[i];

			int nGridPosY = floor(kpt.pt.y*mfGridElementHeightInv);

			if (nGridPosY < 0 || nGridPosY >= mnRowNum)
				continue;

			aGridIndex[nGridPosY].push_back(i);
		}
	}

	void FineMapping:: _fillIndexContainer(ORBMatcher::IndexContainer &aidxcontainer12, std::vector<cv::KeyPoint> &vkpts1,
			std::vector<cv::KeyPoint> &vkpts2)
	{
		size_t N = aidxcontainer12.size();
		assert(N == vkpts1.size());
		int nMinCellY, nMaxCellY;
		for (size_t i = 0; i < N; i++)
		{
			float x = vkpts1[i].pt.x;
			float y = vkpts1[i].pt.y;
			std::vector<size_t> &vIndices = aidxcontainer12[i];

			const int nMinCellY = MYMAX(0, (int)floor((y - mnWindowSize)*mfGridElementHeightInv));
			if (nMinCellY >= mnRowNum)continue;

			const int nMaxCellY = MYMIN((int)mnRowNum - 1, (int)ceil((y + mnWindowSize)*mfGridElementHeightInv));
			if (nMaxCellY < 0)continue;

			for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
			{
				std::vector<size_t> &vCell = maGridIndex2[iy];
				if (vCell.empty())
					continue;

				for (size_t j = 0, jend = vCell.size(); j<jend; j++)
				{
					const cv::KeyPoint &kp2 = vkpts2[vCell[j]];

					const float distx = kp2.pt.x - x;
					const float disty = kp2.pt.y - y;

					if (std::abs(distx)<mnWindowSize && std::abs(disty)<mnWindowSize)
						vIndices.push_back(vCell[j]);
				}
			}
		}
	}

	//Get the average shift in every aGridShift£¬in the sign of Test£¬meanwhile refining the vnMatches12
	void FineMapping::_fillGridShift(GridShift &aGridShift, GridFlag &aGridFlag, std::vector<cv::KeyPoint> &vkpts1,
			std::vector<cv::KeyPoint> &vkpts2, std::vector<int> &vnMatches12)
	{
		assert(aGridShift.size() == mnRowNum);

		bool refine_matches = !false;

		RansacShift ransacshift(0.4, 0.99, 100);

		for (size_t iy = 0; iy < mnRowNum; iy++)
		{
			std::vector<size_t> &vCell = maGridIndex1[iy];
			aGridFlag[iy] = false;

			size_t jN = vCell.size();

			std::vector<std::vector<float> > vshift;
			vshift.reserve(jN);

			std::vector<size_t> vCellMatched;
			vCellMatched.reserve(jN);

			for (size_t j = 0; j<jN; j++)
			{
				size_t nidx1 = vCell[j], nidx2 = vnMatches12[nidx1];
				if (nidx2 == -1)continue;
				std::vector<float> shift(2);
				shift[0] = vkpts2[nidx2].pt.x - vkpts1[nidx1].pt.x;
				shift[1] = vkpts2[nidx2].pt.y - vkpts1[nidx1].pt.y;
				vshift.push_back(shift);
				vCellMatched.push_back(nidx1);
			}

			cv::Vec2f &mainshift = aGridShift[iy];
			mainshift[0] = mainshift[1] = std::numeric_limits<float>::quiet_NaN();

			if (vshift.size() < 5)
			{
				if (refine_matches)
					for (size_t j = 0, jend = vCellMatched.size(); j < jend; j++)
						vnMatches12[vCellMatched[j]] = -1;

				_templateMatchGrid(iy, aGridShift);

				if (!MYISNAN(mainshift[0]))aGridFlag[iy] = true;
				continue;
			}

			std::vector<char> mask;
			ransacshift.run(vshift, mask);
			double inlier_rate = ransacshift.refine(vshift, mask);

			if (inlier_rate > 0.5)
			{
				ransacshift.getShift(mainshift[0], mainshift[1]);
				assert(vCellMatched.size() == mask.size());
				if (refine_matches)
					for (size_t j = 0, jend = vCellMatched.size(); j < jend; j++)
						if (mask[j] == 0)vnMatches12[vCellMatched[j]] = -1;
			}
			else
			{
				if (refine_matches)
					for (size_t j = 0, jend = vCellMatched.size(); j < jend; j++)
						vnMatches12[vCellMatched[j]] = -1;

				_templateMatchGrid(iy, aGridShift);

				if (!MYISNAN(mainshift[0]))aGridFlag[iy] = true;
			}
		}

		bool is_smooth_shift = true;
		if (!is_smooth_shift)return;

		//Worked for the Template motion result 
		for (size_t iy = 0; iy < mnRowNum; iy++)
		{
			if (aGridFlag[iy])
			{
				cv::Vec2f &cur_shift = aGridShift[iy];
				if (MYISNAN(cur_shift[0]))continue;
				std::vector<cv::Vec2f *> vnshift_orb;
				size_t n = 1;
				for (size_t i = 0; i < n; i++)
				{
					int nRowidx = iy - i - 1;
					if (nRowidx >= 0 && !aGridFlag[nRowidx] && !MYISNAN(aGridShift[nRowidx][0]))
					{
						vnshift_orb.push_back(&aGridShift[nRowidx]);
					}
				}

				for (size_t i = 0; i < n; i++)
				{
					int nRowidx = iy + i + 1;
					if (nRowidx < mnRowNum && !aGridFlag[nRowidx] && !MYISNAN(aGridShift[nRowidx][0]))
					{
						vnshift_orb.push_back(&aGridShift[nRowidx]);
					}
				}

				if (vnshift_orb.size() > 0)
				{
					cv::Vec2f avg_shift(0, 0);
					for (size_t i = 0; i < vnshift_orb.size(); i++)
						avg_shift += *vnshift_orb[i];

					avg_shift[0] /= vnshift_orb.size();
					avg_shift[1] /= vnshift_orb.size();

					if (avg_shift.dot(cur_shift) < 0)
					{
						aGridFlag[iy] = false;
						cur_shift[0] = std::numeric_limits<float>::quiet_NaN();
						cur_shift[1] = std::numeric_limits<float>::quiet_NaN();
						continue;
					}

					double avg_shift_l = cv::norm(avg_shift);
					double cur_shift_l = cv::norm(avg_shift);

					if (std::abs(cur_shift_l - avg_shift_l) > avg_shift_l *0.8)
					{
						aGridFlag[iy] = false;
						cur_shift[0] = std::numeric_limits<float>::quiet_NaN();
						cur_shift[1] = std::numeric_limits<float>::quiet_NaN();
						continue;
					}

					aGridFlag[iy] = false;
				}
			}
		}
	}

	bool FineMapping::_fillWarpMap(GridShift &aGridShift1, MapAndX &mAndx1, MapAndX &mAndx2)
	{

		cv::Mat &map1 = mAndx1.first, &map2 = mAndx2.first;
		assert(!map1.empty() && !map2.empty());
		cv::Vec2f &betaX1 = mAndx1.second, &betaX2 = mAndx2.second;
		betaX1[0] = betaX2[0] = FLT_MAX;
		betaX1[1] = betaX2[1] = FLT_MIN;

		std::vector<cv::Vec2f> shift_avg;
		std::vector<int> stunm_y;
		for (size_t iy = 0; iy < mnRowNum; iy++)
		{
			cv::Vec2f &shift_temp = aGridShift1[iy];
			if (MYISNAN(shift_temp[0]) || MYISNAN(shift_temp[1]))continue;
			int center_y = (iy + 0.5)*mnGridElementHeight;
			stunm_y.push_back(center_y);
			shift_avg.push_back(shift_temp);
			if (shift_temp[0] < betaX1[0])betaX1[0] = shift_temp[0];
			if (shift_temp[0] > betaX1[1])betaX1[1] = shift_temp[0];
		}
		betaX1[0] = betaX1[0] * 0.5;
		betaX1[1] = betaX1[1] * 0.5;

		betaX2[0] = -betaX1[1];
		betaX2[1] = -betaX1[0];


		if (shift_avg.size() == 0)return false;

		if (stunm_y[0] > mnRowNum*0.5*mnGridElementHeight)
		{
			assert(mnRowNum > 6);
			int zero_stunm = stunm_y[0] - 3 * mnGridElementHeight;
			stunm_y.insert(stunm_y.begin(), zero_stunm);
			shift_avg.insert(shift_avg.begin(), cv::Vec2f(0, 0));
		}

		if (stunm_y[stunm_y.size() - 1] < mnRowNum*0.5*mnGridElementHeight)
		{
			assert(mnRowNum > 6);
			int zero_stunm = stunm_y[0] + 3 * mnGridElementHeight;
			stunm_y.push_back(zero_stunm);
			shift_avg.push_back(cv::Vec2f(0, 0));
		}
		std::vector<cv::Vec2f> shift_spinal(map1.rows);
		for (size_t i = 0, pre_k = -1, k = 0, i_end = map1.rows; i < i_end; i++)
		{
			if (k == shift_avg.size())shift_spinal[i] = shift_avg[pre_k];
			else if (i == stunm_y[k])
			{
				shift_spinal[i] = shift_avg[k];
				pre_k = k;
				k++;
			}
			else
			{
				if (pre_k == -1)shift_spinal[i] = shift_avg[k];
				else
				{
					double pre_r = (stunm_y[k] - i) / (double)(stunm_y[k] - stunm_y[pre_k]);
					double _r = (i - stunm_y[pre_k]) / (double)(stunm_y[k] - stunm_y[pre_k]);
					shift_spinal[i] = pre_r*shift_avg[pre_k] + _r*shift_avg[k];
				}
			}
		}

#ifdef PRINT_OUT
		cv::Mat temp;
		mimg1.copyTo(temp);

		for (size_t j = 0; j < mnRowNum; j++)
		{
			cv::line(temp, cv::Point(0, j*mnGridElementHeight), cv::Point(temp.cols, j*mnGridElementHeight), cv::Scalar(0, 0, 255));
		}

		std::stringstream ss;
		ss << "spinal_" << call_count << ".jpg";
		for (size_t i = 0; i < shift_spinal.size(); i += 10)
		{
			cv::Point center_x(mimg1.cols*0.5, i);
			cv::Point aim_pt(shift_spinal[i][0], shift_spinal[i][1]);
			aim_pt += center_x;
			cv::circle(temp, center_x, 1, cv::Scalar(0, 0, 255), -1);
			cv::line(temp, center_x, aim_pt, cv::Scalar(0, 0, 255));
		}

		cv::imwrite(ss.str(), temp);
#endif

		for (size_t i = 0, iend = map1.rows; i < iend; i++)
		{
			cv::Vec2f * pmapRow1 = (cv::Vec2f *)map1.ptr(i);
			cv::Vec2f * pmapRow2 = (cv::Vec2f *)map2.ptr(i);
			cv::Vec2f row_shift = -shift_spinal[i] * 0.5;
			for (size_t j = 0, jend = map1.cols; j < jend; j++)
			{
				pmapRow1[j] = row_shift;
				pmapRow2[j] = -row_shift;
			}
		}

		return true;
	}

	void FineMapping::_templateMatchGrid(int r, GridShift &aGridShift)
	{
		assert(r >= 0 && r < mnRowNum);
		cv::Rect grid_roi(0, r*mnGridElementHeight, mnGridElementWidth, mnGridElementHeight);
		cv::Vec2i init_motion = cv::Vec2f(0, 0);
		cv::Vec2f &motion = aGridShift[r];
		motion = cv::Vec2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
		//return;

		//Exclude the black region in mask
		cv::Mat grid_mask1 = mmask1(grid_roi);
		if (CheckMask(grid_mask1))return;

		cv::Mat grid_img1 = mimg1_grey(grid_roi);
		cv::Mat grid_img2 = mimg2_grey(grid_roi);

		cv::Mat binaryCheck1, binaryCheck2;

		TempStat laststat = r > 0 ? mvStat[r - 1] : NONE;

		BinaryByGridient(grid_img1, binaryCheck1);
		if (!CheckValid(binaryCheck1))
		{
			mvStat[r] = FineMapping::ALL_EMPTY;
			return;
		}

		bool up_empty = !CheckValid(binaryCheck1(cv::Rect(0, 0, binaryCheck1.cols, binaryCheck1.rows*0.5)));
		bool down_empty = !CheckValid(binaryCheck1(cv::Rect(0, binaryCheck1.rows*0.5, binaryCheck1.cols, binaryCheck1.rows*0.5)));


		if (down_empty)
		{
			mvStat[r] = FineMapping::DOWN_EMPTY;
			switch (laststat)
			{
			case FineMapping::NONE:
				if (r > 0)
				{
					grid_roi = cv::Rect(0, ((double)r - 0.5)*mnGridElementHeight, mnGridElementWidth, mnGridElementHeight);
					grid_img1 = mimg1_grey(grid_roi);
					grid_img2 = mimg2_grey(grid_roi);
					binaryCheck1.release();
				}
				break;
			case FineMapping::UP_EMPTY:
				motion = aGridShift[r - 1];
				return;
			case FineMapping::DOWN_EMPTY:
				grid_roi = cv::Rect(0, r*mnGridElementHeight, mnGridElementWidth, mnGridElementHeight*0.5);
				grid_img1 = mimg1_grey(grid_roi);
				grid_img2 = mimg2_grey(grid_roi);
				binaryCheck1 = binaryCheck1(cv::Rect(0, 0, binaryCheck1.cols, binaryCheck1.rows*0.5));
				break;
			case FineMapping::ALL_EMPTY:
				//do like FineMapping::DOWN_EMPTY
				grid_roi = cv::Rect(0, r*mnGridElementHeight, mnGridElementWidth, mnGridElementHeight*0.5);
				grid_img1 = mimg1_grey(grid_roi);
				grid_img2 = mimg2_grey(grid_roi);
				binaryCheck1 = binaryCheck1(cv::Rect(0, 0, binaryCheck1.cols, binaryCheck1.rows*0.5));
				break;
			default:
				break;
			}

		}
		else if (up_empty)
		{
			mvStat[r] = FineMapping::UP_EMPTY;
			if (r < mnRowNum - 1)
			{
				grid_roi = cv::Rect(0, (r + 0.5)*mnGridElementHeight, mnGridElementWidth, mnGridElementHeight);
				grid_img1 = mimg1_grey(grid_roi);
				grid_img2 = mimg2_grey(grid_roi);
				binaryCheck1.release();
			}
		}

		if (maGridIndex2[r].size() == 0)
		{
			BinaryByGridient(grid_img2, binaryCheck2);
			if (!CheckValid(binaryCheck2))return;
		}

		bool flag = CompareTemplate(grid_img1, grid_img2, init_motion, mnGridElementWidth*0.3);
		if (flag)
		{
			/*const int motion_th = mnWindowSize*0.5;
			if (std::abs(init_motion[0]) >= motion_th ||
			std::abs(init_motion[1]) >= motion_th)
			{
			return;
			}*/

			if (binaryCheck1.empty())BinaryByGridient(grid_img1, binaryCheck1);
			if (binaryCheck2.empty())BinaryByGridient(grid_img2, binaryCheck2);

			cv::Point tl(init_motion[0], init_motion[1]), tl_aim(0, 0);
			cv::Point br(tl.x + grid_img1.cols - 1, tl.y + grid_img1.rows - 1), br_aim(grid_img2.cols - 1, grid_img2.rows - 1);

			cv::Point tl_lap, br_lap;
			tl_lap.x = tl.x > tl_aim.x ? tl.x : tl_aim.x;
			tl_lap.y = tl.y > tl_aim.y ? tl.y : tl_aim.y;

			br_lap.x = br.x < br_aim.x ? br.x : br_aim.x;
			br_lap.y = br.y < br_aim.y ? br.y : br_aim.y;

			cv::Rect roi1(tl_lap - tl, br_lap - tl);
			cv::Rect roi2(tl_lap, br_lap);

			cv::Mat roi1_binary = binaryCheck1(roi1);
			if (!CheckValid(roi1_binary))return;

			cv::Mat roi2_binary = binaryCheck2(roi2);
			if (!CheckValid(roi2_binary))return;

			//Finally the motion maybe valid
			motion = cv::Vec2f(init_motion);
		}

	}
}
