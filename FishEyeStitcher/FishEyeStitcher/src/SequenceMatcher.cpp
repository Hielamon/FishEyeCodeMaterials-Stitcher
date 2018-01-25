#include <SequenceMatcher.h>
#include <opencv2\imgproc.hpp>
#include <opencv2\highgui.hpp>

SequenceMatcher::SequenceMatcher(const FeatureType &featuretype, float scale)
{
	m_featuretype = featuretype;
	m_scale = scale;
	switch (m_featuretype)
	{
	case F_ORB:	m_featurator = cv::ORB::create(2000);
		m_descriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
		break;
	case F_SIFT:m_featurator = cv::xfeatures2d::SIFT::create();
		m_descriptorMatcher = new cv::FlannBasedMatcher;
		break;
	case F_SURF:m_featurator = cv::xfeatures2d::SURF::create();
		m_descriptorMatcher = new cv::FlannBasedMatcher;
		break;
	default:
		break;
	}
}

SequenceMatcher::~SequenceMatcher() {}

//使用特征匹配的方法找到一个合适的图片匹配排列，并返回这个匹配排列的信息
void SequenceMatcher::process(const std::vector<cv::Mat>&images, std::list<PairInfo> &pairinfos)
{
	int num_images = images.size();
	if (num_images < 2)return;

	m_keypoints_arr.resize(num_images);
	m_descriptors_arr.resize(num_images);

	_getFeatures(images, m_keypoints_arr, m_descriptors_arr);
	int height = images[0].rows;
	int q_height = 1.0 * height * m_scale;

	if (!pairinfos.empty())pairinfos.clear();
	if (!m_all_pairinfos.empty())m_all_pairinfos.clear();
	m_all_pairinfos.resize(num_images - 1);
	std::vector<bool> is_finds(num_images, false);
	int left_index = -1, right_index = -1, max_num = 0;
	for (size_t i = 0; i < num_images - 1; i++)
	{
		for (size_t j = i + 1; j < num_images; j++)
		{
			std::vector<cv::DMatch> matches;
			_getCorresponds(m_descriptors_arr[i], m_descriptors_arr[j], matches);
			PairInfo pair_temp;
			pair_temp.pairs_num = matches.size();
			pair_temp.inliers_num = pair_temp.pairs_num;
			pair_temp.mask.resize(matches.size(), 1);
			pair_temp.index1 = i;
			pair_temp.index2 = j;
			std::vector<cv::KeyPoint> &keypoints1 = m_keypoints_arr[i];
			std::vector<cv::KeyPoint> &keypoints2 = m_keypoints_arr[j];
			for (size_t i = 0; i < matches.size(); i++)
			{
				cv::Point2d point1 = keypoints1[matches[i].queryIdx].pt;
				cv::Point2d point2 = keypoints2[matches[i].trainIdx].pt;
				if (point1.y > q_height || point2.y > q_height)continue;
				(pair_temp.points1).push_back(point1);
				(pair_temp.points2).push_back(point2);
			}

			m_all_pairinfos[i].push_back(pair_temp);

			if (pair_temp.pairs_num > max_num)
			{
				left_index = i;
				right_index = j;
				max_num = pair_temp.pairs_num;
			}
		}
	}

	pairinfos.push_back(m_all_pairinfos[left_index][right_index - left_index - 1]);
	is_finds[left_index] = is_finds[right_index] = true;

	while (1)
	{
		int left_max_num = 0, left_index_temp = -1;
		for (size_t i = 0; i < num_images; i++)
		{
			if (is_finds[i])continue;
			assert(i != left_index);
			int num_i_left;
			if (i > left_index)
				num_i_left = m_all_pairinfos[left_index][i - 1 - left_index].pairs_num;
			else
				num_i_left = m_all_pairinfos[i][left_index - 1 - i].pairs_num;
			if (num_i_left > left_max_num)
			{
				left_max_num = num_i_left;
				left_index_temp = i;
			}
		}

		int right_max_num = 0, right_index_temp = -1;
		for (size_t i = 0; i < num_images; i++)
		{
			if (is_finds[i])continue;
			assert(i != right_index);
			int num_i_right;
			if (i > right_index)
				num_i_right = m_all_pairinfos[right_index][i - 1 - right_index].pairs_num;
			else
				num_i_right = m_all_pairinfos[i][right_index - 1 - i].pairs_num;
			if (num_i_right > right_max_num)
			{
				right_max_num = num_i_right;
				right_index_temp = i;
			}
		}

		if (left_index_temp == -1 && right_index_temp == -1)break;
		if (left_max_num > right_max_num)
		{
			if (left_index_temp > left_index)
			{
				PairInfo pair_temp = m_all_pairinfos[left_index][left_index_temp - left_index - 1];
				int temp = pair_temp.index1;
				pair_temp.index1 = pair_temp.index2;
				pair_temp.index2 = temp;
				std::vector<cv::Point2d> temp_pps = pair_temp.points1;
				pair_temp.points1 = pair_temp.points2;
				pair_temp.points2 = temp_pps;
				pairinfos.push_front(pair_temp);
			}
			else pairinfos.push_front(m_all_pairinfos[left_index_temp][left_index - 1 - left_index_temp]);

			is_finds[left_index_temp] = true;
			left_index = left_index_temp;
		}
		else
		{
			if (right_index_temp < right_index)
			{
				PairInfo pair_temp = m_all_pairinfos[right_index_temp][right_index - 1 - right_index_temp];
				int temp = pair_temp.index1;
				pair_temp.index1 = pair_temp.index2;
				pair_temp.index2 = temp;
				std::vector<cv::Point2d> temp_pps = pair_temp.points1;
				pair_temp.points1 = pair_temp.points2;
				pair_temp.points2 = temp_pps;
				pairinfos.push_back(pair_temp);
			}
			else pairinfos.push_back(m_all_pairinfos[right_index][right_index_temp - 1 - right_index]);

			is_finds[right_index_temp] = true;
			right_index = right_index_temp;
		}
	}

}

//将pairinfos匹配序列的第一张图和最后一张图的匹配信息，push_back 到pairinfos中
void SequenceMatcher::getRingPair(std::list<PairInfo> &pairinfos)
{
	assert(pairinfos.size() >= 2 || !m_all_pairinfos.empty());
	int first_index = pairinfos.begin()->index1;
	int second_index = first_index;
	//std::list<PairInfo>::iterator result_iter;

	for (std::list<PairInfo>::iterator iter = pairinfos.begin(); iter != pairinfos.end(); iter++)
	{
		second_index = iter->index2;
		//result_iter = iter;
	}


	assert(first_index != second_index);
	assert(m_all_pairinfos.size() >= first_index && m_all_pairinfos.size() >= second_index);

	if (first_index < second_index)
		pairinfos.push_back(m_all_pairinfos[first_index][second_index - first_index - 1]);
	else
	{
		PairInfo pair_temp = m_all_pairinfos[second_index][first_index - 1 - second_index];
		int temp = pair_temp.index1;
		pair_temp.index1 = pair_temp.index2;
		pair_temp.index2 = temp;
		std::vector<cv::Point2d> temp_pps = pair_temp.points1;
		pair_temp.points1 = pair_temp.points2;
		pair_temp.points2 = temp_pps;
		pairinfos.push_back(pair_temp);
	}

	//return result_iter;
}

//提取images中每张图片的特征点以及特征描述
void SequenceMatcher::_getFeatures(const std::vector<cv::Mat>&images, std::vector<std::vector<cv::KeyPoint>> &keypoints_arr, std::vector<cv::Mat> &descriptors_arr)
{
	for (size_t i = 0; i < images.size(); i++)
	{
		m_featurator->detectAndCompute(images[i], cv::Mat(), keypoints_arr[i], descriptors_arr[i]);
		for (size_t j = 0; j < keypoints_arr[i].size(); j++)
		{
			keypoints_arr[i][j].pt *= m_scale;
		}
	}
}

//找到des1 在 descriptors2中的最近邻对应点 des2，且满足des2 在 descriptors1中的最近邻对应点为des1
bool SequenceMatcher::_getCorresponds(const cv::Mat &descriptors1, const cv::Mat &descriptors2, std::vector<cv::DMatch>& matches, double ratio)
{

	if (!matches.empty())matches.clear();
	if (m_featuretype == FeatureType::F_ORB)
	{
		std::vector<cv::DMatch> matches_1t2, matches_2t1;

		m_descriptorMatcher->match(descriptors1, descriptors2, matches_1t2);
		m_descriptorMatcher->match(descriptors2, descriptors1, matches_2t1);
		for (size_t i = 0; i < matches_1t2.size(); i++)
		{
			if (matches_2t1[matches_1t2[i].trainIdx].trainIdx == i)
			{
				assert(matches_1t2[i].queryIdx == i);
				matches.push_back(matches_1t2[i]);
			}
		}
	}
	else
	{
		assert(m_featuretype == FeatureType::F_SIFT || m_featuretype == FeatureType::F_SURF);
		std::vector<std::vector<cv::DMatch>> two_matches_1t2, two_matches_2t1;
		m_descriptorMatcher->knnMatch(descriptors1, descriptors2, two_matches_1t2, 2);
		m_descriptorMatcher->knnMatch(descriptors2, descriptors1, two_matches_2t1, 2);

		//float ratio = 0.50;

		for (size_t i = 0; i < two_matches_1t2.size(); i++)
		{
			std::vector<cv::DMatch> &match_tmp = two_matches_1t2[i];
			if (match_tmp.size() < 2 || match_tmp[0].distance / match_tmp[1].distance > ratio)match_tmp.clear();
		}

		for (size_t i = 0; i < two_matches_2t1.size(); i++)
		{
			std::vector<cv::DMatch> &match_tmp = two_matches_2t1[i];
			if (match_tmp.size() < 2 || match_tmp[0].distance / match_tmp[1].distance > ratio)match_tmp.clear();
		}

		for (size_t i = 0; i < two_matches_1t2.size(); i++)
		{
			if (two_matches_1t2[i].size() == 2 && two_matches_2t1[two_matches_1t2[i][0].trainIdx].size() == 2
				&& two_matches_2t1[two_matches_1t2[i][0].trainIdx][0].trainIdx == i)
			{
				assert(two_matches_1t2[i][0].queryIdx == i);
				matches.push_back(two_matches_1t2[i][0]);
			}
		}
	}

	if (matches.size() < 4)return false;
	return true;
}

//得到内点比率
float SequenceMatcher::_getInlierRate(const std::vector<char>& inliers)
{
	int n = 0;
	for (size_t i = 0; i < inliers.size(); i++)
	{
		if (inliers[i] != 0)n++;
	}
	return n / (float)inliers.size();
}

//Draw the Pair Infos for inputs
void DrawPairInfos(std::vector<cv::Mat> &images, std::list<PairInfo> &pairinfos, bool onlyPoints, double scale)
{
	size_t pair_num = pairinfos.size();
	std::string name = "pairs_";
	std::stringstream ss;
	int index = 0;

	int ptRadius = 12, ptThickness = 2, lineThickness = 5;

	for (std::list<PairInfo>::iterator iter = pairinfos.begin(); iter != pairinfos.end(); iter++)
	{

		int index0 = iter->index1, index1 = iter->index2;
		assert(images[index0].type() == images[index1].type());
		int resultType = images[iter->index1].type();
		cv::Size imgSize0(images[index0].size()), imgSize1(images[index1].size());
		cv::Size resultSize(imgSize0.width + imgSize1.width, std::max(imgSize0.height, imgSize1.height));

		cv::Mat result(resultSize, resultType, cv::Scalar(0));;
		images[index0].copyTo(result(cv::Rect(0, 0, imgSize0.width, imgSize0.height)));
		images[index1].copyTo(result(cv::Rect(imgSize0.width, 0, imgSize1.width, imgSize1.height)));

		for (size_t i = 0; i < iter->pairs_num; i++)
		{
			if (iter->mask[i] != 1)continue;
			uchar r = rand() % 255;
			uchar g = rand() % 255;
			uchar b = rand() % 255;
			cv::Scalar color(b, g, r);
			cv::circle(result, iter->points1[i] * scale, ptRadius, color, ptThickness);
			cv::Point2d pt2 = iter->points2[i] * scale;
			pt2.x += imgSize0.width;
			if (!onlyPoints)cv::line(result, iter->points1[i] * scale, pt2, color, lineThickness);
			cv::circle(result, pt2, ptRadius, color, ptThickness);
			//cv::imshow("showtemp", result);
			//cv::waitKey(0);
		}
		ss << index;
		cv::imwrite(name + ss.str() + ".jpg", result);
		ss.str("");
		index++;
	}
}
