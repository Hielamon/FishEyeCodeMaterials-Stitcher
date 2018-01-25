#pragma once
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <list>

struct PairInfo
{
	std::vector<cv::Point2d> points1, points2;
	std::vector<char> mask;
	int index1, index2;
	int pairs_num;
	int inliers_num;
};

class SequenceMatcher
{
public:
	//特征类型
	enum FeatureType
	{
		F_ORB, F_SIFT, F_SURF
	};

	SequenceMatcher(const FeatureType &featuretype = F_ORB, float scale = 1.0);

	~SequenceMatcher();
	
	//使用特征匹配的方法找到一个合适的图片匹配排列，并返回这个匹配排列的信息
	void process(const std::vector<cv::Mat>&images, std::list<PairInfo> &pairinfos);

	//将pairinfos匹配序列的第一张图和最后一张图的匹配信息，push_back 到pairinfos中
	void getRingPair(std::list<PairInfo> &pairinfos);

protected:
	FeatureType m_featuretype;
	cv::Ptr<cv::Feature2D>  m_featurator;
	cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

	std::vector<std::vector<PairInfo> > m_all_pairinfos;

	std::vector<std::vector<cv::KeyPoint> > m_keypoints_arr;
	std::vector<cv::Mat> m_descriptors_arr;

	float m_scale;

	//提取images中每张图片的特征点以及特征描述
	void _getFeatures(const std::vector<cv::Mat>&images, std::vector<std::vector<cv::KeyPoint> > &keypoints_arr, std::vector<cv::Mat> &descriptors_arr);

	//找到des1 在 descriptors2中的最近邻对应点 des2，且满足des2 在 descriptors1中的最近邻对应点为des1
	bool _getCorresponds(const cv::Mat &descriptors1, const cv::Mat &descriptors2, std::vector<cv::DMatch>& matches, double ratio = 0.5);

	//得到内点比率
	float _getInlierRate(const std::vector<char>& inliers);

};

void DrawPairInfos(std::vector<cv::Mat> &images, std::list<PairInfo> &pairinfos, bool onlyPoints = false, double scale = 1.0);

