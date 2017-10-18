#pragma once
#include <opencv2/core/core.hpp>

class ORBMatcher
{
public:

	typedef std::vector<std::vector<size_t> > IndexContainer;
	typedef std::pair<std::vector<cv::KeyPoint>, cv::Mat> KeyAndDescriptor;

	ORBMatcher(float nnratio = 0.6, bool checkOri = true);
	
	~ORBMatcher();

	// Computes the Hamming distance between two ORB descriptors
	static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

	void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

	int SearchIndex(KeyAndDescriptor &kap1, KeyAndDescriptor &kap2, IndexContainer &aIndexContainer,
		std::vector<int> &vnMatches12, std::vector<int> &vnMatches21);

	int SearchAll(KeyAndDescriptor &kap1, KeyAndDescriptor &kap2,
		std::vector<int> &vnMatches12, std::vector<int> &vnMatches21);

	static const int TH_LOW = 50;
	//static const int TH_HIGH = 100;
	static const int HISTO_LENGTH = 30;

private:
	float mfNNratio;
	bool mbCheckOrientation;
};

