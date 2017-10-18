#include <ORBMatcher.h>

ORBMatcher::ORBMatcher(float nnratio, bool checkOri)
	: mfNNratio(nnratio), mbCheckOrientation(checkOri) {}

ORBMatcher::~ORBMatcher() {}

// Computes the Hamming distance between two ORB descriptors
int ORBMatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
	const int *pa = a.ptr<int>();
	const int *pb = b.ptr<int>();

	int dist = 0;

	for (int i = 0; i < 8; i++, pa++, pb++)
	{
		unsigned  int v = *pa ^ *pb;
		v = v - ((v >> 1) & 0x55555555);
		v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
		dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
	}

	return dist;
}

void ORBMatcher::ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
	int max1 = 0;
	int max2 = 0;
	int max3 = 0;

	for (int i = 0; i<L; i++)
	{
		const int s = histo[i].size();
		if (s>max1)
		{
			max3 = max2;
			max2 = max1;
			max1 = s;
			ind3 = ind2;
			ind2 = ind1;
			ind1 = i;
		}
		else if (s>max2)
		{
			max3 = max2;
			max2 = s;
			ind3 = ind2;
			ind2 = i;
		}
		else if (s>max3)
		{
			max3 = s;
			ind3 = i;
		}
	}

	if (max2<0.1f*(float)max1)
	{
		ind2 = -1;
		ind3 = -1;
	}
	else if (max3<0.1f*(float)max1)
	{
		ind3 = -1;
	}
}

int ORBMatcher::SearchIndex(KeyAndDescriptor &kap1, KeyAndDescriptor &kap2, IndexContainer &aIndexContainer,
	std::vector<int> &vnMatches12, std::vector<int> &vnMatches21)
{
	int nmatches = 0;
	vnMatches12 = std::vector<int>(kap1.first.size(), -1);
	vnMatches21 = std::vector<int>(kap2.first.size(), -1);

	std::vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i<HISTO_LENGTH; i++)
		rotHist[i].reserve(500);
	const float factor = 1.0f / HISTO_LENGTH;

	std::vector<int> vMatchedDistance(kap2.first.size(), INT_MAX);

	for (size_t i1 = 0, iend1 = kap1.first.size(); i1<iend1; i1++)
	{
		cv::KeyPoint &kp1 = kap1.first[i1];
		int level1 = kp1.octave;
		if (level1>0)
			continue;

		std::vector<size_t> &vIndices2 = aIndexContainer[i1];

		if (vIndices2.empty())
			continue;

		cv::Mat d1 = kap1.second.row(i1);

		int bestDist = INT_MAX;
		int bestDist2 = INT_MAX;
		int bestIdx2 = -1;

		for (std::vector<size_t>::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
		{
			size_t i2 = *vit;

			cv::Mat d2 = kap2.second.row(i2);

			int dist = DescriptorDistance(d1, d2);

			if (vMatchedDistance[i2] <= dist)
				continue;

			if (dist<bestDist)
			{
				bestDist2 = bestDist;
				bestDist = dist;
				bestIdx2 = i2;
			}
			else if (dist<bestDist2)
			{
				bestDist2 = dist;
			}
		}

		if (bestDist <= TH_LOW)
		{
			if (bestDist<(float)bestDist2*mfNNratio)
			{
				if (vnMatches21[bestIdx2] >= 0)
				{
					vnMatches12[vnMatches21[bestIdx2]] = -1;
					nmatches--;
				}
				vnMatches12[i1] = bestIdx2;
				vnMatches21[bestIdx2] = i1;
				vMatchedDistance[bestIdx2] = bestDist;
				nmatches++;

				if (mbCheckOrientation)
				{
					float rot = kap1.first[i1].angle - kap2.first[bestIdx2].angle;
					if (rot<0.0)
						rot += 360.0f;
					int bin = cvRound(rot*factor);
					if (bin == HISTO_LENGTH)
						bin = 0;
					assert(bin >= 0 && bin<HISTO_LENGTH);
					rotHist[bin].push_back(i1);
				}
			}
		}

	}

	if (mbCheckOrientation)
	{
		int ind1 = -1;
		int ind2 = -1;
		int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i<HISTO_LENGTH; i++)
		{
			if (i == ind1 || i == ind2 || i == ind3)
				continue;
			for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
			{
				int idx1 = rotHist[i][j];
				if (vnMatches12[idx1] >= 0)
				{
					int idx2 = vnMatches12[idx1];
					vnMatches21[idx2] = -1;
					vnMatches12[idx1] = -1;
					nmatches--;
				}
			}
		}

	}

	return nmatches;
}

int ORBMatcher::SearchAll(KeyAndDescriptor &kap1, KeyAndDescriptor &kap2,
	std::vector<int> &vnMatches12, std::vector<int> &vnMatches21)
{
	int nmatches = 0;
	vnMatches12 = std::vector<int>(kap1.first.size(), -1);
	vnMatches21 = std::vector<int>(kap2.first.size(), -1);

	std::vector<int> rotHist[HISTO_LENGTH];
	for (int i = 0; i<HISTO_LENGTH; i++)
		rotHist[i].reserve(500);
	const float factor = 1.0f / HISTO_LENGTH;

	std::vector<int> vMatchedDistance(kap2.first.size(), INT_MAX);

	for (size_t i1 = 0, iend1 = kap1.first.size(); i1<iend1; i1++)
	{
		cv::KeyPoint &kp1 = kap1.first[i1];
		int level1 = kp1.octave;
		if (level1>0)
			continue;

		cv::Mat d1 = kap1.second.row(i1);

		int bestDist = INT_MAX;
		int bestDist2 = INT_MAX;
		int bestIdx2 = -1;

		for (size_t i2 = 0, iend2 = kap2.first.size(); i2<iend2; i2++)
		{
			cv::Mat d2 = kap2.second.row(i2);

			int dist = DescriptorDistance(d1, d2);

			if (vMatchedDistance[i2] <= dist)
				continue;

			if (dist<bestDist)
			{
				bestDist2 = bestDist;
				bestDist = dist;
				bestIdx2 = i2;
			}
			else if (dist<bestDist2)
			{
				bestDist2 = dist;
			}
		}

		if (bestDist <= TH_LOW)
		{
			if (bestDist<(float)bestDist2*mfNNratio)
			{
				if (vnMatches21[bestIdx2] >= 0)
				{
					vnMatches12[vnMatches21[bestIdx2]] = -1;
					nmatches--;
				}
				vnMatches12[i1] = bestIdx2;
				vnMatches21[bestIdx2] = i1;
				vMatchedDistance[bestIdx2] = bestDist;
				nmatches++;

				if (mbCheckOrientation)
				{
					float rot = kap1.first[i1].angle - kap2.first[bestIdx2].angle;
					if (rot<0.0)
						rot += 360.0f;
					int bin = cvRound(rot*factor);
					if (bin == HISTO_LENGTH)
						bin = 0;
					assert(bin >= 0 && bin<HISTO_LENGTH);
					rotHist[bin].push_back(i1);
				}
			}
		}

	}

	if (mbCheckOrientation)
	{
		int ind1 = -1;
		int ind2 = -1;
		int ind3 = -1;

		ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

		for (int i = 0; i<HISTO_LENGTH; i++)
		{
			if (i == ind1 || i == ind2 || i == ind3)
				continue;
			for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
			{
				int idx1 = rotHist[i][j];
				if (vnMatches12[idx1] >= 0)
				{
					int idx2 = vnMatches12[idx1];
					vnMatches21[idx2] = -1;
					vnMatches12[idx1] = -1;
					nmatches--;
				}
			}
		}

	}

	return nmatches;
}