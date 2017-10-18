#pragma once
#include <vector>
#include <iostream>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <cmath>

template <class T>
class RansacBase
{
public:
	RansacBase(double bad_threshold, double confidence, int iterate_num,bool use_confidence = true)
	{
		m_N = iterate_num;
		m_confidence = confidence;
		m_bad_threshold = bad_threshold;
		m_use_confidence = use_confidence;
	}
	~RansacBase(){}

	double run(const std::vector< std::vector<T> > &points, std::vector<char> &mask)
	{
		int total = points.size();
		std::vector<char> mask_temp(total, 0);
	    //_ASSERT(total > m_c && m_c > 0);
		int iterate_num = m_N;
		srand((int)time(0));
		std::vector<std::vector<T> > rand_points(m_c);
		m_interior_ratio = 0;
		std::vector<double> best_Model(m_coff_count);
		m_Model.resize(m_coff_count);
		for (int i = 0; i < iterate_num; i++)
		{
			for (int j = 0; j < m_c; j++)
			{
				rand_points[j] = points[rand() % total];
			}

			if (!estimateModel(rand_points))continue;

			double temp_interior_ratio = _getInteriorRatio(points);
			if (temp_interior_ratio > m_interior_ratio)
			{
				best_Model = m_Model;
				m_interior_ratio = temp_interior_ratio;
				if (m_use_confidence)
				{
					int temp_iterate_num = (int)(log(1 - m_confidence) / log(1 - pow(m_interior_ratio, m_c)));
					if (temp_iterate_num < iterate_num)iterate_num = temp_iterate_num;
				}
			}
		}
		m_Model = best_Model;

		for (size_t i = 0; i < total; i++)
			if (getError(points[i]) <= m_bad_threshold)mask_temp[i] = 1;
		mask = mask_temp;
		return m_interior_ratio;
	}

	double refine(const std::vector<std::vector<T> > &points, std::vector<char> &mask, size_t iter = 1)
	{
		int total = points.size();
		for (size_t i = 0; i < iter; i++)
		{
			averageModel(points, mask);
			for (size_t j = 0; j < total; j++)
				mask[j] = getError(points[j]) <= m_bad_threshold ? 1 : 0;
		}

		int inlier_count = 0;
		for (size_t i = 0; i < total; i++)
			if (mask[i] == 1)inlier_count++;

		return inlier_count / (double)total;
	}

protected:
	//virtual void project(const std::vector<T> &src_point, std::vector<T> &dst_point) = 0;

	virtual bool estimateModel(const std::vector<std::vector<T> > &points) = 0;

	virtual double getError(const std::vector<T> &point) = 0;

	virtual void averageModel(const std::vector<std::vector<T> > &points, std::vector<char> &mask)
	{

	}

	std::vector<double> m_Model;

	int m_c;

	int m_coff_count;

	bool m_use_confidence;

	double _getInteriorRatio(const std::vector<std::vector<T> > &points)
	{
		
		int outeriors_count = 0;
		int totals = points.size();
		int cur_bads = totals * (1 - m_interior_ratio);
		for (int i = 0; i < totals; i++)
		{
			if (getError(points[i]) > m_bad_threshold)outeriors_count++;
			if (outeriors_count > cur_bads)return 0.0;
		}
		return (totals - outeriors_count) / (double)totals;
	}

	int m_N;
	
	double m_confidence;
	double m_bad_threshold;
	double m_interior_ratio;
};

class RansacCircle : public RansacBase<int>
{
public:
	RansacCircle(double bad_threshold, double confidence, int iterate_num, bool use_confidence = true)
		: RansacBase<int>(bad_threshold, confidence, iterate_num, use_confidence)
	{
		m_coff_count = 3;
		m_c = 3;
	}
	~RansacCircle() {}

	void getCircle(double &x, double &y, double &radius_square)
	{
		x = m_Model[0];
		y = m_Model[1];
		radius_square = m_Model[2];
	}

protected:
	virtual bool estimateModel(const std::vector<std::vector<int> > &points)
	{
		//_ASSERT(points.size() == m_c && m_c == 3);
		//_ASSERT(points[0].size() == 2);

		std::vector<int> line01;
		_cross(points[0], points[1], line01);
		std::vector<int> points2(3);
		points2[0] = points[2][0];
		points2[1] = points[2][1];
		points2[2] = 1;
		if (std::abs(_dot(points2, line01)) < 1e-6)return false;

		std::vector<int> line02;
		_cross(points[0], points[2], line02);

		std::vector<int> line01_median(3);
		line01_median[0] = -line01[1];
		line01_median[1] = line01[0];
		line01_median[2] = (line01[1] * (points[0][0] + points[1][0]) - line01[0] * (points[0][1] + points[1][1])) / 2;

		std::vector<int> line02_median(3);
		line02_median[0] = -line02[1];
		line02_median[1] = line02[0];
		line02_median[2] = (line02[1] * (points[0][0] + points[2][0]) - line02[0] * (points[0][1] + points[2][1])) / 2;

		std::vector<int> circle_center;
		_cross(line02_median, line01_median, circle_center);
		m_Model[0] = circle_center[0] / (double)circle_center[2];
		m_Model[1] = circle_center[1] / (double)circle_center[2];

		m_Model[2] = (m_Model[0] - points[0][0])*(m_Model[0] - points[0][0]) + (m_Model[1] - points[0][1])*(m_Model[1] - points[0][1]);
		return m_Model[0] < 0 || m_Model[1] < 0 || m_Model[2] < 0 ? false : true;
	}

	virtual double getError(const std::vector<int> &point)
	{
		//_ASSERT(point.size() == 2);
		return std::abs(sqrt((m_Model[0] - point[0])*(m_Model[0] - point[0]) + (m_Model[1] - point[1])*(m_Model[1] - point[1])) - sqrt(m_Model[2]));
	}

private:
	int _dot(const std::vector<int>& point1, const std::vector<int> &point2)
	{
		int dot_result = 0;
		//_ASSERT(point1.size() == point2.size());
		for (size_t i = 0; i < point1.size(); i++)
		{
			dot_result += point1[i] * point2[i];
		}
		return dot_result;
	}

	void _cross(const std::vector<int>& point1, const std::vector<int> &point2, std::vector<int> &result)
	{
		//_ASSERT(point1.size() == point2.size() && (point1.size() == 2 || point1.size() == 3));
		result.resize(3);
		if (point1.size() == 2)
		{
			result[0] = point1[1] - point2[1];
			result[1] = point2[0] - point1[0];
			result[2] = point1[0] * point2[1] - point2[0] * point1[1];
		}
		else
		{
			result[0] = point1[1] * point2[2] - point2[1] * point1[2];
			result[1] = point2[0] * point1[2] - point1[0] * point2[2];
			result[2] = point1[0] * point2[1] - point2[0] * point1[1];
		}
	}
};

class RansacRotation : public RansacBase<double>
{
public:
	RansacRotation(double bad_threshold, double confidence, int iterate_num, bool use_confidence = true)
		: RansacBase<double>(bad_threshold, confidence, iterate_num, use_confidence)
	{
		m_coff_count = 9;
		m_c = 2;
	}
	~RansacRotation() {}

	void getRotation(cv::Mat &R)
	{
		cv::Mat R_temp(3, 3, CV_64FC1, &m_Model[0]);
		R_temp.copyTo(R);
	}

private:

	virtual bool estimateModel(const std::vector<std::vector<double> > &points)
	{
		//_ASSERT(points.size() == m_c && m_c == 2);
		//_ASSERT(points[0].size() == 6);

		cv::Vec3d dist0_vect(points[0][0] - points[0][3], points[0][1] - points[0][4], points[0][2] - points[0][5]);
		cv::Vec3d dist1_vect(points[1][0] - points[1][3], points[1][1] - points[1][4], points[1][2] - points[1][5]);

		double c = dist0_vect.dot(dist1_vect) / (cv::norm(dist0_vect)*cv::norm(dist1_vect));
		//std::cout<<"There is some doubt about abs : std::abs( 1 - c) = "<< std::abs(1-c)<<std::endl;
		if (std::abs(1 - c) < 1e-10)
			return false;

		cv::Mat H = cv::Mat(3, 3, CV_64FC1, cv::Scalar(0));
		double *H_ptr = reinterpret_cast<double *>(H.data);

		for (size_t i = 0; i < 2; i++)
		{
			H_ptr[0] += points[i][0] * points[i][3];
			H_ptr[1] += points[i][0] * points[i][4];
			H_ptr[2] += points[i][0] * points[i][5];
			H_ptr[3] += points[i][1] * points[i][3];
			H_ptr[4] += points[i][1] * points[i][4];
			H_ptr[5] += points[i][1] * points[i][5];
			H_ptr[6] += points[i][2] * points[i][3];
			H_ptr[7] += points[i][2] * points[i][4];
			H_ptr[8] += points[i][2] * points[i][5];
		}

		cv::SVD svd;
		cv::Mat w, u, vt;
		svd.compute(H, w, u, vt);

		cv::Mat R(3, 3, CV_64FC1, &m_Model[0]);

		R = vt.t()*u.t();
		return true;
	}

	virtual double getError(const std::vector<double> &point)
	{
		//_ASSERT(point.size() == 6);

		cv::Vec3d R_p1;
		R_p1[0] = m_Model[0] * point[0] + m_Model[1] * point[1] + m_Model[2] * point[2];
		R_p1[1] = m_Model[3] * point[0] + m_Model[4] * point[1] + m_Model[5] * point[2];
		R_p1[2] = m_Model[6] * point[0] + m_Model[7] * point[1] + m_Model[8] * point[2];

		cv::Vec3d p2(point[3], point[4], point[5]);
		cv::Vec3d r = p2 - R_p1;

		return cv::norm(r);
	}

};

class RansacShift : public RansacBase<float>
{
public:
	RansacShift(double bad_ratio, double confidence, int iterate_num, bool use_confidence = true)
		: RansacBase<float>(0, confidence, iterate_num, use_confidence)
	{
		m_coff_count = 2;
		m_c = 1;
		mbad_ratio = bad_ratio;
	}
	~RansacShift() {}

	void getShift(float &x, float &y)
	{
		x = m_Model[0];
		y = m_Model[1];
	}

protected:
	virtual bool estimateModel(const std::vector<std::vector<float> > &points)
	{
		//_ASSERT(points.size() == m_c && m_c == 1);
		//_ASSERT(points[0].size() == 2);

		m_Model[0] = points[0][0];
		m_Model[1] = points[0][1];

		m_bad_threshold = sqrt((m_Model[0] * m_Model[0] + m_Model[1] * m_Model[1]))*mbad_ratio;

		return true;
	}

	virtual double getError(const std::vector<float> &point)
	{
		//_ASSERT(point.size() == 2);
		return std::abs(sqrt((m_Model[0] - point[0])*(m_Model[0] - point[0]) + (m_Model[1] - point[1])*(m_Model[1] - point[1])));
	}

	virtual void averageModel(const std::vector<std::vector<float> > &points, std::vector<char> &mask)
	{
		float temp_model[2] = { 0 };
		size_t total = points.size();
		for (size_t i = 0, count = 0; i < total; i++)
		{
			if (mask[i] == 1)
			{
				float count_1_inv = 1.0 / (count + 1);
				temp_model[0] = temp_model[0] * (count_1_inv * count) + count_1_inv*points[i][0];
				temp_model[1] = temp_model[1] * (count_1_inv * count) + count_1_inv*points[i][1];
				count++;
			}
		}

		m_Model[0] = temp_model[0];
		m_Model[1] = temp_model[1];

		m_bad_threshold = sqrt((m_Model[0] * m_Model[0] + m_Model[1] * m_Model[1]))*mbad_ratio;

	}

private:

	double mbad_ratio;
};
