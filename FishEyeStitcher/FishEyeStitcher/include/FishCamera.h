#pragma once
#include <OpencvCommon.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <sstream>
#include <string>
#include <numeric>

#define PRINT_ITERINFO 0


namespace CircleFish
{
	//ʹ��http://wiki.panotools.org/Lens_correction_model�����Ľ���ģ��
	//a*r^4+b*r^3+c*r^2+(1-a-b-c)*r=r_udis
	//����ģ��ѡ��r=f*theta
	class FishCamera
	{
	public:
		FishCamera();

		FishCamera(const FishCamera &other);

		FishCamera(double c_radius, cv::Point center, double Fov = 180.0, double _a = 0.0, double _b = 0.0, double _c = 0.0);

		~FishCamera();

		FishCamera & operator =(const FishCamera &fc);

		double radius;
		double u0, v0;
		double fov;
		double a, b, c;

		double focal_l;
		cv::Mat R;
	};

	//��һԪ�Ĵη���a*x^4+b*x^3+c*x^2+d*x+e=0��һ����-e������һ�����ƽ�
	//������ϢΪx��ֵ��0~1֮��
	double solvequartic(double a, double b, double c, double d, double e);

	//�������ϵĵ�ͶӰ������ͼ��
	void mapS2I(const cv::Point3d &s_pt, const FishCamera &C, cv::Point2d &img_pt);

	//������ͼ���ϵĵ�ͶӰ������
	void mapI2S(const cv::Point2d &img_pt, const FishCamera &C, cv::Point3d &s_pt);

	void mapRotation(cv::Mat &R, cv::Mat &src, cv::Mat &dst);

	void mapSP2I(cv::Point2d &sp_pt, cv::Point2d &img_pt, int sphere_height, const FishCamera &C);

	//����ͼƬ�����澭γROI��mapͼ
	void buildMap(cv::Rect roi, int sphere_height, const FishCamera &C, cv::Mat &map);


}
