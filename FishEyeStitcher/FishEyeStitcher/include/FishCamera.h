#pragma once
#include <OpencvCommon.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <sstream>
#include <string>
#include <numeric>

#define PRINT_ITERINFO 0


namespace CircleFish
{
	//使用http://wiki.panotools.org/Lens_correction_model给出的矫正模型
	//a*r^4+b*r^3+c*r^2+(1-a-b-c)*r=r_udis
	//鱼眼模型选择r=f*theta
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

	//解一元四次方程a*x^4+b*x^3+c*x^2+d*x+e=0的一个在-e附近的一个近似解
	//先验信息为x的值在0~1之间
	double solvequartic(double a, double b, double c, double d, double e);

	//将球面上的点投影到鱼眼图像
	void mapS2I(const cv::Point3d &s_pt, const FishCamera &C, cv::Point2d &img_pt);

	//将鱼眼图像上的点投影到球面
	void mapI2S(const cv::Point2d &img_pt, const FishCamera &C, cv::Point3d &s_pt);

	void mapRotation(cv::Mat &R, cv::Mat &src, cv::Mat &dst);

	void mapSP2I(cv::Point2d &sp_pt, cv::Point2d &img_pt, int sphere_height, const FishCamera &C);

	//建立图片到球面经纬ROI的map图
	void buildMap(cv::Rect roi, int sphere_height, const FishCamera &C, cv::Mat &map);


}
