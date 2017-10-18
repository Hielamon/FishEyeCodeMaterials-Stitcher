#include <FishCamera.h>

namespace CircleFish
{

	FishCamera::FishCamera()
	{
		radius = u0 = v0 = a = b = c = 0;
		fov = CV_PI;
		focal_l = 2 * radius / fov;
		R = cv::Mat::eye(3, 3, CV_64F);
	}

	FishCamera::FishCamera(const FishCamera &other)
	{
		radius = other.radius;
		u0 = other.u0;
		v0 = other.v0;
		fov = other.fov;
		a = other.a;
		b = other.b;
		c = other.c;

		focal_l = other.focal_l;

		other.R.copyTo(R);
	}

	FishCamera::FishCamera(double c_radius, cv::Point center, double Fov, double _a, double _b, double _c)
	{
		radius = c_radius;
		u0 = center.x;
		v0 = center.y;
		fov = Fov*CV_PI / 180;
		a = _a;
		b = _b;
		c = _c;

		focal_l = 2 * radius / fov;

		R = cv::Mat::eye(3, 3, CV_64F);
	}

	FishCamera::~FishCamera() {}

	FishCamera & FishCamera::operator =(const FishCamera &fc)
	{
		radius = fc.radius;
		u0 = fc.u0;
		v0 = fc.v0;
		fov = fc.fov;
		a = fc.a;
		b = fc.b;
		c = fc.c;

		focal_l = fc.focal_l;

		fc.R.copyTo(R);
		return *this;
	}

	//解一元四次方程a*x^4+b*x^3+c*x^2+d*x+e=0的一个在-e附近的一个近似解
	//先验信息为x的值在0~1之间
	double solvequartic(double a, double b, double c, double d, double e)
	{
		double eps = 1e-8;
		double x0 = -e;
		double y0 = (((a*x0 + b)*x0 + c)*x0 + d)*x0 + e;
		if (std::abs(y0) < eps)return x0;

		double k0 = ((4 * a*x0 + 3 * b)*x0 + 2 * c)*x0 + d;
		double init_step = y0*k0 < 0 ? 0.1 : -0.1;

		double x1 = x0 + init_step;
		double y1 = (((a*x1 + b)*x1 + c)*x1 + d)*x1 + e;

		int iter_num = 0;
		while (std::abs(y1) > eps && iter_num < 1000)
		{
			if (y1*y0 > 0)
			{
				x0 = x1;
				y0 = y1;
				x1 += init_step;
			}
			else
			{
				init_step *= 0.5;
				x1 = x0 + init_step;
			}

			y1 = (((a*x1 + b)*x1 + c)*x1 + d)*x1 + e;
			iter_num++;
		}

		return x1;
	}

	//将球面上的点投影到鱼眼图像
	void mapS2I(const cv::Point3d &s_pt, const FishCamera &C, cv::Point2d &img_pt)
	{
		double theta = atan2(s_pt.y, s_pt.x);
		double phi = atan2(sqrt(s_pt.x*s_pt.x + s_pt.y*s_pt.y), s_pt.z);
		double r_udis_n = std::abs(phi * 2 / C.fov);
		double d = 1 - C.a - C.b - C.c;
		double r_dis_n = (((C.a*r_udis_n + C.b)*r_udis_n + C.c)*r_udis_n + d)*r_udis_n;
		double r_dis = C.radius*r_dis_n;

		img_pt.x = r_dis*cos(theta) + C.u0;
		img_pt.y = -r_dis*sin(theta) + C.v0;
	}

	//将鱼眼图像上的点投影到球面
	void mapI2S(const cv::Point2d &img_pt, const FishCamera &C, cv::Point3d &s_pt)
	{
		double x = img_pt.x - C.u0;
		double y = -img_pt.y + C.v0;
		double r_dis = sqrt(x*x + y*y);
		double r_dis_n = r_dis / C.radius;

		double r_udis_n = solvequartic(C.a, C.b, C.c, (1 - C.a - C.b - C.c), -r_dis_n);
		double theta = atan2(y, x);

		double phi = r_udis_n*C.fov*0.5;

		s_pt.x = sin(phi)*cos(theta);
		s_pt.y = sin(phi)*sin(theta);
		s_pt.z = cos(phi);
	}

	void mapRotation(cv::Mat &R, cv::Mat &src, cv::Mat &dst)
	{
		assert(src.rows * 2 == src.cols);

		int height = src.rows;
		double pi_sh = CV_PI / height;
		double sh_pi = height / CV_PI;

		double *R_ptr = (double *)R.data;

		int center_x = src.cols * 0.5;
		int center_y = src.rows * 0.5;

		cv::Mat map = cv::Mat(src.rows, src.cols, CV_32FC2, cv::Scalar(0));

		for (int i = 0; i < map.rows; i++)
		{
			double * map_row_ptr = (double *)map.ptr(i);
			for (int j = 0, idx = 0; j < map.cols; j++, idx += 2)
			{
				double theta = (j - center_x) * pi_sh;
				double phi = -(i - center_y) * pi_sh;

				double v_x = cos(phi)*sin(theta);
				double v_y = sin(phi);
				double v_z = cos(phi)*cos(theta);

				double _x = R_ptr[0] * v_x + R_ptr[3] * v_y + R_ptr[6] * v_z;
				double _y = R_ptr[1] * v_x + R_ptr[4] * v_y + R_ptr[7] * v_z;
				double _z = R_ptr[2] * v_x + R_ptr[5] * v_y + R_ptr[8] * v_z;

				theta = atan2(_x, _z);
				phi = atan2(_y, sqrt(_x*_x + _z*_z));

				double x_coord = theta*sh_pi + center_x;
				double y_coord = -phi*sh_pi + center_y;
				map_row_ptr[idx] = x_coord;
				map_row_ptr[idx + 1] = y_coord;
			}
		}
		cv::Mat map1, map2;
		cv::convertMaps(map, cv::Mat(), map1, map2, CV_16SC2);
		cv::remap(src, dst, map1, map2, CV_INTER_CUBIC);
	}

	void mapSP2I(cv::Point2d &sp_pt, cv::Point2d &img_pt, int sphere_height, const FishCamera &C)
	{
		double fov = CV_PI;
		double pi_sh = fov / sphere_height;
		int center = sphere_height * 0.5;
		int dcoord_x = sp_pt.x - center;
		int dcoord_y = -sp_pt.y + center;
		double theta = dcoord_x * pi_sh;
		double phi = dcoord_y * pi_sh;
		// Vector in 3D space
		double v_x = cos(phi)*sin(theta);
		double v_y = sin(phi);
		double v_z = cos(phi)*cos(theta);

		double *R_ptr = reinterpret_cast<double *>((C.R).data);

		double _x = R_ptr[0] * v_x + R_ptr[1] * v_y + R_ptr[2] * v_z;
		double _y = R_ptr[3] * v_x + R_ptr[4] * v_y + R_ptr[5] * v_z;
		double _z = R_ptr[6] * v_x + R_ptr[7] * v_y + R_ptr[8] * v_z;


		cv::Point3d s_pt(_x, _y, _z);
		mapS2I(s_pt, C, img_pt);
	}

	//建立图片到球面经纬ROI的map图
	void buildMap(cv::Rect roi, int sphere_height, const FishCamera &C, cv::Mat &map)
	{
		map = cv::Mat(roi.height, roi.width, CV_32FC2, cv::Scalar(0));
		int center = sphere_height * 0.5;
		double fov = CV_PI;
		double pi_sh = fov / sphere_height;


		int x_start = roi.x, y_start = roi.y, x_end = roi.x + roi.width, y_end = roi.y + roi.height;
		double *R_ptr = reinterpret_cast<double *>((C.R).data);
		for (int i = y_start, index_y = 0; i < y_end; i++, index_y++)
		{
			float *map_row_ptr = reinterpret_cast<float *>(map.ptr(index_y));
			for (int j = x_start, index_x = 0; j < x_end; j++, index_x += 2)
			{
				int dcoord_x = j - center;
				int dcoord_y = -i + center;
				double theta = dcoord_x * pi_sh;
				double phi = dcoord_y * pi_sh;
				// Vector in 3D space
				double v_x = cos(phi)*sin(theta);
				double v_y = sin(phi);
				double v_z = cos(phi)*cos(theta);

				double _x = R_ptr[0] * v_x + R_ptr[1] * v_y + R_ptr[2] * v_z;
				double _y = R_ptr[3] * v_x + R_ptr[4] * v_y + R_ptr[5] * v_z;
				double _z = R_ptr[6] * v_x + R_ptr[7] * v_y + R_ptr[8] * v_z;


				cv::Point2d img_pt;
				cv::Point3d s_pt(_x, _y, _z);
				mapS2I(s_pt, C, img_pt);

				map_row_ptr[index_x] = img_pt.x;
				map_row_ptr[index_x + 1] = img_pt.y;
			}
		}

	}

}
