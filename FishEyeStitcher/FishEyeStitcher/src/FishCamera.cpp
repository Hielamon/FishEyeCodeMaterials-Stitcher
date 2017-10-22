#include <FishCamera.h>


namespace CircleFish
{

	FishCamera::FishCamera() {}

	FishCamera::FishCamera(const std::shared_ptr<CameraModel> & _pModel, const std::shared_ptr<Rotation> & _pRot)
	{
		pModel = _pModel;
		pRot = _pRot;
	}

	FishCamera::~FishCamera() {}

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
