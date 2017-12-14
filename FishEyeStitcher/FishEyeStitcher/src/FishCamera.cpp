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

	bool FishCamera::SaveToXML(cv::FileStorage &fs, int index)
	{
		//Save the CameraModel info
		if (!fs.isOpened() || pModel.use_count() == 0 ||
			pRot.use_count() == 0) return false;

		std::string indexStr = "";
		std::stringstream ioStr;
		if (index >= 0)
		{
			ioStr.str("");
			ioStr << "_C" << index;
			indexStr = ioStr.str();
		}
		

		fs << "CModelName" + indexStr << pModel->getTypeName();
		fs << "f" + indexStr << pModel->f << "u0" + indexStr << pModel->u0 << "v0" + indexStr << pModel->v0;
		fs << "fov" + indexStr << pModel->fov << "maxRadius" + indexStr << pModel->maxRadius;
		int extraParamNum = pModel->vpParameter.size() - 3;
		assert(extraParamNum == 0 || extraParamNum == 2);
		fs << "extraParamNum" + indexStr << extraParamNum;
		
		for (size_t i = 3; i < pModel->vpParameter.size(); i++)
		{
			ioStr.str("");
			ioStr <<  "arg_" << i << indexStr;
			fs << ioStr.str() << *(pModel->vpParameter[i]);
		}

		//Save the Rotation
		fs << "RotationMat" + indexStr << pRot->R;

		return true;
	}

	bool FishCamera::LoadFromXML(cv::FileStorage &fs, int index)
	{
		if (!fs.isOpened()) return false;

		std::string indexStr = "";
		std::stringstream ioStr;
		if (index >= 0)
		{
			ioStr.str("");
			ioStr << "_C" << index;
			indexStr = ioStr.str();
		}

		//Load the CameraModel info
		std::string CModelName;
		fs["CModelName" + indexStr] >> CModelName;
		double f, u0, v0, fov, maxRadius;
		fs["f" + indexStr] >> f; fs["u0" + indexStr] >> u0; fs["v0" + indexStr] >> v0;
		fs["fov" + indexStr] >> fov; fs["maxRadius" + indexStr] >> maxRadius;

		int extraParamNum;
		fs["extraParamNum" + indexStr] >> extraParamNum;
		assert(extraParamNum == 0 || extraParamNum == 2);
		double args[2] = { 0 , 0 };
		for (size_t i = 0; i < extraParamNum; i++)
		{
			ioStr.str("");
			ioStr << "arg_" << i + 3 << indexStr;
			fs[ioStr.str()] >> args[i];
		}

		pModel = createCameraModel(CModelName, u0, v0, f, fov, maxRadius, args[0], args[1]);

		//Load the Rotation
		cv::Mat R;
		fs["RotationMat" + indexStr] >> R;
		pRot = std::make_shared<Rotation>(R);

		return true;
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

		double *R_ptr = reinterpret_cast<double *>((C.pRot->R).data);

		double _x = R_ptr[0] * v_x + R_ptr[1] * v_y + R_ptr[2] * v_z;
		double _y = R_ptr[3] * v_x + R_ptr[4] * v_y + R_ptr[5] * v_z;
		double _z = R_ptr[6] * v_x + R_ptr[7] * v_y + R_ptr[8] * v_z;


		cv::Point3d s_pt(_x, _y, _z);
		C.pModel->mapS2I(s_pt, img_pt);
	}

	//建立图片到球面经纬ROI的map图
	void buildMap(cv::Rect roi, int sphere_height, const FishCamera &C, cv::Mat &map)
	{
		map = cv::Mat(roi.height, roi.width, CV_32FC2, cv::Scalar(0));
		int center = sphere_height * 0.5;
		double fov = CV_PI;
		double pi_sh = fov / sphere_height;


		int x_start = roi.x, y_start = roi.y, x_end = roi.x + roi.width, y_end = roi.y + roi.height;
		double *R_ptr = reinterpret_cast<double *>((C.pRot->R).data);
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
				C.pModel->mapS2I(s_pt, img_pt);

				map_row_ptr[index_x] = img_pt.x;
				map_row_ptr[index_x + 1] = img_pt.y;
			}
		}

	}

	
}
