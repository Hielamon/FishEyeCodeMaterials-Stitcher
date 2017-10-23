#pragma once
#include <OpencvCommon.h>

class CameraModel
{
public:
	CameraModel(double _u0, double _v0, double _f) :
		u0(_u0), v0(_v0), f(_f)
	{
		assert(f > 0);
		vpParameter.push_back(&u0);
		vpParameter.push_back(&v0);
		vpParameter.push_back(&f);
	}

	
	~CameraModel() {}

	//After change the projecting parameters of a camera model 
	//We need to update the Fov of the Camera if need
	void updateFov()
	{
		double tmpAngle;
		inverseProject(maxRadius / f, tmpAngle);
		fov = tmpAngle * 2;
	}

	//mapping the image coordinate to the unit sphere coordinate	
	virtual bool mapI2S(const cv::Point2d &imgPt, cv::Point3d &spherePt)
	{
		double x = (imgPt.x - u0) / f;
		double y = (-imgPt.y + v0) / f;
		double r_dist = sqrt(x*x + y*y);

		double theta, phi;
		theta = atan2(y, x);

		if (!inverseProject(r_dist, phi))
		{
			std::cout << "Warning: Invalid mapping in mapI2S" << std::endl;
			return false;
		}

		spherePt.x = sin(phi)*cos(theta);
		spherePt.y = sin(phi)*sin(theta);
		spherePt.z = cos(phi);
		return true;
	}

	//mapping the unit sphere coordinate to the image coordinate	
	virtual bool mapS2I(const cv::Point3d &spherePt, cv::Point2d &imgPt)
	{
		double theta, phi, r_dist;
		theta = atan2(spherePt.y, spherePt.x);
		phi = atan2(sqrt(spherePt.x*spherePt.x + spherePt.y*spherePt.y), spherePt.z);

		if (phi * 2 > fov)
			return false;

		if (!project(phi, r_dist))
		{
			std::cout << "Warning: Invalid mapping in mapS2I" << std::endl;
			return false;
		}

		imgPt.x = r_dist*cos(theta)*f + u0;
		imgPt.y = -r_dist*sin(theta)*f + v0;
		return true;
	}

	//projecting the imaging radius to the incident angle
	virtual bool inverseProject(const double& radius, double &angle)
	{
		if (radius < 0)
		{
			return false;
		}
		angle = atan(radius);
		return true;
	}

	//projecting the incident angle to the imaging radius
	virtual bool project(const double& angle, double &radius)
	{
		if (angle < 0 || angle >= CV_PI*0.5)
		{
			return false;
		}

		radius = tan(angle);
		return true;
	}

	virtual std::string getTypeName()
	{
		return "Default";
	}

	double u0, v0, f;
	double fov, maxRadius;

	//The array of parameters point, which is arrange as
	//vParameter[0:3] = {&u0, &v0, &f}
	//which can be extended in the derive class
	std::vector<double*> vpParameter;

private:
	CameraModel() {}
};

namespace FishEye
{
	//The solver for quadratic equation with one unknown
	//a*x^2 + b*x + c = 0
	inline std::vector<double> solverUnitaryQuadratic(const double &a, const double &b, const double &c)
	{
		std::vector<double> result;
		result.reserve(2);
		if (a == 0)
		{
			if (b != 0)
			{
				result.push_back(-c / b);
			}
		}
		else
		{
			double delta = b*b - 4 * a*c;
			if (delta >= 0)
			{
				if (delta > 0)
				{
					double sqrtDelta = sqrt(delta);
					result.push_back((-b + sqrtDelta) / (2 * a));
					result.push_back((-b - sqrtDelta) / (2 * a));
				}
				else
				{
					result.push_back(-b * 0.5 / a);
				}
			}
		}
		return result;
	}


	class Equidistant : public CameraModel
	{
	public:
		Equidistant(double _u0, double _v0, double _f, double _fov) :
			CameraModel(_u0, _v0, _f)
		{
			fov = _fov;
			project(fov * 0.5, maxRadius);
			maxRadius = maxRadius * f;
		}
		~Equidistant() {}

		//projecting the imaging radius to the incident angle
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{
				return false;
			}
			angle = radius;
			return true;
		}

		//projecting the incident angle to the imaging radius
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			radius = angle;
			return true;
		}

		virtual std::string getTypeName()
		{
			return "Equidistant";
		}
	};

	class Equisolid : public CameraModel
	{
	public:
		Equisolid(double _u0, double _v0, double _f, double _fov) :
			CameraModel(_u0, _v0, _f)
		{
			fov = _fov;
			project(fov * 0.5, maxRadius);
			maxRadius = maxRadius * f;
		}
		~Equisolid() {}

		//projecting the imaging radius to the incident angle
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{
				return false;
			}

			angle = asin(radius * 0.5) * 2;
			return true;
		}

		//projecting the incident angle to the imaging radius
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			radius = 2 * sin(angle*0.5);
			return true;
		}

		virtual std::string getTypeName()
		{
			return "Equisolid";
		}
	};

	class Stereographic : public CameraModel
	{
	public:
		Stereographic(double _u0, double _v0, double _f, double _fov) :
			CameraModel(_u0, _v0, _f)
		{
			fov = _fov;
			project(fov * 0.5, maxRadius);
			maxRadius = maxRadius * f;
		}
		~Stereographic() {}

		//projecting the imaging radius to the incident angle
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{ 
				return false;
			}
			angle = atan(radius * 0.5) * 2;
			return true;
		}

		//projecting the incident angle to the imaging radius
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			radius = 2 * tan(angle*0.5);
			return true;
		}

		virtual std::string getTypeName()
		{
			return "Stereographic";
		}
	};


	//Refer to : A Generic Camera Model and Calibration Method for Conventional, Wide-Angle, and Fish-Eye Lenses
	class PolynomialAngle : public CameraModel
	{
	public:
		PolynomialAngle(double _u0, double _v0, double _f, double _maxRadius,
						double _k1 = 1.0, double _k2 = 0.0) :
			k1(_k1), k2(_k2), CameraModel(_u0, _v0, _f)
		{
			maxRadius = _maxRadius;
			double tmpRadius = maxRadius / f;
			double tmpFov;
			inverseProject(tmpRadius, tmpFov);
			fov = 2 * tmpFov;

			//add extra parameters reference
			vpParameter.push_back(&k1);
			vpParameter.push_back(&k2);
		}
		~PolynomialAngle() {}

		//projecting the imaging radius to the incident angle
		//radius = k1 * (angle) + k2 * (angle)^3
		//According to https://zh.wikipedia.org/wiki/%E4%B8%89%E6%AC%A1%E6%96%B9%E7%A8%8B
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{
				return false;
			}

			double a = k2, c = k1, d = -radius;
			if (a == 0)
			{
				if (c == 0)
				{
					return false;
				}

				angle = -d / c;
				//return true;
			}
			else
			{
				double p = c / a, q = d / a;
				double p3 = p*p*p, q2 = q*q;
				double delta = q2 + 4 * p3 / 27;
				if (delta > 0)
				{
					double u3 = (-q + sqrt(delta))*0.5;
					double v3 = (-q - sqrt(delta))*0.5;
					angle = customPow(u3, 1.0 / 3) + customPow(v3, 1.0 / 3);
				}
				else if (delta == 0)
				{
					if (p == 0)
					{
						angle = 0;
					}
					else
					{
						angle = customPow(q*0.2, 1.0 / 3);
						angle = angle > 0 ? angle : -angle;
					}
				}
				else
				{
					double Q = -p / 3.0, R = -q * 0.5;
					double sqrtQ = sqrt(Q);
					double theta = acos(R / (Q * sqrtQ));

					//The third root {2 * sqrtQ*cos((theta + CV_2PI)} is negative and excluded
					double angle1 = 2 * sqrtQ*cos(theta / 3.0);
					double angle2 = 2 * sqrtQ*cos((theta - CV_2PI) / 3.0);
					
					if (angle1 < CV_PI && angle2 < CV_PI)
					{
						angle = std::min(angle1, angle2);
						return false;
					}
					else 
					{
						angle = angle1 < CV_PI ? angle1 : angle2;
						if (angle > CV_PI)
						{
							return false;
						}
					}
				}
			}

			return true;
		}

		//projecting the incident angle to the imaging radius
		//radius = k1 * (angle) + k2 * (angle)^3
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			radius = k1 * angle + k2 * angle * angle * angle;

			return radius >= 0;
		}

		virtual std::string getTypeName()
		{
			return "PolynomialAngle";
		}

		double k1, k2;
	};

	//Refer to : A Toolbox for Easily Calibrating Omnidirectional
	class PolynomialRadius : public CameraModel
	{
	public:
		PolynomialRadius( double _u0, double _v0, double _f, double _maxRadius,
						 double _a0 = 1.0, double _a2 = 0.0) :
			a0(_a0), a2(_a2), CameraModel( _u0, _v0, _f)
		{
			maxRadius = _maxRadius;
			double tmpRadius = maxRadius / f;
			double tmpFov;
			inverseProject(tmpRadius, tmpFov);
			fov = 2 * tmpFov;

			vpParameter.push_back(&a0);
			vpParameter.push_back(&a2);
		}
		~PolynomialRadius() {}

		//projecting the imaging radius to the incident angle
		//rd / (a0 + a2*rd^2) = sin(theta) / cos(theta)
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{
				return false;
			}

			angle = atan2(radius, a0 + a2*radius*radius);
			return true;
		}

		//projecting the incident angle to the imaging radius
		//rd / (a0 + a2*rd^2) = sin(theta) / cos(theta)
		//rd^2*a2*sin(theta) - rd*cos(theta) + a0*sin(theta) = 0
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			bool isInFov = angle <= fov * 0.5;

			double a = a2 * sin(angle), b = -cos(angle), c = a0 * sin(angle);
			std::vector<double> root = solverUnitaryQuadratic(a, b, c);
			radius = -1;
			bool unique = false;

			for (size_t i = 0; i < root.size(); i++)
			{
				if (root[i] >= 0)
				{
					if (isInFov && root[i] > (maxRadius / f)) continue;
					radius = root[i];
					unique = !unique;
				}
			}
			
			return unique;
		}

		virtual std::string getTypeName()
		{
			return "PolynomialRadius";
		}

		double a0, a2;
	};

	//Refer to : A unifying theory for central panoramic systems and practical implications
	class GeyerModel : public CameraModel
	{
	public:
		GeyerModel( double _u0, double _v0, double _f, double _maxRadius,
				   double _m = 1.0, double _l = 0.0) :
			m(_m), l(_l), CameraModel(_u0, _v0, _f)
		{
			maxRadius = _maxRadius;
			double tmpRadius = maxRadius / f;
			double tmpFov;
			inverseProject(tmpRadius, tmpFov);
			fov = 2 * tmpFov;

			vpParameter.push_back(&m);
			vpParameter.push_back(&l);
		}
		~GeyerModel() {}

		//projecting the imaging radius to the incident angle
		//rd = (m + l)*sin(theta) / ( l + cos(theta))
		//d + e*cos(theta) = sin(theta)
		virtual bool inverseProject(const double& radius, double &angle)
		{
			if (radius < 0)
			{
				return false;
			}

			//bool isInFov = radius <= maxRadius;

			double d = radius * l / (m + l);
			double e = radius / (m + l);

			//d^2 - 1 + 2*d*e*cos(theta) + (e^2 + 1)*cos(theta)^2 = 0
			double a = e*e + 1;
			double b = 2 * d * e;
			double c = d*d - 1;
			double cosValue = 0;

			std::vector<double> root = solverUnitaryQuadratic(a, b, c);
			int rootNum = root.size();
			angle = -1;
			bool unique = false;

			//obtain the smaller one
			for (size_t i = 0; i < root.size(); i++)
			{
				double cosValue = root[i];
				if (abs(cosValue) <= 1)
				{
					double tmpAngle = acos(cosValue);
					double left = d + e * cosValue;
					double right = sin(tmpAngle);
					if (left * right >= 0)
					{
						if (!unique)
						{
							angle = tmpAngle;
							unique = !unique;
						}
						else if (tmpAngle < angle)
						{
							angle = tmpAngle;
						}
					}
				}
			}

			return unique;
		}

		//projecting the incident angle to the imaging radius
		//rd = (m + l)*sin(theta) / ( l + cos(theta))
		virtual bool project(const double& angle, double &radius)
		{
			if (angle < 0 || angle > CV_PI)
			{
				return false;
			}

			radius = (m + l) * sin(angle) / (l + cos(angle));
			
			return radius >= 0;
		}

		virtual std::string getTypeName()
		{
			return "GeyerModel";
		}

		double l, m;
	};

	
}

inline std::shared_ptr<CameraModel> createCameraModel(const std::string &typeName, double _u0, double _v0, double _f, 
													  double _fov, double _maxRadius,
													  double arg1 = 1.0, double arg2 = 0.0)
{
	std::shared_ptr<CameraModel> result = std::make_shared<CameraModel>(_u0, _v0, _f);
	if (typeName == "Equidistant")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::Equidistant>(_u0, _v0, _f, _fov));
	}
	else if (typeName == "Equisolid")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::Equisolid>(_u0, _v0, _f, _fov));
	}
	else if (typeName == "Stereographic")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::Stereographic>(_u0, _v0, _f, _fov));
	}
	else if (typeName == "PolynomialAngle")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::PolynomialAngle>(
			_u0, _v0, _f, _maxRadius, arg1, arg2));
	}
	else if (typeName == "PolynomialRadius")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::PolynomialRadius>(
			_u0, _v0, _f, _maxRadius, arg1, arg2));
	}
	else if (typeName == "GeyerModel")
	{
		result = std::static_pointer_cast<CameraModel>(std::make_shared<FishEye::GeyerModel>(
			_u0, _v0, _f, _maxRadius, arg1, arg2));
	}

	return result;
}




