#include "CoordinateConvert.h"

#define TRANS_ERROR_SUCCESS 0
#define PI 3.14159265

bool CoordinateConvert::Init(const string& src, const string& dst)
{
	bool bRet = false;

	if (!(this->pj_merc = pj_init_plus(dst.c_str())))
	{
		cerr << "pj_init_plus init error" << endl;
		return bRet;
	}

	if (!(pj_latlong = pj_init_plus(src.c_str())))
	{
		cerr << "pj_init_plus init error" << endl;
		return bRet;
	}

	return true;
}

bool CoordinateConvert::ProjSinglePoint(const cv::Point3d& geoPoint, cv::Point3d& gaussPoint)
{
	int nError = -1;
	gaussPoint.x = geoPoint.x;
	gaussPoint.y = geoPoint.y;
	gaussPoint.z = geoPoint.z;

	gaussPoint.x *= DEG_TO_RAD;
	gaussPoint.y *= DEG_TO_RAD;

	nError = pj_transform(pj_latlong, pj_merc, 1, 1, &gaussPoint.x, &gaussPoint.y, NULL);

	// Unsafe convert precision lost!  [12/1/2017 panan]
	string strX = std::to_string(gaussPoint.x);
	if (strX.size() < 2) {
		return false;
	}
	strX = string(strX.begin() + 2, strX.end());

	gaussPoint.x = boost::lexical_cast<double>(strX);

	return (nError == TRANS_ERROR_SUCCESS);
}

void CoordinateConvert::ProjPoints(const vector<cv::Point3d>& vecGeoPint, vector<cv::Point3d>& vecGaussPoint)
{
	vecGaussPoint.clear();

	for (const auto& it : vecGeoPint)
	{
		cv::Point3d gaussPoint = cv::Point3d(it);
		gaussPoint.x *= DEG_TO_RAD;
		gaussPoint.y *= DEG_TO_RAD;
		if (TRANS_ERROR_SUCCESS != pj_transform(pj_latlong, pj_merc, 1, 1, &gaussPoint.x, &gaussPoint.y, NULL))
		{
			cerr << "pj_transform error during ProjPoints()" << endl;
			continue;
		}
		vecGaussPoint.push_back(gaussPoint);
	}
}

bool CoordinateConvert::InverseProjSinglePoint(const cv::Point3d &gaussPoint, cv::Point3d &geoPoint)
{
    geoPoint.x = gaussPoint.x;
    geoPoint.y = gaussPoint.y;
    geoPoint.z = gaussPoint.z;

    if (TRANS_ERROR_SUCCESS != pj_transform(pj_merc,pj_latlong, 1, 1, &geoPoint.x, &geoPoint.y, NULL))
    {
        cerr << "pj_transform error during ProjPoints()" << endl;
        return false;
    }

    geoPoint.x /= DEG_TO_RAD;
    geoPoint.y /= DEG_TO_RAD;

    return true;
}

void CoordinateConvert::InverseProjPoints(const std::vector<cv::Point3d>& vecGaussPoint, vector<cv::Point3d>& vecGeoPoint)
{
	vecGeoPoint.clear();

	for (const auto& it : vecGaussPoint)
	{
		cv::Point3d gaussPoint = cv::Point3d(it);

		if (TRANS_ERROR_SUCCESS != pj_transform(pj_latlong, pj_merc, 1, 1, &gaussPoint.x, &gaussPoint.y, NULL))
		{
			cerr << "pj_transform error during ProjPoints()" << endl;
			continue;
		}
		gaussPoint.x /= DEG_TO_RAD;
		gaussPoint.y /= DEG_TO_RAD;


		vecGeoPoint.push_back(gaussPoint);
	}
}
