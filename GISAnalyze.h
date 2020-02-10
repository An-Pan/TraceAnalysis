//
// Created by ibd02 on 20-2-7.
//

#ifndef TRACEANALYSIS_GISHELPER_H
#define TRACEANALYSIS_GISHELPER_H


#include "gdal_priv.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>

#include "GPSData.h"
#include "CoordinateConvert.h"

using namespace std;
class GISAnalyze {
public:
    GISAnalyze();
    ~GISAnalyze(){};

    void InitGroudTruth(const char* path);

    void InitPredictionTrace(const std::vector<cv::Vec4d>& trace,const std::vector<int>& image_ids);

    void SaveAnalyzeReport(const std::string base_path);

    void CalcHeading(int step = 5);

    void CalcPoints2TraceDis();

    // num : 擬合點數  ndim： 曲線維度
    void CalcVelocityByTrace(int num = 6,int ndim = 5);

//    void CalcVelocityByTrace(const std::vector<cv::Vec4d>& trace,std::vector<cv::Vec2d>& vx_vec,std::vector<cv::Vec2d>& vy_vec);
//
//    int CreateShpFile();

private:
    cv::Mat PolyFit(vector<cv::Point2d>& points,int n);

    // http://blog.sina.com.cn/s/blog_658a93570101hynw.html
    // https://blog.csdn.net/ETNJTOTG/article/details/84130971
    double CalcAzimuth(double lon_a, double lat_a, double lon_b, double lat_b);

private:
    std::shared_ptr<CoordinateConvert> coord_cvt_ptr_;

    std::vector<GPSData> gt_data_vec_;                   // 目標船的真值數據
    std::vector<PredictionData> prediction_data_vec_;            // 感知結果計算

    OGRLineString gt_trace_;

};


#endif //TRACEANALYSIS_GISHELPER_H
