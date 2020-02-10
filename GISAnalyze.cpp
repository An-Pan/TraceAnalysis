//
// Created by ibd02 on 20-2-7.
//

#include "GISAnalyze.h"
#include "ogrsf_frmts.h"

#include <iostream>


#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>


GISAnalyze::GISAnalyze() {

    coord_cvt_ptr_ = std::make_shared<CoordinateConvert>();

    if(!coord_cvt_ptr_->Init("+proj=latlong +ellps=WGS84  +datum=WGS84 +no_defs", "+proj=tmerc +lat_0=0 +lon_0=114 +k=1 +x_0=38500000 +y_0=0 +ellps=GRS80 +units=m +no_defs")){
        return ;
    }
}

void GISAnalyze::InitGroudTruth(const char *path) {
    std::ifstream file_in(path);
    char buf[2048]={0};

    while(file_in.getline(buf,2048)){

        string strbuf = buf;
        vector<string> items;
        boost::split(items,strbuf,boost::is_any_of(","));

        if(items.size() < 12)
            continue;
        double time = boost::lexical_cast<double>(items[2]);
        double heading = boost::lexical_cast<double>(items[3]);
        double pitch = boost::lexical_cast<double>(items[4]);
        double roll = boost::lexical_cast<double>(items[5]);

        double lattitude = boost::lexical_cast<double>(items[6]);
        double longitude = boost::lexical_cast<double>(items[7]);
        double altitude = boost::lexical_cast<double>(items[8]);

        double vx = boost::lexical_cast<double >(items[9]);
        double vy = boost::lexical_cast<double >(items[10]);
        double vz = boost::lexical_cast<double >(items[11]);

        cv::Point3d gauss_point;
        coord_cvt_ptr_->ProjSinglePoint(cv::Point3d(longitude, lattitude, altitude), gauss_point);

        GPSData data;
        data.time = time;
        data.heading = heading;
        data.pitch = pitch;
        data.lattitude = lattitude;
        data.longitude = longitude;
        data.altitude = altitude;
        data.x = gauss_point.x;
        data.y = gauss_point.y;
        data.vx = vx;
        data.vy = vy;

        gt_data_vec_.push_back(data);

        gt_trace_.addPoint(gauss_point.x,gauss_point.y);

        memset(buf,0,2048);
    }

    file_in.close();

}

void GISAnalyze::InitPredictionTrace(const std::vector<cv::Vec4d>& trace,const std::vector<int>& image_ids) {
    for(int i=0;i<trace.size();i++){
        double time = trace[i][0];
        double x = trace[i][1];
        double y = trace[i][2];
        double z = trace[i][3];

        int image_id = image_ids[i];
        cv::Point3d geo_point;
        //x+= 38000000;
        coord_cvt_ptr_->InverseProjSinglePoint(cv::Point3d(x + 38000000,y,z),geo_point);

        PredictionData data;
        data.x = x;
        data.y = y;
        data.z = z;
        data.image_id = image_id;
        data.time = time;

        data.longitude = geo_point.x;
        data.lattitude = geo_point.y;

        prediction_data_vec_.push_back(data);

    }


}

void GISAnalyze::SaveAnalyzeReport(const std::string base_path)
{
    std::ofstream file_heading(base_path + "heading_compare.txt");
    std::ofstream file_pos(base_path + "position_compare.txt");
    std::ofstream file_speed(base_path + "velocity_compare.txt");
    std::ofstream file_distance(base_path+"distance.txt");




    for(auto it : prediction_data_vec_)
    {
        if(fabs(it.vx) > 10 || fabs(it.vy) > 10 ){
            continue;
        }


        double time = it.time;

        int index = (time - gt_data_vec_[0].time)/0.05;

        file_heading<<std::setprecision(10)<<gt_data_vec_[index].time << ","<<
            gt_data_vec_[index].heading<<","<<
            it.image_id<<","<<
            std::setprecision(10)<<time<<","<<
            it.heading<<endl;

        file_pos<<std::setprecision(10)<<gt_data_vec_[index].time << ","<<
            std::setprecision(16)<<gt_data_vec_[index].x<<","<<
            std::setprecision(16)<<gt_data_vec_[index].y<<","<<
            it.image_id<<","<<
            std::setprecision(10)<<time<<","<<
            std::setprecision(16)<<it.x<<","<<
            std::setprecision(16)<<it.y<<endl;

        file_speed<<std::setprecision(10)<<gt_data_vec_[index].time<<","<<
            gt_data_vec_[index].vx<<","<<
            gt_data_vec_[index].vy<<","<<
            it.image_id<<","<<
            std::setprecision(10)<<time<<","<<
            it.vx<<","<<
            it.vy<<endl;

        file_distance<<time<<","<<it.image_id<<","<<it.distance<<endl;

    }

    file_heading.close();
    file_pos.close();
    file_speed.close();


}


void GISAnalyze::CalcHeading(int step)
{
    for(int i = 0;i<prediction_data_vec_.size()-step;i++){
        PredictionData data_pre = prediction_data_vec_[i];
        PredictionData data_nxt = prediction_data_vec_[i+step];

        double heading = CalcAzimuth(data_pre.longitude,data_pre.lattitude,data_nxt.longitude,data_nxt.lattitude);

        prediction_data_vec_[i].heading =heading;
    }
}

void GISAnalyze::CalcPoints2TraceDis()
{
    for(auto& it : prediction_data_vec_){
        double x = it.x;
        double y = it.y;

        OGRPoint pt(x,y);
        double dis = gt_trace_.Distance(&pt);

        it.distance = dis;
    }
}

void GISAnalyze::CalcVelocityByTrace(int num ,int ndim )
{
    double shift_t = prediction_data_vec_[0].time;
    double shift_x = prediction_data_vec_[0].x;
    double shift_y = prediction_data_vec_[0].y;

    vector<cv::Point2d> tx_points;
    vector<cv::Point2d> ty_points;


    //int n = 5;      // 定義多項式次數
    //int num = 6;    // 測試發現6點擬合效果交好

    for(int i = num/2;i<prediction_data_vec_.size()-num/2;i++){
        tx_points.clear();
        ty_points.clear();
        double target_time = prediction_data_vec_[i].time;
        for(int j=-num/2;j<num/2;j++){
            double t = prediction_data_vec_[i+j].time;
            double x = prediction_data_vec_[i+j].x;
            double y = prediction_data_vec_[i+j].y;

            tx_points.push_back(cv::Point2d(t-shift_t,x-shift_x));
            ty_points.push_back(cv::Point2d(t-shift_t,y-shift_y));

        }
        // 多項式曲線擬合
        //int n = 5;      // 定義多項式次
        cv::Mat K_x = PolyFit(tx_points,ndim);
        cv::Mat K_y = PolyFit(ty_points,ndim);

        double target_vx = 0.0;
        double target_vy = 0.0;

        for(int j=0;j<ndim+1;j++){
            target_vx += j * K_x.at<double>(j,0) * pow(target_time-shift_t,j-1);
            target_vy += j * K_y.at<double>(j,0) * pow(target_time-shift_t,j-1);
        }

        prediction_data_vec_[i].vx = target_vx;
        prediction_data_vec_[i].vy = target_vy;
        //vx_vec.push_back(cv::Vec2d(target_time,target_vx));
        //vy_vec.push_back(cv::Vec2d(target_time,target_vy));

    }


}

double GISAnalyze::CalcAzimuth(double lon_a, double lat_a, double lon_b, double lat_b)
{
    double cosC = cos((90 - lat_b) * M_PI / 180) * cos((90 - lat_a) * M_PI / 180)
                  + sin((90 - lat_b) * M_PI / 180) * sin((90 - lat_a) * M_PI / 180) * cos((lon_b - lon_a) * M_PI / 180);
    double sinC = sqrt(1 - pow(cosC, 2));
    double sinA = sin((90 - lat_b) * M_PI / 180) * sin((lon_b - lon_a) * M_PI / 180) / sinC;

    double azimuth = 0.0;
    if(sinA >=1 ){
        azimuth = 90.0;
    }else{
        azimuth = asin(sinA) * 180 / M_PI;
    }


    // 以A爲原點，B在第一像限
    if(lon_b > lon_a && lat_b > lat_a){
        return azimuth;
    }else if(lon_b < lon_a && lat_b > lat_a){  // 第2像限
        return 360+azimuth;
    }else{
        return 180-azimuth;
    }

}

// https://blog.csdn.net/i_chaoren/article/details/79822574
cv::Mat GISAnalyze::PolyFit(vector<cv::Point2d> &points, int n) {

    int size = points.size();


    int x_num = n+1;

    cv::Mat U(size,x_num,CV_64F);

    cv::Mat Y(size,1,CV_64F);

    for(int i=0;i<U.rows;i++){
        for(int j=0;j<U.cols;j++){
            U.at<double>(i,j) = pow(points[i].x,j);
        }
    }

    for(int i=0;i<Y.rows;i++){
        Y.at<double>(i,0) = points[i].y;
    }

    cv::Mat K(x_num,1,CV_64F);
    K = (U.t()*U).inv() * U.t()*Y;

    return K;

}

//////////////////////////////////////////////////////////////////////////////////////////////////



// 每10個點擬合一條 t-x曲線,求一節導數得到速度
void GISAnalyze::CalcVelocityByTrace(const std::vector<cv::Vec4d>& trace,std::vector<cv::Vec2d>& vx_vec,std::vector<cv::Vec2d>& vy_vec)
{
    double shift_t = trace[0][0];
    double shift_x = trace[0][1];
    double shift_y = trace[0][2];

    vector<cv::Point2d> tx_points;
    vector<cv::Point2d> ty_points;


    int n = 5;      // 定義多項式次數
    int num = 6;    // 測試發現6點擬合效果交好

    for(int i = num/2;i<trace.size()-num/2;i++){
        tx_points.clear();
        ty_points.clear();
        double target_time = trace[i][0];
        for(int j=-num/2;j<num/2;j++){
            double t = trace[i+j][0];
            double x = trace[i+j][1];
            double y = trace[i+j][2];

            tx_points.push_back(cv::Point2d(t-shift_t,x-shift_x));
            ty_points.push_back(cv::Point2d(t-shift_t,y-shift_y));

        }
        // 多項式曲線擬合
        //int n = 5;      // 定義多項式次
        cv::Mat K_x = PolyFit(tx_points,n);
        cv::Mat K_y = PolyFit(ty_points,n);

        double target_vx = 0.0;
        double target_vy = 0.0;

        for(int j=0;j<n+1;j++){
            target_vx += j * K_x.at<double>(j,0) * pow(target_time-shift_t,j-1);
            target_vy += j * K_y.at<double>(j,0) * pow(target_time-shift_t,j-1);
        }

        vx_vec.push_back(cv::Vec2d(target_time,target_vx));
        vy_vec.push_back(cv::Vec2d(target_time,target_vy));

    }



}
