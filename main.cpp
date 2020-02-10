#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "GISAnalyze.h"
#include "GPSData.h"

using namespace std;
using namespace cv;


void ReadTrace(const char* file_path,std::vector<cv::Vec4d>& trace,std::vector<int>& image_id);

int BinarySearch(double t,const std::vector<cv::Vec4d>& trace,int left,int right);

void MergeSameTarget(std::vector<cv::Vec4d>& trace,std::vector<int>& image_ids);


int main()
{
    std::vector<cv::Vec4d> trace;

    std::vector<int> image_ids;

    ReadTrace("/media/ibd02/KINGSTON/data2/1578127581663/lidar2world_lidar.txt", trace, image_ids);

    MergeSameTarget(trace,image_ids);

    GISAnalyze gisHelper;

    gisHelper.InitPredictionTrace(trace,image_ids);

    gisHelper.InitGroudTruth("/home/ibd02/Project/yunzhou/TraceAnalysis/cmake-build-debug/02.txt");

    // 1.計算航向
    gisHelper.CalcHeading(3);


    // 2.計算速度
    gisHelper.CalcVelocityByTrace();

    // 3.計算絕對位置
    gisHelper.CalcPoints2TraceDis();





    gisHelper.SaveAnalyzeReport("/media/ibd02/KINGSTON/data2/1578127581663/report/");




    return 0;

}

void ReadTrace(const char* file_path,std::vector<cv::Vec4d>& trace,std::vector<int>& image_id)
{
    ifstream file_in(file_path);

    char buf[1024]={0};

    while(file_in.getline(buf,1024)){
        string strbuf = buf;
        vector<string> items;
        boost::split(items,strbuf,boost::is_any_of(" "));
        if(items.size() != 6)
            continue;

        int id = boost::lexical_cast<int>(items[0]);
        double time = boost::lexical_cast<double>(items[2]);
        double x = boost::lexical_cast<double>(items[3]);
        double y = boost::lexical_cast<double>(items[4]);
        double z = boost::lexical_cast<double>(items[5]);

        trace.push_back(cv::Vec4d(time,x,y,z));
        image_id.push_back(id);

    }
    file_in.close();
}


int BinarySearch(double t,const std::vector<cv::Vec4d>& trace,int left,int right)
{
    int media = (left + right) /2;

    if(left == media){

        if(t > trace[left][0] && t < trace[left+1][0]){
            return left;
        }else{
            return -1;
        }
    }

    if(t >= trace[media][0] && t < trace[media+1][0]){
        return media;
    }else if(t > trace[media][0] && t <= trace[media+1][0]){
        return media+1;
    }else if(t < trace[media][0] ){
        return BinarySearch(t,trace,left,media);
    }else if(t > trace[media+1][0]){
        return BinarySearch(t,trace,media,right);
    }else{
        return -1;
    }

}

void MergeSameTarget(std::vector<cv::Vec4d>& trace,std::vector<int>& image_ids)
{
    std::vector<cv::Vec4d> trace_tmp;
    std::vector<int> image_ids_tmp;

    for(int i=0;i<image_ids.size()-1;){
        // 說明左右圖像中都檢測到了目標
        if(image_ids[i] == image_ids[i+1]){
            double x = (trace[i][1] + trace[i+1][1]) * 0.5;
            double y = (trace[i][2] + trace[i+1][2]) * 0.5;
            double z = (trace[i][3] + trace[i+1][3]) * 0.5;
            double time = (trace[i][0] + trace[i+1][0]) * 0.5;

            trace_tmp.push_back(cv::Vec4d(time,x,y,z));
            image_ids_tmp.push_back(image_ids[i]);
            i+=2;

        }else{
            trace_tmp.push_back(trace[i]);
            image_ids_tmp.push_back(image_ids[i]);
            i++;
        }
    }

    trace.swap(trace_tmp);
    image_ids.swap(image_ids_tmp);

}
