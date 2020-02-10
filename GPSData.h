//
// Created by ibd02 on 20-2-8.
//

#ifndef TRACEANALYSIS_GPSDATA_H
#define TRACEANALYSIS_GPSDATA_H


class GPSData {
public:

    double time;
    double heading;
    double pitch;
    double roll;
    double lattitude;   // 緯度 34.196
    double longitude;   // 經度 108.86
    double altitude;
    double x;           // 投影坐標
    double y;
    double vx;
    double vy;
    double vz;


};


class PredictionData
{
public:
    int image_id = -1;           // 對應的圖像id
    double time = 0.0;
    double heading = 0.0;         // 感知目標方位 （heading 真北方向夾角）
    double lattitude = 0.0;       // 緯度
    double longitude =  0.0;       // 經度
    double x = 0.0;               // 投影坐標
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;              // x方向速度
    double vy = 0.0;
    double yz = 0.0;

    double distance = 0.0;      // 到真實軌跡的偏差

};


#endif //TRACEANALYSIS_GPSDATA_H
