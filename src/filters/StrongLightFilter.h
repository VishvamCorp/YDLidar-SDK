#ifndef STRONGLIGHTFILTER_H
#define STRONGLIGHTFILTER_H
#include "FilterInterface.h"


//Strong light filter (tail filter)
class YDLIDAR_API StrongLightFilter : public FilterInterface
{
public:
    enum FilterStrategy //Strategies
    {
        FS_1, //Method 1: angle-distance approach
        FS_2, //Method 2: intercept approach
    };
public:
    StrongLightFilter();
    virtual ~StrongLightFilter();
    
    virtual void filter(const LaserScan &in,
                int lidarType,
                int version,
                LaserScan &out);
    void setMaxDist(float dist) {maxDist = dist;}
    void setMaxAngle(float angle) {maxAngle = angle;}
    void setMinNoise(int noise) {minNoise = noise;}
    void setStrategy(int s) {m_strategy = s;}

protected:
    bool filter1(const LaserScan &in,
                LaserScan &out);
    bool filter2(const LaserScan &in,
                LaserScan &out);

protected:
    struct Point
    {
        float x = .0;
        float y = .0;

        Point(float x = .0, float y = .0);

        static Point angular2Polar(const Point &p); //Convert Cartesian coordinates to polar
        static Point polar2Angular(const Point &p); //Convert polar coordinates to Cartesian
        //Compute the distance from a point to a line in Cartesian space
        static float calcDist(
            const Point &p,
            const Point &p1,
            const Point &p2);
        //Compute vector length
        static float calcLen(
            const Point &v);
        //Compute dot product of two vectors
        static float calcDot(
            const Point &v1,
            const Point &v2);
        //Compute the angle between two line segments in Cartesian space
        static float calcAngle(
            const Point &p1,
            const Point &p2,
            const Point &p3,
            const Point &p4);
    };

    //Filtering strategy
    int m_strategy = FS_2;
    float maxDist = 0.05; //Maximum distance threshold in meters (adjust as needed)
    float maxAngle = 12.0; //Maximum angle threshold in degrees (adjust as needed)
    int minNoise = 2; //Minimum count of consecutive noise points (adjust as needed)
};

#endif // STRONGLIGHTFILTER_H
