#ifndef undistorted_top_view_PARAMETERS
#define undistorted_top_view_PARAMETERS

#include <string>

struct Parameters
{
    std::string cameraType;
    std::string inTopic;
    std::string outTopic;
    std::string calibrationFile;
    std::string transportHint;
    double scaleFactor;
    int rightBound;
    int leftBound;
    int bottomBound;
    int topBound;
};

#endif

