#include <iostream>
#include <opencv2/opencv.hpp>
#include<Eigen/dense>
using namespace cv;
using namespace std;
using namespace Eigen;

int main(void)
{
    Eigen::Matrix3f TBN;
    Eigen::Vector3f a,b,c;
    a<<1,0,0;
    b<<0,1,0;
    c<<0,0,1;
    TBN<<a,b,c;
    cout<<TBN;

    return 0;
}