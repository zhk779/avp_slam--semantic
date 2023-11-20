#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include <Eigen/Dense>
#include <vector>

#include <string>

typedef cv::Point3_<uint8_t> Pixel;
typedef pcl::PointXYZRGB PointType;
pcl::PointCloud<PointType>::Ptr cameraCloud(new pcl::PointCloud<PointType>());

const int imageRowIncrease=1;
const int imageColIncrease=1;
const double cameraRealiableDis=15.0;
const std::set<std::string> validColor={"113,193,46"};

std::vector<double> KDouble={2.4334967658138854e+02, 0., 4.7841060926592388e+02, 0., 2.4296188015508932e+02, 3.2073719693989187e+02, 0., 0., 1. };
std::vector<double> R0Double={0, 1.0, 0, 0.0, 0, 1, 1.0, 0.0, 0.0};
std::vector<double> T0Double={0, 0.56, 0.0};

Eigen::Matrix3d K = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(KDouble.data(), 3, 3);
Eigen::Matrix3d R0 = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R0Double.data(), 3, 3);
Eigen::Vector3d T0 = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(T0Double.data(), 3, 1);


void calCloudFromImage(Eigen::Matrix3d& K, Eigen::Matrix3d& RT,const cv::Mat& image,pcl::PointCloud<PointType>::Ptr& cameraCloud){
    Eigen::Matrix3d KInv=K.inverse();
    Eigen::Matrix3d RTInv=RT.inverse();
    int row=image.rows;
    int col=image.cols;
    cv::Mat img_tmp = image.clone();
    // cv::imshow("分割图", image);
    // cv::imshow("克隆图", img_tmp);
    // cv::waitKey(0);

    for(int i=0;i<row;i=i+imageRowIncrease){
        for(int j=0;j<col;j=j+imageColIncrease){

            Pixel* pixel = img_tmp.ptr<Pixel>(i,j);
            int b = pixel->z;
            int g = pixel->y;
            int r = pixel->x;
            //   there,for simplicity,just according to color,detect invalid area like sky area;
            //   for real scene,should adapt machine learning method or other method detecting invalid area
            std::stringstream color_string;
            color_string << b << "," << g << "," << r;
            // if (b != 0)
            //     std::cout << color_string.str() << std::endl;

            if (validColor.count(color_string.str()) == 0)
            {
                continue; // filt invalid collor
            }

            // Eigen::Vector3d u;
            // u(0) = j;
            // u(1) = i;
            // u(2) = 1;
            // Eigen::Vector3d u1;

            // u1 = KInv * u;
            // u1 = RTInv * u1;
            // u1(0) = u1.x() / u1.z();
            // u1(1) = u1.y() / u1.z();
            // double dis = sqrt(u1.x() * u1.x() + u1.y() * u1.y());

            // if (dis > cameraRealiableDis)
            //     continue;

            PointType po;
            po.x = j;
            po.y = i;
            po.z = 0;
            po.r = r;
            po.g = g;
            po.b = b;
            cameraCloud->push_back(po);
        }
    }
}

void cameraCloudHandler(cv::Mat &img)
{
    calCloudFromImage(K, R0, img, cameraCloud);
}

int main(int argc, char *argv[])
{
    char imageName[] = "/home/snow/Documents/workspace/slam_ws/src/segmodel_pkg/scripts/output/test_results/1697629184.257601.png";
    cv::Mat img = cv::imread(imageName, cv::IMREAD_COLOR);

    char tmp[256];
    getcwd(tmp, 256);

    if(img.empty())     // 判断文件是否正常打开  
    {
         fprintf(stderr, "Can not load image %s\n", imageName);
         fprintf(stderr, "Current path is %s\n", tmp);
        //  waitKey(6000);  // 等待6000 ms后窗口自动关闭   
         return -1;
    }

    cameraCloudHandler(img);

    pcl::visualization::PCLVisualizer viewer; 
    viewer.addPointCloud<PointType> (cameraCloud, "sample cloud");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}

