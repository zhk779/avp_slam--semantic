#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>

#include "sourrond/common.h"

#define DEBUG
#define AWB_LUN_BANLANCE_ENALE    0

char ipm_topic[] = "/ipm";
std::mutex mBuf;

std::queue<sensor_msgs::ImageConstPtr> cameraFrontBuf;
std::queue<sensor_msgs::ImageConstPtr> cameraLeftBuf;
std::queue<sensor_msgs::ImageConstPtr> cameraRightBuf;
std::queue<sensor_msgs::ImageConstPtr> cameraBackBuf;

double timeCameraFrontImage=0;
double timeCameraLeftImage=0;
double timeCameraRightImage=0;
double timeCameraBackImage=0;

cv_bridge::CvImageConstPtr front_ptr;
cv_bridge::CvImageConstPtr left_ptr;
cv_bridge::CvImageConstPtr right_ptr;
cv_bridge::CvImageConstPtr back_ptr;


void cameraFrontHandler(const sensor_msgs::ImageConstPtr &imageMsg){
    if(cameraFrontBuf.size() > 2){
        return;
    }
    mBuf.lock();
    cameraFrontBuf.push(imageMsg);
    mBuf.unlock();
}

void cameraLeftHandler(const sensor_msgs::ImageConstPtr &imageMsg){
    if(cameraLeftBuf.size() > 2){
        return;
    }
    mBuf.lock();
    cameraLeftBuf.push(imageMsg);
    mBuf.unlock();
}

void cameraRightHandler(const sensor_msgs::ImageConstPtr &imageMsg){
    if(cameraRightBuf.size() > 2){
        return;
    }
    mBuf.lock();
    cameraRightBuf.push(imageMsg);
    mBuf.unlock();
}

void cameraBackHandler(const sensor_msgs::ImageConstPtr &imageMsg){
    if(cameraBackBuf.size() > 2){
        return;
    }
    mBuf.lock();
    cameraBackBuf.push(imageMsg);
    mBuf.unlock();
}


//   synchronism images of all camera
void removeUnsynData(double& timeMax,double& timeCameraImage,std::queue<sensor_msgs::ImageConstPtr>& cameraImageBuf){
     while((timeCameraImage<timeMax) && !cameraImageBuf.empty()){
                 mBuf.lock();
                 cameraImageBuf.pop();
                 mBuf.unlock();
                 if(!cameraImageBuf.empty())
                     timeCameraImage=cameraImageBuf.front()->header.stamp.toSec();
            }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ipm_node");
    ros::NodeHandle nh;

    ROS_INFO("start to generate ipm image in the topic %s", ipm_topic);
    cv::Mat car_img;
    cv::Mat origin_dir_img[4];
    cv::Mat undist_dir_img[4];
    cv::Mat merge_weights_img[4];
    cv::Mat out_put_img;
    float *w_ptr[4];
    CameraPrms prms[4];

    //1.load some predefine parameters and mask images
    //  load car img to cover the area that camera can not see (body of car)
    // car_img = cv::imread("/home/snow/Documents/workspace/slam_ws/src/ipm_pkg/images/car.png");
    cv::Mat car_img_tmp = cv::Mat(cv::Size(xr - xl, yb - yt), CV_8UC3, cv::Scalar(0, 0, 0));

    // load weight for intersection region
    cv::Mat weights = cv::imread("/home/snow/Documents/workspace/slam_ws/src/ipm_pkg/yaml/weights.png", -1);
    if (weights.channels() != 4) {
        std::cerr << "imread weights failed " << weights.channels() << "\r\n";
        return -1;
    }

    for (int i = 0; i < 4; ++i) {
        merge_weights_img[i] = cv::Mat(weights.size(), CV_32FC1, cv::Scalar(0, 0, 0));
        w_ptr[i] = (float *)merge_weights_img[i].data;
    }
    //read weights of corner
    int pixel_index = 0;
    for (int h = 0; h < weights.rows; ++h) {
        uchar* uc_pixel = weights.data + h * weights.step;
        for (int w = 0; w < weights.cols; ++w) {
            w_ptr[0][pixel_index] = uc_pixel[0] / 255.0f;
            w_ptr[1][pixel_index] = uc_pixel[1] / 255.0f;
            w_ptr[2][pixel_index] = uc_pixel[2] / 255.0f;
            w_ptr[3][pixel_index] = uc_pixel[3] / 255.0f;
            uc_pixel += 4;
            ++pixel_index;
        }
    }

// #ifdef DEBUG
//     for (int i = 0; i < 4; ++i) {
//         //0 左下 1 右上 2 左上 3 左下
//         display_mat(merge_weights_img[i], "w");
//     }
// #endif

    // read calibration prms
    for (int i = 0; i < 4; ++i) {
        auto& prm = prms[i];
        prm.name = camera_names[i];
        auto ok = read_prms("/home/snow/Documents/workspace/slam_ws/src/ipm_pkg/yaml/" + prm.name + ".yaml", prm);
        if (!ok) {
            return -1;
        }
    }

    //2.subscribe every camera (front back left right), put images in buffers 
    ros::Subscriber subCamera0Image = nh.subscribe("/camera_front/image_raw", 2, cameraFrontHandler);
    ros::Subscriber subCamera1Image = nh.subscribe("/camera_left/image_raw", 2, cameraLeftHandler);
    ros::Subscriber subCamera2Image = nh.subscribe("/camera_right/image_raw", 2, cameraRightHandler);
    ros::Subscriber subCamera3Image = nh.subscribe("/camera_back/image_raw", 2, cameraBackHandler);

    //3.loop to generate ipm images
    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        //  synchronism images of multi cameras according to timestamp
        // cameraFrontBuf cameraLeftBuf cameraRightBuf cameraBackBuf
        // ROS_WARN("Here *****************!!!!!!!!!!!!");
        // init out_put_image
        out_put_img = cv::Mat(cv::Size(total_w, total_h), CV_8UC3, cv::Scalar(0, 0, 0));
        // cv::Mat car_img_tmp = cv::Mat(cv::Size(xr - xl, yb - yt), CV_8UC3, cv::Scalar(0, 0, 0));
        // cv::resize(car_img, car_img_tmp, cv::Size(xr - xl, yb - yt));

        if(!cameraFrontBuf.empty() && !cameraBackBuf.empty() && !cameraLeftBuf.empty() && !cameraRightBuf.empty())
        {
            timeCameraFrontImage=cameraFrontBuf.front()->header.stamp.toSec();
            timeCameraLeftImage=cameraLeftBuf.front()->header.stamp.toSec();
            timeCameraRightImage=cameraRightBuf.front()->header.stamp.toSec();
            timeCameraBackImage=cameraBackBuf.front()->header.stamp.toSec();

            if((timeCameraFrontImage!=timeCameraLeftImage) || (timeCameraFrontImage!=timeCameraRightImage) || (timeCameraFrontImage!=timeCameraBackImage))
            {
                ROS_WARN("IPM_Node: image is not syn");

                double timeImageMax=timeCameraFrontImage;
                timeImageMax=std::max(timeImageMax,timeCameraLeftImage);
                timeImageMax=std::max(timeImageMax,timeCameraRightImage);
                timeImageMax=std::max(timeImageMax,timeCameraBackImage);

                removeUnsynData(timeImageMax,timeCameraFrontImage,cameraFrontBuf);
                removeUnsynData(timeImageMax,timeCameraLeftImage,cameraLeftBuf);
                removeUnsynData(timeImageMax,timeCameraRightImage,cameraRightBuf);
                removeUnsynData(timeImageMax,timeCameraBackImage,cameraBackBuf);

                continue;

            }

            // ROS_INFO("IPM_Node: image is syned");
            std::string encoding = "bgr8";
            mBuf.lock();
            // ROS_WARN("bufer before size is %ld", cameraFrontBuf.size());
            front_ptr = cv_bridge::toCvShare(cameraFrontBuf.front(), encoding);
            cameraFrontBuf.pop();
            // ROS_WARN("bufer after size is %ld", cameraFrontBuf.size());
            mBuf.unlock();

            mBuf.lock();
            left_ptr = cv_bridge::toCvShare(cameraLeftBuf.front(), encoding);
            cameraLeftBuf.pop();
            mBuf.unlock();

             mBuf.lock();
            right_ptr = cv_bridge::toCvShare(cameraRightBuf.front(), encoding);
            cameraRightBuf.pop();
            mBuf.unlock();

            mBuf.lock();
            back_ptr = cv_bridge::toCvShare(cameraBackBuf.front(), encoding);
            cameraBackBuf.pop();
            mBuf.unlock();

            std::vector<cv::Mat*> srcs;
            origin_dir_img[0] = front_ptr->image;
            srcs.push_back(&origin_dir_img[0]);
            origin_dir_img[1] = left_ptr->image;
            srcs.push_back(&origin_dir_img[1]);
            origin_dir_img[3] = right_ptr->image;
            srcs.push_back(&origin_dir_img[3]);
            origin_dir_img[2] = back_ptr->image;
            srcs.push_back(&origin_dir_img[2]);
            
            //4.lum equalization and awb for four channel image
        #if AWB_LUN_BANLANCE_ENALE
            awb_and_lum_banlance(srcs);
        #endif

            //3. undistort image
            for (int i = 0; i < 4; ++i) {
                auto& prm = prms[i];
                cv::Mat& src = origin_dir_img[i];
            
                undist_by_remap(src, src, prm);
                cv::warpPerspective(src, src, prm.project_matrix, project_shapes[prm.name]);
                
                if (camera_flip_mir[i] == "r+") {
                    cv::rotate(src, src, cv::ROTATE_90_CLOCKWISE);
                } else if (camera_flip_mir[i] == "r-") {
                    cv::rotate(src, src, cv::ROTATE_90_COUNTERCLOCKWISE);
                } else if (camera_flip_mir[i] == "m") {
                    cv::rotate(src, src, cv::ROTATE_180);
                }
                //display_mat(src, "project");
                //cv::imwrite(prms.name + "_undist.png", src);
                undist_dir_img[i] = src.clone();
            }

            //4.start combine
            std::cout  << argv[0] << " app start combine" << std::endl;
            car_img_tmp.copyTo(out_put_img(cv::Rect(xl, yt, car_img_tmp.cols, car_img_tmp.rows)));
            //4.1 out_put_img center copy 
            for (int i = 0; i < 4; ++i) {
                cv::Rect roi;
                bool is_cali_roi = false;
                if (std::string(camera_names[i]) == "front") {
                    roi = cv::Rect(xl, 0, xr - xl, yt);
                    //std::cout << "\nfront" << roi;
                    undist_dir_img[i](roi).copyTo(out_put_img(roi));
                } else if (std::string(camera_names[i]) == "left") {
                    roi = cv::Rect(0, yt, xl, yb - yt);
                    //std::cout << "\nleft" << roi << out_put_img.size();
                    undist_dir_img[i](roi).copyTo(out_put_img(roi));
                } else if (std::string(camera_names[i]) == "right") {
                    roi = cv::Rect(0, yt, xl, yb - yt);
                    //std::cout << "\nright" << roi << out_put_img.size();
                    undist_dir_img[i](roi).copyTo(out_put_img(cv::Rect(xr, yt, total_w - xr, yb - yt)));
                } else if (std::string(camera_names[i]) == "back") {
                    roi = cv::Rect(xl, 0, xr - xl, yt);
                    //std::cout << "\nright" << roi << out_put_img.size();
                    undist_dir_img[i](roi).copyTo(out_put_img(cv::Rect(xl, yb, xr - xl, yt)));
                } 
            }
            //4.2the four corner merge
            //w: 0 左下 1 右上 2 左上 3 左下
            //image: "front", "left", "back", "right"
            cv::Rect roi;
            //左上
            roi = cv::Rect(0, 0, xl, yt);
            cv::Mat src1 = undist_dir_img[0](roi);
            cv::Mat src2 = undist_dir_img[1](roi);
            cv::Mat out = out_put_img(roi);
            merge_image(src1, src2, merge_weights_img[2], out);
            //右上
            roi = cv::Rect(xr, 0, xl, yt);
            src1 = undist_dir_img[0](roi);
            src2 = undist_dir_img[3](cv::Rect(0, 0, xl, yt));
            out = out_put_img(cv::Rect(xr, 0, xl, yt));
            merge_image(src1, src2, merge_weights_img[1], out);
            //左下
            roi = cv::Rect(0, yb, xl, yt);
            src1 = undist_dir_img[2](cv::Rect(0, 0, xl, yt));
            src2 = undist_dir_img[1](roi);
            out = out_put_img(roi);
            merge_image(src1, src2, merge_weights_img[0], out);
            //右下
            roi = cv::Rect(xr, 0, xl, yt);
            src1 = undist_dir_img[2](roi);
            src2 = undist_dir_img[3](cv::Rect(0, yb, xl, yt));
            out = out_put_img(cv::Rect(xr, yb, xl, yt));
            merge_image(src1, src2, merge_weights_img[3], out);

            cv::imwrite("/home/snow/Documents/workspace/slam_ws/src/imgpub_pkg/img/ADAS_EYES_360_VIEW.png", out_put_img);
            
        #ifdef DEBUG   
            cv::resize(out_put_img, out_put_img, cv::Size(out_put_img.size()/2)),
            display_mat(out_put_img, "out_put_img");
        #endif

            return 0;

            // std::vector<cv::Mat*>().swap(srcs);
        }

        // car_img_tmp.release();
        // out_put_img.release();
        rate.sleep();
        

    }

    car_img.release();
    std::cout  << argv[0] << " app finished" << std::endl;  
    return 0;
}