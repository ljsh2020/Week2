#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "CameraApi.h" //相机SDK的API头文件
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

using namespace cv;

unsigned char *g_pRgbBuffer; //处理后数据缓存区

//putText()
//滑动条控制图像移动
//滑动条位置->rect()->img(rect)
//mouse callback
//left->stop,drawn
//right->save

//标定板规格
const int board_w = 11, board_h = 8;
const int board_n = board_w * board_h;
Size board_size(11, 8);

//生成世界坐标
Size square_size(10, 10);
std::vector<std::vector<Point3f>> obj_points;
std::vector<Point3f> obj_points_buf;
std::vector<int> point_count;

Mat camera_matrix(3, 3, CV_32FC1, Scalar::all(0));
Mat dist_coeffs(1, 5, CV_32FC1, Scalar::all(0));
std::vector<Mat> rvecs;
std::vector<Mat> tvecs;

//像坐标
std::vector<Point2f> img_points_buf;
std::vector<std::vector<Point2f>> img_points;

//图片大小
Size img_size;

Mat frame;

bool haveSize = 0;
int success = 0;
void callback(int event, int x, int y, int flags, void *param);

int main()
{
    for (int j = 0; j < board_h; j++)
    {
        for (int k = 0; k < board_w; k++)
        {
            Point3f pt;
            pt.x = k * square_size.width;
            pt.y = j * square_size.height;
            pt.z = 0;
            obj_points_buf.push_back(pt);
        }
    }

    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; //设备描述信息
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    int channel = 3;

    CameraSdkInit(1);
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    if (iCameraCounts == 0)
    {
        return -1;
    }
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }
    CameraGetCapability(hCamera, &tCapability);
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    CameraPlay(hCamera);
    CameraSetExposureTime(hCamera, 100);
    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    while (true)
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            Mat img( cv::Size(  sFrameInfo.iWidth, 
                                sFrameInfo.iHeight), 
                                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3, 
                                g_pRgbBuffer);
            frame = img.clone();
            imshow("camera", frame);
            setMouseCallback("camera", callback, 0);
            if (waitKey(5) == 27)
                break;
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }
    CameraUnInit(hCamera);
    free(g_pRgbBuffer);

    std::cout << calibrateCamera(obj_points, img_points, img_size, camera_matrix, dist_coeffs, rvecs, tvecs) << std::endl;
    std::cout << camera_matrix << std::endl
              << dist_coeffs << std::endl;

    return 0;
}

void callback(int event, int x, int y, int flags, void *param)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        bool isFound = 0;
        isFound = findChessboardCorners(frame, board_size, img_points_buf);
        std::cout<< img_points_buf.size()<< std::endl;

        if (isFound)
        {
            success++;
            //get img_size
            if (!haveSize)
            {
                img_size.width = frame.cols;
                img_size.height = frame.rows;
                haveSize = 1;
            }
            imwrite(std::__cxx11::to_string(success)+"_orig.jpg",frame);
            Mat gray_img;
            cvtColor(frame, gray_img, COLOR_BGR2GRAY);
            find4QuadCornerSubpix(gray_img, img_points_buf, Size(5, 5));
            drawChessboardCorners(frame, board_size, img_points_buf, isFound);
            imshow("camera", frame);
            imwrite(std::__cxx11::to_string(success)+"_dst.jpg",frame);
            img_points.push_back(img_points_buf);
            img_points_buf.clear();

            obj_points.push_back(obj_points_buf);
            point_count.push_back(board_n);
            std::cout << point_count.size() <<std::endl<<calibrateCamera(obj_points, img_points, img_size, camera_matrix, dist_coeffs, rvecs, tvecs) << std::endl;
            std::cout << camera_matrix << std::endl<< dist_coeffs << std::endl;
            waitKey(500);
        }
        else
        {
            std::cout << "Fail to find all points" << std::endl;
        }
    }
    
}