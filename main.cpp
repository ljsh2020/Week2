#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

int main(){

    //标定板规格
    const int board_w = 9, board_h = 6; 
    const int board_n = board_w * board_h; 
    Size board_size( 9,6 );


    //像坐标
    std::vector< Point2f > img_points_buf;
    std::vector< std::vector< Point2f > > img_points;

    //图片大小
    Size img_size;

    bool isFound = 1;
    int success = 0;
    bool haveSize = 0;

    for (int i = 0; i < 24; i++){
        Mat src = imread("/home/nvidia/Documents/week2/calibrate/"+std::__cxx11::to_string(i)+"_orig.jpg");
        Mat gray_img;
        cvtColor( src, gray_img, COLOR_BGR2GRAY );
        adaptiveThreshold( gray_img,gray_img, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 51, 0 );
        isFound = findChessboardCorners( gray_img, board_size, img_points_buf );
        if (isFound){
            if (!haveSize) {
               img_size.width = src.cols; 
               img_size.height = src.rows;
               haveSize = 1;
            }
            find4QuadCornerSubpix(gray_img, img_points_buf, Size(5,5));
            Mat drawn_img = src.clone();
            drawChessboardCorners( drawn_img, board_size, img_points_buf ,isFound);
            imshow("result", drawn_img);
            waitKey(5);
            img_points.push_back(img_points_buf);
            img_points_buf.clear();

            success++;
        }
        else{
            std::cout << i << "Failed" << std::endl;
        }
        
    }
    std::cout << success << "succeed" << std::endl;

    
    //生成世界坐标
    Size square_size(10,10);
    std::vector< std::vector< Point3f > > obj_points; 
    std::vector< Point3f > obj_points_buf; 
    std::vector< int > point_count;



    for (int i = 0;i < success; i++){
        obj_points_buf.clear();
        for (int j = 0; j < board_h; j++){
            for (int k = 0; k < board_w; k++){
                Point3f pt;
                pt.x = k * 10;
                pt.y = j * 10;
                pt.z = 0;
                obj_points_buf.push_back(pt);
            }
        }
        obj_points.push_back( obj_points_buf );
        point_count.push_back( board_n );
    }

    Mat camera_matrix( 3, 3, CV_32FC1, Scalar::all( 0 ) ); 
    Mat dist_coeffs( 1, 5, CV_32FC1, Scalar::all( 0 ) ); 
    std::vector< Mat > rvecs; 
    std::vector< Mat > tvecs;

    std::cout << calibrateCamera( obj_points, img_points, img_size, camera_matrix, dist_coeffs, rvecs, tvecs ) << std::endl; 
    std::cout << camera_matrix << std::endl << dist_coeffs << std::endl;

    return 0;

}