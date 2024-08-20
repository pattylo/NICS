/*
    This file is part of NICS - a non-inertial control system

    NICS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    NICS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NICS.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file led.cpp
 * \date 16/08/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief lib - led
*/

#include "nics_rse/vdrse_lib.h"

/* ================ Initialization utilities function below ================ */
bool nics::VdrseLib::initialization(cv::Mat& frame, cv::Mat depth)
{
    std::get<1>(corres_global_current).clear();

    std::vector<Eigen::Vector2d> pts_2d_detect = LED_extract_POI(frame, depth); 
    std::vector<Eigen::Vector3d> pts_3d_pcl_detect = pointcloud_generate(pts_2d_detect, depth);

    //after above, I got:
    //pointcloud in {c}
    std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

    for(auto what :  pts_3d_pcl_detect)
    {
        norm_of_x_points.push_back(what.x());
        norm_of_y_points.push_back(what.y());
        norm_of_z_points.push_back(what.z());
    }

    if(pts_3d_pcl_detect.size() == LED_no  //we got LED_no
        && calculate_MAD(norm_of_x_points) < MAD_x_threshold //no outlier
        && calculate_MAD(norm_of_y_points) < MAD_y_threshold 
        && calculate_MAD(norm_of_z_points) < MAD_z_threshold) 
    {
        Sophus::SE3d pose;

        int i = 0;
        std::vector<int> corres_to_sort;

        for(int i = 0; i < blobs_for_initialize.size(); i++)
        {
            corres_to_sort.emplace_back(i);
        }
        
        std::vector<int> corres(LED_no);
        std::vector<int> final_corres;
        double error_total = INFINITY;

        Eigen::Matrix3d R;
        Eigen::Vector3d t;

        do{
            corres.clear();
            for(auto what : corres_to_sort)
                corres.push_back(what);

            std::vector<Eigen::Vector2d> pts_2d_detect_temp;   

            for(auto what : corres)
            {
                pts_2d_detect_temp.push_back(pts_2d_detect[what]);
            }
                                                    
            solve_pnp_initial_pose(pts_2d_detect_temp, pts_on_body_frame);
            
            pose_global_sophus = pose_epnp_sophus;

            double e = get_reprojection_error(                    
                pts_on_body_frame,
                pts_2d_detect_temp,
                pose_global_sophus,
                false
            );

            if(e < error_total)
            {                    
                error_total = e;
                final_corres = corres;
            }                        
        }
        while (next_permutation(
            corres_to_sort.begin(), 
            corres_to_sort.end()
            )
        );
                
        BA_error = error_total;

        correspondence::matchid corres_temp;
        
        pts_2d_detect_correct_order.clear();
        
        for(auto what : final_corres)
        {
            corres_temp.detected_indices = what;
            corres_temp.detected_ornot = true;
            corres_temp.pts_3d_correspond = pts_3d_pcl_detect[what];            
            corres_temp.pts_2d_correspond = pts_2d_detect[what];

            pts_2d_detect_correct_order.push_back(pts_2d_detect[what]); 

            std::get<1>(corres_global_current).push_back(corres_temp);
        }

        if(BA_error > LED_no * 3)
        {
            std::cout<<BA_error<<std::endl;
            ROS_WARN("HELLO?");
            patty::Debug("INIT FAIL");
            return false;
        }

        if(std::get<1>(corres_global_current).size() != LED_no)
        {
            ROS_RED_STREAM("PLEASE DEBUG");
        }

        camOptimize(
            pose_global_sophus, 
            pts_on_body_frame, 
            pts_2d_detect_correct_order,
            BA_error
        );

        detect_no = 6;
        std::get<0>(corres_global_current) = detect_no;
        corres_global_previous = corres_global_current;

        std::cout<<pose_epnp_sophus.matrix()<<std::endl<<std::endl;;
        Sophus::SE3d temp_lala = pose_cam_inWorld_SE3 
            * pose_cam_inGeneralBodySE3 
            * pose_epnp_sophus;
        Sophus::SE3d temp_lala2 = pose_cam_inWorld_SE3 
            * pose_cam_inGeneralBodySE3 
            * pose_global_sophus;
    
        return true;
    }
    else
        return false;
}

void nics::VdrseLib::solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> pts_3d)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    cv::Mat no_ro_rmat = cv::Mat::eye(3,3,CV_64F);
    
    cv::Vec3d rvec, tvec;
    // cv::Rodrigues(no_ro_rmat, rvec);

    cv::Mat camMat = cv::Mat::eye(3,3,CV_64F);
    std::vector<cv::Point3f> pts_3d_;
    std::vector<cv::Point2f> pts_2d_;

    cv::Point3f temp3d;
    cv::Point2f temp2d;

    for(auto what : pts_3d)
    {
        temp3d.x = what(0);
        temp3d.y = what(1);
        temp3d.z = what(2);

        pts_3d_.push_back(temp3d);
    }

    for(auto what : pts_2d)
    {
        temp2d.x = what(0);
        temp2d.y = what(1);    

        pts_2d_.push_back(temp2d);
    }

    camMat.at<double>(0,0) = cameraMat(0,0);
    camMat.at<double>(0,2) = cameraMat(0,2);
    camMat.at<double>(1,1) = cameraMat(1,1);
    camMat.at<double>(1,2) = cameraMat(1,2);

    // either one
    // cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
    cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_EPNP);

    //opt pnp algorithm
    //, cv::SOLVEPNP_EPNP
    //, cv::SOLVEPNP_IPPE
    //, cv::SOLVEPNP_P3P

    //return values
    cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);

    R <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);

    Eigen::Matrix3d reverse_mat;
    reverse_mat <<
            1.0000000,  0.0000000,  0.0000000,
            0.0000000, -1.0000000, -0.0000000,
            0.0000000,  0.0000000, -1.0000000;

    t =  Eigen::Vector3d(
          tvec(0),
          tvec(1),
          tvec(2)  
        );

    // R = R * reverse_mat;
    if(tvec(2) < 0) //sometimes opencv yeilds reversed results, flip it 
    {
        R = R * reverse_mat;
        t = (-1) * t;
    }

    pose_epnp_sophus = Sophus::SE3d(R, t);
}

inline double nics::VdrseLib::calculate_MAD(std::vector<double> norm_of_points)
{
    int n = norm_of_points.size();
    double mean = 0, delta_sum = 0, MAD;
    if(n != 0)
    {
        mean = accumulate(norm_of_points.begin(), norm_of_points.end(), 0.0) / n;
        for(int i = 0; i < n; i++)
            delta_sum = delta_sum + abs(norm_of_points[i] - mean);        
        MAD = delta_sum / n;
    }   

    return MAD;
}

/* ================ POI Extraction utilities function below ================ */
std::vector<Eigen::Vector2d> nics::VdrseLib::LED_extract_POI(cv::Mat& frame, cv::Mat depth)
{   
    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, EFFECTIVE_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV);
    // filter out far depths
    
    // depth_mask_src.convertTo(depth_mask_src, CV_8U);
    depth_mask_dst1.convertTo(depth_mask_src, CV_8U);
   
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame, frame, cv::Size(39,39), 1.0, 1.0, cv::BORDER_DEFAULT);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    cv::GaussianBlur(frame, frame, cv::Size(9,9), 1.0, 1.0, cv::BORDER_DEFAULT);
    
    frame_initial_thresholded = frame.clone();

    // detect frame after filter out background
    cv::bitwise_and(depth_mask_src, frame, frame); //filter out with depth information

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = true;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 1.0;
    params.minArea = 1.0;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    detector->detect(frame, keypoints_rgb_d);
	cv::drawKeypoints( frame, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    
    blobs_for_initialize = keypoints_rgb_d;

    min_blob_size = INFINITY;

    std::vector<Eigen::Vector2d> POI;
    for(auto what : keypoints_rgb_d)
    {
        min_blob_size =(what.size < min_blob_size ? what.size : min_blob_size);
        POI.push_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    }
        
    return POI;
}

std::vector<Eigen::Vector2d> nics::VdrseLib::LED_extract_POI_alter(cv::Mat& frame, cv::Mat depth)
{   
    std::vector<Eigen::Vector2d> pts_2d_detected;

    cv::Mat depth_mask_src = depth.clone(), depth_mask_dst1, depth_mask_dst2;

    cv::threshold(depth_mask_src, depth_mask_dst1, EFFECTIVE_DISTANCE * 1000, 50000, cv::THRESH_BINARY_INV);
    //filter out far depths
    
    depth_mask_dst1.convertTo(depth_mask_dst1, CV_8U);
   
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame, frame, cv::Size(39,39), 1.0, 1.0, cv::BORDER_DEFAULT);
    cv::threshold(frame, frame, BINARY_THRES, 255, cv::THRESH_BINARY);
    frame_initial_thresholded = frame.clone();

    // detect frame after filter out background
    cv::bitwise_and(depth_mask_dst1, frame, frame); //filter out with depth information

    Eigen::MatrixXd reproject_2d_pts_matrix;
    reproject_2d_pts_matrix.resize(LED_no, 2);

    for(int i = 0; i < LED_no; i++)
    {
        reproject_2d_pts_matrix.block<1,2>(i, 0) = reproject_3D_2D(
            pts_on_body_frame[i],
            pose_global_sophus
        );
    }

    Eigen::Vector2d minXY = reproject_2d_pts_matrix.colwise().minCoeff();
    Eigen::Vector2d maxXY = reproject_2d_pts_matrix.colwise().maxCoeff();
    Eigen::Vector2d deltaXY = maxXY - minXY;

    minXY = minXY - deltaXY / 2;
    maxXY = maxXY + deltaXY / 2;

    cv::Rect rect_ROI(
        cv::Point2i(minXY.x(), minXY.y()), 
        cv::Point2i(maxXY.x(), maxXY.y())
    );

    cv::Mat ROI_mask = cv::Mat::zeros(
        frame.size(),
        frame.type()
    );

    cv::rectangle(ROI_mask, rect_ROI, CV_RGB(255, 255, 255), -1, 8, 0);
    cv::Mat final_ROI;
    frame.copyTo(final_ROI, ROI_mask);
    // final input should be final_ROI here!!!!!!!!!!!
    cv::GaussianBlur(final_ROI, final_ROI, cv::Size(19,19), 1.0, 1.0, cv::BORDER_DEFAULT);

    // Blob method
    std::vector<cv::KeyPoint> keypoints_rgb_d;
	cv::SimpleBlobDetector::Params params;

	params.filterByArea = false;
    params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByConvexity = false;
	params.filterByInertia = false;
    params.minDistBetweenBlobs = 0.01;

	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    detector->detect(final_ROI, keypoints_rgb_d);
	cv::drawKeypoints(final_ROI, keypoints_rgb_d, im_with_keypoints,CV_RGB(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
   
    blobs_for_initialize = keypoints_rgb_d;

    std::vector<cv::Point2f> POI_pts;
    std::vector<cv::Point2f> centers;
    cv::Mat labels;

    // no k-means
    for(auto what : keypoints_rgb_d)
        pts_2d_detected.emplace_back(Eigen::Vector2d(what.pt.x, what.pt.y));
    
    frame_input = im_with_keypoints.clone();

    if(pts_2d_detected.size() > LED_no)
        ROS_WARN("LED_No over detection!!!!");

    return pts_2d_detected;
}

std::vector<Eigen::Vector3d> nics::VdrseLib::pointcloud_generate(std::vector<Eigen::Vector2d> pts_2d_detected, cv::Mat depthimage)
{
    //get 9 pixels around the point of interest
    int no_pixels = 9;
    int POI_width = (sqrt(9) - 1 ) / 2;

    std::vector<Eigen::Vector3d> pointclouds;

    int x_pixel, y_pixel;
    Eigen::Vector3d temp;

    depth_avg_of_all = 0;
    
    for(int i = 0; i < pts_2d_detected.size(); i++)
    {
        x_pixel = pts_2d_detected[i].x();
        y_pixel = pts_2d_detected[i].y();
        
        cv::Point depthbox_vertice1 = cv::Point(x_pixel - POI_width, y_pixel - POI_width);
        cv::Point depthbox_vertice2 = cv::Point(x_pixel + POI_width, y_pixel + POI_width);
        cv::Rect letsgetdepth(depthbox_vertice1, depthbox_vertice2);

        cv::Mat ROI(depthimage, letsgetdepth);
        cv::Mat ROIframe;
        ROI.copyTo(ROIframe);
        std::vector<cv::Point> nonzeros;

        cv::findNonZero(ROIframe, nonzeros);
        std::vector<double> nonzerosvalue;
        for(auto temp : nonzeros)
        {
            double depth = ROIframe.at<ushort>(temp);
            nonzerosvalue.push_back(depth);
        }

        double depth_average;
        if(nonzerosvalue.size() != 0)
            depth_average = accumulate(nonzerosvalue.begin(), nonzerosvalue.end(),0.0)/nonzerosvalue.size();

        double z_depth = 0.001 * depth_average;

        depth_avg_of_all = depth_avg_of_all + z_depth;

        temp.x() = x_pixel;
        temp.y() = y_pixel;
        temp.z() = 1;

        temp = z_depth * cameraMat.inverse() * temp;
        
        pointclouds.push_back(temp);
    }

    depth_avg_of_all = depth_avg_of_all / pointclouds.size();

    return pointclouds;
}

/* ================ Correspondences utilities function below ================ */
void nics::VdrseLib::get_correspondence(
    std::vector<Eigen::Vector2d>& pts_2d_detected
)
{
    std::vector<Eigen::Vector2d> pts;
    std::vector<Eigen::Vector2d> pts_detected_previous_in_order;

    if(BA_error < 5.0)
    {
        Eigen::Vector2d reproject_temp;
        for(auto what : pts_on_body_frame)
        {
            reproject_temp = reproject_3D_2D(
                    what, 
                    pose_global_sophus
                );
            pts.emplace_back(reproject_temp);
        }

        if(
            pts_2d_detected.size() == LED_no 
            && 
            std::get<0>(corres_global_previous) == LED_no
        )
        {
            pts_detected_previous_in_order =  shift2D(
                pts,
                pts_2d_detected
            );
        }
        else
            pts_detected_previous_in_order = pts;
    }
    else
    {
        ROS_WARN("use previous detection 2D");

        for(auto what : std::get<1>(corres_global_previous))
            pts.emplace_back(what.pts_2d_correspond);

        if(
            pts_2d_detected.size() == LED_no 
            && 
            std::get<0>(corres_global_previous) == LED_no
        )
        {
            pts_detected_previous_in_order =  shift2D(
                pts,
                pts_2d_detected
            );
        }
        else
            pts_detected_previous_in_order = pts;
    }

    correspondence_search_2D2DCompare(
        pts_2d_detected,
        pts_detected_previous_in_order
    );

    corres_global_previous = corres_global_current;
    
    pts_on_body_frame_in_corres_order.clear();
    pts_detected_in_corres_order.clear(); 

    for(int i = 0; i < std::get<1>(corres_global_current).size(); i++)
    {            
        if(std::get<1>(corres_global_current)[i].detected_ornot)
        {  
            pts_detected_in_corres_order.push_back(
                std::get<1>(corres_global_current)[i].pts_2d_correspond
            );             
            pts_on_body_frame_in_corres_order.push_back(pts_on_body_frame[i]);
            std::get<1>(corres_global_current)[i].detected_ornot = false; 
            // reset for next time step
        }        
    }

    i++;        
}

std::vector<Eigen::Vector2d> nics::VdrseLib::shift2D(
    std::vector<Eigen::Vector2d>& pts_2D_previous,
    std::vector<Eigen::Vector2d>& pts_detect_current
)
{
    std::vector<Eigen::Vector2d> shifted_pts;
    int total_no = 0;

    // get average previous
    Eigen::Vector2d cg_previous;
    cg_previous.setZero();
    total_no = 0;
    for(auto& what : pts_2D_previous)
    {
        cg_previous += what;
        total_no++;
    }
    cg_previous /= total_no;

    // get average current
    Eigen::Vector2d cg_current;
    cg_current.setZero();
    total_no = 0;
    for(auto& what : pts_detect_current)
    {
        cv::circle(
            frame_input, 
            cv::Point(what(0), what(1)), 
            2.5, 
            CV_RGB(0,255,0),
            -1
        );
        cg_current += what;
        total_no++;
    }
    cg_current /= total_no;

    Eigen::Vector2d delta2D = cg_current - cg_previous;
    Eigen::Vector2d reproject_temp;

    // shift pts
    for(auto& what : pts_2D_previous)
    {
        reproject_temp = what + delta2D;
        shifted_pts.emplace_back(reproject_temp);
        cv::circle(
            frame_input, 
            cv::Point(reproject_temp(0), reproject_temp(1)), 
            1.0, 
            CV_RGB(0,0,255),
            -1
        );
    }
        
    return shifted_pts;
}

void nics::VdrseLib::correspondence_search_2D2DCompare(
    std::vector<Eigen::Vector2d>& pts_2d_detected,
    std::vector<Eigen::Vector2d>& pts_2d_detected_previous
)
{
    std::vector<cv::Point2f> pts;

    for(auto what : pts_2d_detected_previous)
        pts.emplace_back(cv::Point2f(what.x(), what.y()));
    
    //first emplace back pts_on_body_frame in 2D at this frame
    //in preset order
    for(auto what : pts_2d_detected)
        pts.emplace_back(cv::Point2f(what.x(), what.y()));
    
    std::vector<cv::Point2f> centers;
    cv::Mat labels;

    cv::kmeans(pts, LED_no, labels, cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 10, 1), 8, cv::KMEANS_PP_CENTERS, centers);

    int error_count = 0;

    for(int i = 0; i < LED_no; i++)
    {
        int clusterIdx = labels.at<int>(i);
        for(int k = 0; k < pts.size(); k++)
        {
            if(i == k)
                continue;
            int clusterIdx_detected = labels.at<int>(k);
            if(clusterIdx == clusterIdx_detected)
            {
                if(k < LED_no)
                {
                    error_count++;
                    break;
                }  
                else
                {
                    std::get<1>(corres_global_current)[i].detected_ornot = true;
                    std::get<1>(corres_global_current)[i].pts_2d_correspond = pts_2d_detected[k - LED_no];
                }                                                    
            }
        }
    }    
}

