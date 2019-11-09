/*
  Copyright 2019 coyote009

  This file is part of coyote_racer.

  coyote_racer is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 2 of the License, or
  (at your option) any later version.

  coyote_racer is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with coyote_racer.  If not, see <http://www.gnu.org/licenses/>.

 */

//#include <gperftools/profiler.h>
#include "find_marker.h"
#include "undistort_points.h"

void fm_init( fm_data &fm, int max_id, const std::vector<int> &ng_ids )
{
    fm.dictionary = cv::aruco::getPredefinedDictionary( cv::aruco::DICT_4X4_50 );
    fm.param = cv::aruco::DetectorParameters::create();

    //fm.param->adaptiveThreshWinSizeMin = 7;
    //fm.param->adaptiveThreshWinSizeMax = 7;
    //fm.param->adaptiveThreshWinSizeStep = 10;
    //fm.param->adaptiveThreshConstant = 6;

    //fm.param->minMarkerPerimeterRate = 0.01;

    //fm.param->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    fm.param->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

    fm.marker_size = 7.0;

    fm.max_id = max_id;
    fm.ng_ids = ng_ids;
}

void fm_find_marker( const fm_data &fm, const cv::Mat &img_in, std::vector<fm_marker> &markers,
                     const cv::Mat &mat_cam, const cv::Mat &vec_dist, const cv::Mat &xi )
{
    std::vector<int> marker_ids;
    std::vector< std::vector<cv::Point2f> > marker_corners;
    //std::vector< std::vector<cv::Point2f> > rejected_corners;
    std::vector<cv::Vec3d> vecs_rot, vecs_t;


    cv::aruco::detectMarkers( img_in, fm.dictionary, marker_corners, marker_ids, fm.param /*, rejected_corners*/ );

    std::vector<int> marker_valid( marker_ids.size(), 1 );

    if( !xi.empty() ) /* omnidirectional model */
    {
        std::vector< std::vector<cv::Point2f> > marker_corners_undist( marker_corners.size() );
        for( int i = 0; i < marker_corners.size(); i++ )
        {
            /* Undistort points and get vectors on unit sphere */
            std::vector<cv::Point3f> undist;
            undistort_points( marker_corners[i], undist, mat_cam, vec_dist, xi,
                              cv::Mat::eye( 3, 3, CV_64FC1 ) );

            /* Remove markers close to FoV boundary, i.e. 180deg */
            for( int j = 0; j < undist.size(); j++ )
            {
                if( cvIsNaN( undist[j].z ) || (undist[j].z < 0.1) )
                {
                    marker_valid[i] = 0;
                }
            }
            
            /* Project vectors on screen with focal_len=1 */
            cv::convertPointsFromHomogeneous( undist, marker_corners_undist[i] );
        }

        cv::Mat mat_cam_undist = cv::Mat::eye( 3, 3, CV_64FC1 );
        cv::Mat vec_dist_undist = cv::Mat::zeros( 1, 5, CV_64FC1 );

        cv::aruco::estimatePoseSingleMarkers( marker_corners_undist, fm.marker_size,
                                              mat_cam_undist, vec_dist_undist, vecs_rot, vecs_t );
    }
    else /* perspective model */
    {
        cv::aruco::estimatePoseSingleMarkers( marker_corners, fm.marker_size,
                                              mat_cam, vec_dist, vecs_rot, vecs_t );
    }
        
    markers.clear();
 
    for( uint i=0; i<marker_corners.size(); i++ )
    {
        if( !marker_valid[i] )
        {
            continue;
        }
        
        /* Reject NG markers */
        int ng_marker = 0;
        if( marker_ids[i] > fm.max_id )
        {
            ng_marker = 1;
        }
        else
        {
            for( int j = 0; j < fm.ng_ids.size(); j++ )
            {
                if( marker_ids[i] == fm.ng_ids[j] )
                {
                    ng_marker = 1;
                    break;
                }
            }
        }
        if( ng_marker )
        {
            continue;
        }

        cv::Mat mat_R;
        cv::Mat vec_R;
        cv::Mat vec_t;

        double theta;
        cv::Mat mat_R_2d;
        cv::Mat vec_t_2d;

        fm_marker marker;

        cv::Rodrigues( vecs_rot[i], mat_R );
        mat_R.row( 1 ) *= -1.0;
        mat_R.row( 2 ) *= -1.0;

        vec_t = cv::Mat( vecs_t[i] );
        vec_t.at<double>( 1 ) *= -1.0;
        vec_t.at<double>( 2 ) *= -1.0;

        mat_R = mat_R.t();
        vec_t = -mat_R * vec_t;

        cv::Rodrigues( mat_R, vec_R );

        theta = vec_R.at<double>( 1 );
        mat_R_2d = ( cv::Mat_<double>( 2, 2 ) <<
                     cos( theta ), -sin( theta ),
                     sin( theta ),  cos( theta ) );
        vec_t_2d = ( cv::Mat_<double>( 2, 1 ) <<
                     vec_t.at<double>( 2 ),
                     vec_t.at<double>( 0 ) );

        vec_t_2d = -mat_R_2d.t() * vec_t_2d;
        vec_t_2d.row( 0 ) *= -1.0;

        marker.id = marker_ids[i];
        marker.corners = marker_corners[i];
        marker.coord = vec_t_2d;
        marker.angle = theta;

        markers.push_back( marker );
    }
}

