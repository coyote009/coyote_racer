/*
  Copyright 2020 coyote009

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

#include <opencv2/opencv.hpp>
#include "raspivideocap.h"
#include "find_marker.h"

static void read_intrinsics( const std::string &fname, const cv::Size &size_proc,
                             cv::Mat &mat_cam, cv::Mat &vec_dist, cv::Mat &xi,
                             const cv::Rect &roi )
{
    cv::Size size_in;

    cv::FileStorage fs;
    if( !fs.open( fname, cv::FileStorage::READ ) )
    {
        fprintf( stderr, "Failed to open intrinsics\n" );
        return;
    }
    fs["MAT_CAM"] >> mat_cam;
    fs["VEC_DIST"] >> vec_dist;
    fs["XI"] >> xi;
    fs["SIZE_IN"] >> size_in;
    fs.release();

    assert( size_proc.width * size_in.height == size_proc.height * size_in.width );

    double size_ratio = ((double) size_proc.width) / ((double) size_in.width);

    mat_cam.at<double>( 0, 0 ) *= size_ratio;
    mat_cam.at<double>( 1, 1 ) *= size_ratio;
    mat_cam.at<double>( 0, 2 ) = ( mat_cam.at<double>( 0, 2 ) + 0.5 ) * size_ratio - roi.x - 0.5;
    mat_cam.at<double>( 1, 2 ) = ( mat_cam.at<double>( 1, 2 ) + 0.5 ) * size_ratio - roi.y - 0.5;
}

int main()
{
    const int num_buffers = 10;
    const int gray = 0;
    const int hflip = 1;
    const int vflip = 1;

#if 0
    const cv::Size size_in( 480, 480 );
    const int fps = 40;
    const cv::Rect roi( 0, 96, size_in.width, size_in.height/2 );
    std::string fname_calib = "../work/calib_intrinsics_1216x1216.yml";
#else
    const cv::Size size_in( 480, 288 );
    const int fps = 120;
    const cv::Rect roi( 0, 0, size_in.width, size_in.width/2 );
    std::string fname_calib = "../work/calib_intrinsics_960x576_120.yml";
#endif

    cv::Mat mat_cam;
    cv::Mat vec_dist;
    cv::Mat xi;
    read_intrinsics( fname_calib, size_in, mat_cam, vec_dist, xi, roi );

    RaspiVideoCapture cap( num_buffers );
    if( !cap.open( size_in.width, size_in.height, fps, gray, hflip, vflip ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    fm_data fm;
    fm_init( fm );

    //FILE *fp_log = fopen( "../work/log.csv", "w" );
    //if( !fp_log )
    //{
    //    fprintf( stderr, "Failed to open log file\n" );
    //    return 1;
    //}

    cv::namedWindow( "img_proc" );

    cv::TickMeter tm;
    int frame = 0;
    tm.start();
        
    while( 1 )
    {
        cv::Mat img_in;
        cap.read( img_in );

        cv::Mat img_proc;
        img_in( roi ).copyTo( img_proc );

        std::vector<fm_marker> markers;
        fm_find_marker( fm, img_proc, markers, mat_cam, vec_dist, xi );

#if 1 /* imshow consumes 13fps */
        std::vector< std::vector<double> > logs( 40, std::vector<double>( 13, -1 ) );
        
        for( int i=0; i<markers.size(); i++ )
        {
            char buf[64];


            uint j;
            for( j=1; j<markers[i].corners.size(); j++ )
            {
                cv::line( img_proc, markers[i].corners[j-1], markers[i].corners[j], CV_RGB(0,255,0) );
            }
            cv::line( img_proc, markers[i].corners[j-1], markers[i].corners[0], CV_RGB(0,255,0) );

            sprintf( buf, "id=%d", markers[i].id );
            cv::putText( img_proc, buf, markers[i].corners[0],
                         cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255,0,0) );

            int area = (int) cv::contourArea( markers[i].corners );
            sprintf( buf, "area=%d", area );
            cv::putText( img_proc, buf, markers[i].corners[0] - cv::Point2f(0,10),
                         cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255,0,0) );

            sprintf( buf, "pos=(%f,%f)", markers[i].coord(0), markers[i].coord(1) );
            cv::putText( img_proc, buf, markers[i].corners[0] - cv::Point2f(0,20),
                         cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255,0,0) );

            sprintf( buf, "angle=%f", markers[i].angle / CV_PI * 180.0 );
            cv::putText( img_proc, buf, markers[i].corners[0] - cv::Point2f(0,30),
                         cv::FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255,0,0) );

            int id = markers[i].id;
            id = (id > 39) ? 39 : id;

            logs[id][0] = id;
            logs[id][1] = area;
            logs[id][2] = markers[i].coord(0);
            logs[id][3] = markers[i].coord(1);
            logs[id][4] = markers[i].angle;

            for( j = 0 ; j < markers[i].corners.size(); j++ )
            {
                logs[id][5+2*j] = markers[i].corners[j].x;
                logs[id][5+2*j+1] = markers[i].corners[j].y;
            }
        }

        //for( int i = 0; i < logs.size(); i++ )
        //{
        //    for( int j = 0; j < logs[i].size(); j++ )
        //    {
        //        fprintf( fp_log, "%g,", logs[i][j] );
        //    }
        //}
        //fprintf( fp_log, "\n" );

        cv::imshow( "img_proc", img_proc );
#endif
        
        frame++;
        if( cv::waitKey( 1 ) == 27 )
        {
            break;
        }
    }

    tm.stop();

    //double avg_time = tm.getTimeSec() / tm.getCounter();
    double avg_time = tm.getTimeSec() / ((double) frame );
    printf( "Time/Frame = %f[sec], FPS = %f\n", avg_time, 1.0 / avg_time );

    //fclose( fp_log );
    
    return 0;
}
