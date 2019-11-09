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

#include <opencv2/opencv.hpp>
#include "raspivideocap.h"
#include "motor_ctrl.h"
#include "comm_socket.h"
#include "find_marker.h"
#include "track_marker.h"
#include "follow_path.h"

//#define WRITE_LOG

struct camera_parameter
{
    int num_buffers;
    int gray;
    int hflip;
    int vflip;

    cv::Size size_in;
    int fps;
    cv::Rect roi;

    cv::Mat mat_cam;
    cv::Mat vec_dist;
    cv::Mat xi;
};

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

#define CLIP_LU( x, l, u ) ( ((x) < (l)) ? (l) : ( ((x) > (u)) ? (u) : (x) ) )

static void make_slam_stream( const tm_data &tm, double time, const cv::Vec2d &cont_val,
                              std::vector<uchar> &stream )
{
    stream.clear();

    std::vector<float> data;

    data.push_back( (float) time );
    data.push_back( (float) cont_val( 0 ) );
    data.push_back( (float) cont_val( 1 ) );
    
    for( int i = 0; i < 4; i++ )
    {
        data.push_back( (float) tm.x_post.at<double>( i ) );
    }

    for( int i = 0; i < (tm.x_post.rows - 4) / 3; i++ )
    {
        data.push_back( (float) tm.x_post.at<double>( 4 + i*3 ) );
        data.push_back( (float) tm.x_post.at<double>( 4 + i*3 + 1 ) );
        data.push_back( (float) tm.x_post.at<double>( 4 + i*3 + 2 ) );
        data.push_back( (float) tm.markers[i].id );
    }

    stream.resize( data.size() * sizeof(float) );
    memcpy( stream.data(), data.data(), data.size() * sizeof(float) );
}

static void write_log_header( FILE *fp )
{
    fprintf( fp, ",,,,,,,," );

    for( int i = 0; i < 40; i++ )
    {
        fprintf( fp, ",ID%02d,,,,,", i );
    }

    fprintf( fp, "\n" );
    
    fprintf( fp, "frame,time,inv_cond_num,throttle,steering,z_est,x_est,v_est,phi_est" );

    for( int i = 0; i < 40; i++ )
    {
        fprintf( fp, ",z_obs,x_obs,phi_obs,z_est,x_est,phi_obs" );
    }

    fprintf( fp, "\n" );
}

static void write_log( FILE *fp, int frame, double time, const cv::Vec2d &cont_val,
                       const std::vector<fm_marker> fmarkers,
                       const tm_data &tm )
{
    fprintf( fp, "%d,%g,%g,%g,%g", frame, time, tm.inv_cond_num, cont_val(0), cont_val(1) );

    for( int i = 0; i < 4; i++ )
    {
        fprintf( fp, ",%g", tm.x_post.at<double>( i ) );
    }

    /* Max. 39 markers + unknown ID */
    std::vector< std::vector<double> > marker_list( 40, std::vector<double>( 6, -1 ) );

    for( int i = 0; i < fmarkers.size(); i++ )
    {
        int id = fmarkers[i].id;
        id = ( id > 39 ) ? 39 : id;

        marker_list[id][0] = fmarkers[i].coord(0);
        marker_list[id][1] = fmarkers[i].coord(1);
        marker_list[id][2] = fmarkers[i].angle;
    }

    for( int i = 0; i < tm.markers.size(); i++ )
    {
        int id = tm.markers[i].id;
        id = ( id > 39 ) ? 39 : id;

        marker_list[id][3] = tm.markers[i].pt(0);
        marker_list[id][4] = tm.markers[i].pt(1);
        marker_list[id][5] = tm.x_post.at<double>( 4 + i*3+2 );
    }

    for( int i = 0; i < marker_list.size(); i++ )
    {
        for( int j = 0; j < marker_list[i].size(); j++ )
        {
            fprintf( fp, ",%g", marker_list[i][j] );
        }
    }

    for( int i = 0; i < fmarkers.size(); i++ )
    {
        //double area = cv::contourArea( fmarkers[i].corners );
        //fprintf( fp, ",%g", area );

        for( int j = 0; j < fmarkers[i].corners.size(); j++ )
        {
            fprintf( fp, ",%g,%g", fmarkers[i].corners[j].x, fmarkers[i].corners[j].y );
        }
    }

    fprintf( fp, "\n" );
}

static int generate_map( camera_parameter &cam_param,
                         comm_data &comm, data_sender_data &dsender,
                         tm_data &tm )
{
    /*
      Initialize camera
    */
    RaspiVideoCapture cap( cam_param.num_buffers );
    if( !cap.open( cam_param.size_in.width, cam_param.size_in.height, cam_param.fps,
                   cam_param.gray, cam_param.hflip, cam_param.vflip ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    /*
      Initialize find marker
    */
    fm_data fm;
    std::vector<int> ng_markers = { 17 };
    fm_init( fm, 31, ng_markers );

    /*
      Initialize motor control
    */
    motor_ctrl_data mc;
    if( mc_init( mc, "", 1 ) )
    {
        return 1;
    }

    cv::Vec2d cont_val( 0.0, 0.0 );
    double throttle = 0;
    double steering = 0;

    /*
      Commands to be received
    */
    std::vector<std::string> commands = { "EDD", "FWD", "BWD", "STP", "LFT", "RGT", "CNT" };

    /*
      Stream
    */
    std::vector<int> jpg_params;
    jpg_params.push_back( cv::IMWRITE_JPEG_QUALITY );
    jpg_params.push_back( 90 );

    /*
      Log
    */
#ifdef WRITE_LOG
    FILE *fp_log;
    if( !( fp_log = fopen( "../work/log_mg.csv", "w" ) ) )
    {
        fprintf( stderr, "Failed to open log file\n" );
        return 1;
    }

    write_log_header( fp_log );
#endif
    
    /*
      Prepare loop
    */
    //cv::namedWindow( "img_proc" );

    cv::TickMeter timer;
    int frame = 0;
    timer.start();
    double time_begin = (double) cv::getTickCount();
        
    while( 1 )
    {
        cv::Mat img_in;
        cap.read( img_in );

        cv::Mat img_proc;
        img_in( cam_param.roi ).copyTo( img_proc );

        std::vector<fm_marker> markers;
        fm_find_marker( fm, img_proc, markers,
                        cam_param.mat_cam, cam_param.vec_dist, cam_param.xi );

        tm_exec( tm, markers, cont_val );

        /*
          Write log
        */
        double time = ( ((double) cv::getTickCount()) - time_begin ) / cv::getTickFrequency();
#ifdef WRITE_LOG
        write_log( fp_log, frame, time, cont_val, markers, tm );
#endif

        /*
          Send image
        */
        cv::Mat img_sml;
        cv::resize( img_in, img_sml, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
        std::vector<uchar> stream;
        cv::imencode( ".jpg", img_sml, stream, jpg_params );
        data_sender_send( dsender, "IMG", stream );

        /*
          Send SLAM data
        */
        make_slam_stream( tm, time, cont_val, stream );
        data_sender_send( dsender, "SLM", stream );
        
        frame++;

        int command = comm_check_cmds( comm, commands );
        if( command >= 0 )
        {
            int end_loop = 0;
            int change_motor = 0;
            switch( command )
            {
            case 0: end_loop = 1; break;
            case 1: cont_val(0) += 0.1; change_motor = 1; break;
            case 2: cont_val(0) -= 0.1; change_motor = 1; break;
            case 3: cont_val(0) = cont_val(1) = 0.0; change_motor = 1; break;
            case 4: cont_val(1) -= 0.1; change_motor = 1; break;
            case 5: cont_val(1) += 0.1; change_motor = 1; break;
            case 6: cont_val(1) = 0.0; change_motor = 1; break;
            }

            //printf( "Command #%d received\n", command );

            if( end_loop )
            {
                mc_set_val( mc, 0, 0 );
                break;
            }
            else if( change_motor )
            {
                cont_val(0) = CLIP_LU( cont_val(0), -1.0, 1.0 );
                cont_val(1) = CLIP_LU( cont_val(1), -1.0, 1.0 );
                mc_set_val( mc, cont_val(0), cont_val(1) );
            }
        }
    }

    timer.stop();

    //double avg_time = timer.getTimeSec() / timer.getCounter();
    double avg_time = timer.getTimeSec() / ((double) frame );
    printf( "Time/Frame = %f[sec], FPS = %f\n", avg_time, 1.0 / avg_time );

#ifdef WRITE_LOG
    fclose( fp_log );
#endif
    
    mc_fina( mc );

    return 0;
}

struct path_data
{
    std::vector<cv::Point2f> nodes;
    std::vector<float> velocity;
    int loop;

    int current_idx;
};

static int wait_path_data( comm_data &comm, path_data &path )
{
    std::vector<std::string> commands = { "PTH" };
    std::vector<uchar> buf;

    cv::namedWindow( "wait" );
    
    while( 1 )
    {
        if( comm_receive_data( comm, commands, buf ) == 0 )
        {
            int num_data = ( buf.size() - sizeof(int) ) / ( sizeof(float) * 3 );
            path.nodes.resize( num_data );
            path.velocity.resize( num_data );

            int node_len = num_data * 2 * sizeof(float);
            int velocity_len = num_data * sizeof(float);
            memcpy( path.nodes.data(), buf.data(), node_len );
            memcpy( path.velocity.data(), buf.data() + node_len, velocity_len );
            memcpy( &path.loop, buf.data() + node_len + velocity_len, sizeof(int) );

            break;
        }

        if( cv::waitKey( 50 ) == 27 )
        {
            break;
        }
    }

    cv::destroyWindow( "wait" );

    return 0;
}

static int set_fp_target( path_data &path, follow_path_target &target )
{
    target.pt_src = path.nodes[path.current_idx];
    target.velocity_src = path.velocity[path.current_idx];
    
    if( path.current_idx == (path.nodes.size() - 1) )
    {
        if( path.loop == 0 )
        {
            return 1;
        }

        target.pt_dst = path.nodes[0];
        target.velocity_dst = path.velocity[0];
    }
    else
    {
        target.pt_dst = path.nodes[path.current_idx+1];
        target.velocity_dst = path.velocity[path.current_idx+1];
    }

    return 0;
}

static void compute_control( path_data &path, cv::Mat &car_state, cv::Vec2d &cont_val )
{
    if( path.nodes.size() < 2 )
    {
        cont_val = cv::Vec2d( 0, 0 );
        return;
    }
    
    follow_path_target target;

    if( set_fp_target( path, target ) )
    {
        cont_val = cv::Vec2d( 0, 0 );
        return;
    }

    cv::Vec2f pt_car = cv::Vec2d( car_state( cv::Rect( 0, 0, 1, 2 ) ) );
    float velocity_car = car_state.at<double>( 2 );
    float angle_car = car_state.at<double>( 3 );

    while( fp_reached_end( pt_car, target ) )
    {
        if( path.current_idx == (path.nodes.size() - 1) )
        {
            path.current_idx = 0;
        }
        else
        {
            path.current_idx++;
        }

        if( set_fp_target( path, target ) )
        {
            cont_val = cv::Vec2d( 0, 0 );
            return;
        }
    }

    //const float throttle_eq_coef = 0.2 / 75.0; /*!!!!!!!! a/b from dynamics model !!!!!!!!*/
    const float throttle_eq_coef = 1.1 / 260.0; /*!!!!!!!! a/b from dynamics model !!!!!!!!*/
    float throttle, steering;
    fp_compute_control( pt_car, velocity_car, angle_car,
                        target, throttle_eq_coef, throttle, steering );

    cont_val = cv::Vec2d( throttle, steering );
}

static int follow_path( camera_parameter &cam_param,
                        comm_data &comm, data_sender_data &dsender,
                        tm_data &tm, path_data &path )
{
    /*
      Initialize camera
    */
    RaspiVideoCapture cap( cam_param.num_buffers );
    if( !cap.open( cam_param.size_in.width, cam_param.size_in.height, cam_param.fps,
                   cam_param.gray, cam_param.hflip, cam_param.vflip ) )
    {
        fprintf( stderr, "Failed to open camera\n" );
        return 1;
    }

    /*
      Initialize find marker
    */
    fm_data fm;
    std::vector<int> ng_markers = { 17 };
    fm_init( fm, 31, ng_markers );

    /*
      Initialize motor control
    */
    motor_ctrl_data mc;
    if( mc_init( mc, "", 1 ) )
    {
        return 1;
    }

    cv::Vec2d cont_val( 0.0, 0.0 );
    double throttle = 0;
    double steering = 0;

    /*
      Stream
    */
    std::vector<int> jpg_params;
    jpg_params.push_back( cv::IMWRITE_JPEG_QUALITY );
    jpg_params.push_back( 90 );

    /*
      Log
    */
#ifdef WRITE_LOG
    FILE *fp_log;
    if( !( fp_log = fopen( "../work/log_fp.csv", "w" ) ) )
    {
        fprintf( stderr, "Failed to open log file\n" );
        return 1;
    }

    write_log_header( fp_log );
#endif
    
    /*
      Prepare loop
    */
    //cv::namedWindow( "img_proc" );

    /* Read and throw away initial images */
    for( int i = 0; i < 10; i++ )
    {
        cv::Mat img_in;
        cap.read( img_in );
    }

    cv::TickMeter timer;
    int frame = 0;
    timer.start();
    double time_begin = (double) cv::getTickCount();
        
    while( 1 )
    {
        cv::Mat img_in;
        cap.read( img_in );

        cv::Mat img_proc;
        img_in( cam_param.roi ).copyTo( img_proc );

        std::vector<fm_marker> markers;
        fm_find_marker( fm, img_proc, markers,
                        cam_param.mat_cam, cam_param.vec_dist, cam_param.xi );

        tm_exec( tm, markers, cont_val );

        compute_control( path, tm.x_post, cont_val );
        cont_val(0) = CLIP_LU( cont_val(0), -1.0, 1.0 );
        cont_val(1) = CLIP_LU( cont_val(1), -1.0, 1.0 );
#if 0
        if( frame % 20 == 0 )
        {
            printf( "velocity=%g throttle=%g steering=%g current_idx=%d\n",
                    tm.x_post.at<double>(2),  cont_val(0), cont_val(1), path.current_idx );
        }
#else
        mc_set_val( mc, cont_val(0), cont_val(1) );
#endif

        /*
          Write log
        */
        double time = ( ((double) cv::getTickCount()) - time_begin ) / cv::getTickFrequency();
#ifdef WRITE_LOG
        write_log( fp_log, frame, time, cont_val, markers, tm );
#endif

        /*
          Send image
        */
        cv::Mat img_sml;
        cv::resize( img_in, img_sml, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST );
        std::vector<uchar> stream;
        cv::imencode( ".jpg", img_sml, stream, jpg_params );
        data_sender_send( dsender, "IMG", stream );

        /*
          Send SLAM data
        */
        make_slam_stream( tm, time, cont_val, stream );
        data_sender_send( dsender, "SLM", stream );
        
        frame++;

        if( comm_check_cmd( comm, "EDD" ) )
        {
            mc_set_val( mc, 0, 0 );
            break;
        }
    }

    timer.stop();

    //double avg_time = timer.getTimeSec() / timer.getCounter();
    double avg_time = timer.getTimeSec() / ((double) frame );
    printf( "Time/Frame = %f[sec], FPS = %f\n", avg_time, 1.0 / avg_time );

#ifdef WRITE_LOG
    fclose( fp_log );
#endif
    
    mc_fina( mc );

    return 0;
}

int main( int argc, char **argv )
{
    /*
      Camera and calibration parameters
    */
    const int num_buffers = 10;
    const int gray = 0;
    const int hflip = 1;
    const int vflip = 1;
    const cv::Size size_in( 480, 288 );
    const int fps = 120;
    const cv::Rect roi( 0, 0, size_in.width, size_in.width/2 );
    const std::string fname_calib = "../work/calib_intrinsics_960x576_120.yml";

    camera_parameter cam_param;

    cam_param.num_buffers = num_buffers;
    cam_param.gray = gray;
    cam_param.hflip = hflip;
    cam_param.vflip = vflip;
    cam_param.size_in = size_in;
    cam_param.fps = fps;
    cam_param.roi = roi;

    read_intrinsics( fname_calib, cam_param.size_in,
                     cam_param.mat_cam, cam_param.vec_dist, cam_param.xi, cam_param.roi );

    /*
      Initialize communication
    */
    comm_data comm;
    std::string client_addr;
    if( comm_server_init_wait( comm, client_addr ) )
    {
        fprintf( stderr, "Failed to initialize communication\n" );
        return 1;
    }

    data_sender_data dsender;
    if( data_sender_init( dsender, client_addr ) )
    {
        fprintf( stderr, "Error initializing data_sender\n" );
        comm_fina( comm );
        return 1;
    }

    /*
      Map generation
    */
    tm_data tm; /* map_data */

    if( generate_map( cam_param, comm, dsender, tm ) )
    {
        fprintf( stderr, "Error in map generation\n" );
        comm_fina( comm );
        return 1;
    }

    /*
      Wait for path generation
    */
    path_data path;
    if( wait_path_data( comm, path ) )
    {
        fprintf( stderr, "Failed to get path data\n" );
        comm_fina( comm );
        return 1;
    }

    printf( "Received path list\n" );
    for( int i = 0; i < path.nodes.size(); i++ )
    {
        printf( "(%g,%g)@%g\n", path.nodes[i].x, path.nodes[i].y, path.velocity[i] );
    }
    printf( "loop=%d\n", path.loop );

    /*
      Path following
    */
    path.current_idx = 0;
    if( follow_path( cam_param, comm, dsender, tm, path ) )
    {
        fprintf( stderr, "Error in path following\n" );
        comm_fina( comm );
        return 1;
    }

    /*
      Finalize communication
    */

    if( data_sender_fina( dsender ) )
    {
        fprintf( stderr, "Error finalizing data_sender\n" );
    }
    comm_fina( comm );

    return 0;
}
