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

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "comm_socket.h"

static void draw_map( const std::vector<float> &slam_data, cv::Mat &img_map,
                      const cv::Point &map_origin, int print_time = 1 )
{
    cv::Point2f pt;
    char buf[64];

    if( print_time )
    {
        float time = slam_data[0];
        sprintf( buf, "time=%f", time );
        cv::putText( img_map, buf, cv::Point( 0, 15 ),
                     cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,255,0) );
    }
    
    {
        const float cont_scale = 50.0;
        cv::Vec2f cont_val( slam_data[1], slam_data[2] );

        cv::Vec2f pos( slam_data[3], slam_data[4] );
        float velocity = slam_data[5];
        float angle = slam_data[6];

        float cos_p = cos( angle );
        float sin_p = sin( angle );
        
        cv::Vec2f vec( velocity * cos_p, velocity * sin_p );
        cv::Vec2f vec_cont( cos_p * cont_val(0) - sin_p * cont_val(1),
                            sin_p * cont_val(0) + cos_p * cont_val(1) );
        vec_cont *= cont_scale;

        pt.x = map_origin.x + pos(1);
        pt.y = map_origin.y - pos(0);
        cv::circle( img_map, pt, 2, CV_RGB(0,255,0) );

        cv::line( img_map, pt, pt + cv::Point2f( vec(1), -vec(0) ),
                  CV_RGB(0,255,0) );

        cv::line( img_map, pt, pt + cv::Point2f( vec_cont(1), -vec_cont(0) ),
                  CV_RGB(255,0,0) );
    }
    
    for( int i = 7; i < slam_data.size(); i += 4 )
    {
        cv::Vec2f pos( slam_data[i], slam_data[i+1] );
        int id = (int) slam_data[i+3];

        pt.x = map_origin.x + pos(1);
        pt.y = map_origin.y - pos(0);
        cv::circle( img_map, pt, 2, CV_RGB(255,255,255) );

        sprintf( buf, "%d", id );
        cv::putText( img_map, buf, pt, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,255,255) );
    }
}

int generate_map( comm_data &comm, data_receiver_data &dreceiver,
                  const std::string &fname_out,
                  const cv::Size &map_size, const cv::Point &map_origin,
                  std::vector<float> &slam_data )
{
    cv::Size img_size( map_size.width, map_size.height/2 );
    cv::Size out_size( map_size.width, map_size.height + img_size.height );
    
    cv::VideoWriter vw;
    if( !vw.open( fname_out, cv::VideoWriter::fourcc('X','V','I','D'),
                  30.0, out_size ) )
    {
        fprintf( stderr, "Failed to open output AVI\n" );
        return 1;
    }

    cv::Mat img_out( out_size, CV_8UC3 );

    cv::namedWindow( "img_stream" );

    double time_begin = (double) cv::getTickCount();
    int video_frame = 0;

    while( 1 )
    {
        cv::Mat img_stream;
        cv::Mat img_map = cv::Mat::zeros( map_size, CV_8UC3 );
        
        std::vector<uchar> stream;
        if( data_receiver_receive( dreceiver, stream ) == 0 )
        {
            if( dreceiver.cmd == "IMG" )
            {
                img_stream = cv::imdecode( stream, cv::IMREAD_COLOR );
                if( !img_stream.empty() )
                {
                    cv::imshow( "img_stream", img_stream );

                    cv::resize( img_stream,
                                img_out( cv::Rect( 0, 0,
                                                   img_size.width,
                                                   img_size.height ) ),
                                img_size, 0, 0, cv::INTER_LINEAR );
                }
            }
            else if( dreceiver.cmd == "SLM" )
            {
                if( ( stream.size() % 4 ) == 0 )
                {
                    slam_data.resize( stream.size() / 4 );
                    memcpy( slam_data.data(), stream.data(), stream.size() );

                    draw_map( slam_data, img_map, map_origin );

                    cv::imshow( "img_map", img_map );

                    img_map.copyTo( img_out( cv::Rect( 0, img_size.height,
                                                       map_size.width,
                                                       map_size.height ) ) );
                }
            }
        }

        /* record video @ 30fps */
        double time = ( ((double) cv::getTickCount()) - time_begin ) / cv::getTickFrequency();
        if( time > (0.033 * video_frame) )
        {
            vw.write( img_out );
            video_frame++;
        }

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            if( comm_send_cmd( comm, "EDD" ) == 4 )
            {
                break;
            }
        }
        else
        {
            switch( key )
            {
            case 'i': comm_send_cmd( comm, "FWD" ); break;
            case 'k': comm_send_cmd( comm, "BWD" ); break;
            case ' ': comm_send_cmd( comm, "STP" ); break;
            case 'j': comm_send_cmd( comm, "LFT" ); break;
            case 'l': comm_send_cmd( comm, "RGT" ); break;
            case ',': comm_send_cmd( comm, "CNT" ); break;
            }
        }
    }

    cv::destroyWindow( "img_stream" );
    cv::destroyWindow( "img_map" );

    return 0;
}

struct path_data
{
    std::vector<cv::Point2f> nodes;
    std::vector<float> velocity;
    int loop;
    path_data() : loop( 0 ) {}
};

struct path_input_data
{
    path_data *ppath;
    int trackbar_val;
    path_input_data() : trackbar_val( 0 ) {}
};

static void on_mouse( int event, int x, int y, int flags, void *userdata )
{
    const float proximity_thresh = 4.0;
    path_input_data *ppath = (path_input_data *) userdata;

    if( event == cv::EVENT_LBUTTONDOWN )
    {
        if( ppath->ppath->loop )
        {
            /* Path loop already closed */
            return;
        }
        
        if( ppath->ppath->nodes.size() )
        {
            cv::Point2f vec_to_start = ppath->ppath->nodes[0] - cv::Point2f( x, y );
            if( cv::norm( vec_to_start ) < proximity_thresh )
            {
                ppath->ppath->loop = 1;
                return;
            }
        }

        ppath->ppath->nodes.push_back( cv::Point2f( x, y ) );
        ppath->ppath->velocity.push_back( (float) ppath->trackbar_val * 10.0 );
    }
    else if( event == cv::EVENT_RBUTTONDOWN )
    {
        if( ppath->ppath->loop )
        {
            ppath->ppath->loop = 0;
        }
        else if( ppath->ppath->nodes.size() )
        {
            ppath->ppath->nodes.pop_back();
            ppath->ppath->velocity.pop_back();
        }
    }
}

static void draw_path( path_data &path, cv::Mat &img )
{
    const cv::Scalar color = CV_RGB(128,128,128);
    char buf[64];
    for( int i = 0; i < path.nodes.size(); i++ )
    {
        cv::circle( img, path.nodes[i], 2, color );
        if( i != 0 )
        {
            cv::line( img, path.nodes[i-1], path.nodes[i], color );
        }

        if( path.velocity[i] > -1.0e-10 )
        {
            sprintf( buf, "%d", (int) path.velocity[i] );
            cv::putText( img, buf, path.nodes[i],
                         cv::FONT_HERSHEY_SIMPLEX, 0.4,
                         color );
        }
    }
    if( path.loop && (path.nodes.size() >= 3) )
    {
        cv::line( img, path.nodes[path.nodes.size()-1],
                  path.nodes[0], color );
    }
}

static void draw_from_actual_path( path_data &path, cv::Mat &img,
                                   const cv::Point &map_origin )
{
    const cv::Scalar color = CV_RGB(128,128,128);

    std::vector<cv::Point2f> map_nodes( path.nodes.size() );
    for( int i = 0; i < path.nodes.size(); i++ )
    {
        map_nodes[i].x = path.nodes[i].y + map_origin.x;
        map_nodes[i].y = map_origin.y - path.nodes[i].x;
    }
    
    char buf[64];
    for( int i = 0; i < path.nodes.size(); i++ )
    {
        cv::circle( img, map_nodes[i], 2, color );
        if( i != 0 )
        {
            cv::line( img, map_nodes[i-1], map_nodes[i], color );
        }

        if( path.velocity[i] > -1.0e-10 )
        {
            sprintf( buf, "%d", (int) path.velocity[i] );
            cv::putText( img, buf, map_nodes[i],
                         cv::FONT_HERSHEY_SIMPLEX, 0.4,
                         color );
        }
    }
    if( path.loop && (path.nodes.size() >= 3) )
    {
        cv::line( img, map_nodes[path.nodes.size()-1],
                  map_nodes[0], color );
    }
}

static int set_path( std::vector<float> &slam_data,
                     const cv::Size &map_size, const std::string &fname_out,
                     const cv::Point &map_origin, path_data &path )
{
    path_input_data path_input;
    path_input.ppath = &path;

    cv::VideoWriter vw;
    if( !vw.open( fname_out, cv::VideoWriter::fourcc('X','V','I','D'),
                  30.0, map_size ) )
    {
        fprintf( stderr, "Failed to open output AVI\n" );
        return 1;
    }

    cv::namedWindow( "map" );
    cv::setMouseCallback( "map", on_mouse, (void *) &path_input );

    cv::createTrackbar( "velocity(10cm/s)", "map", &path_input.trackbar_val, 10 );

    double time_begin = (double) cv::getTickCount();
    int video_frame = 0;

    while( 1 )
    {
        cv::Mat img_map = cv::Mat::zeros( map_size, CV_8UC3 );

        draw_map( slam_data, img_map, map_origin, 0 );
        draw_path( path, img_map );

        cv::imshow( "map", img_map );

        /* record video @ 30fps */
        double time = ( ((double) cv::getTickCount()) - time_begin ) / cv::getTickFrequency();
        if( time > (0.033 * video_frame) )
        {
            vw.write( img_map );
            video_frame++;
        }

        int key = cv::waitKey( 10 );
        if( key == 27 )
        {
            break;
        }
    }

    for( int i = 0; i < path.nodes.size(); i++ )
    {
        cv::Point2f pt_actual( map_origin.y - path.nodes[i].y,
                               path.nodes[i].x - map_origin.x );
        path.nodes[i] = pt_actual;
    }

    cv::destroyWindow( "map" );

    return 0;
}

static int send_path( comm_data &comm, const path_data &path )
{
    int node_len = path.nodes.size() * sizeof(float) * 2;
    int velocity_len = path.nodes.size() * sizeof(float);
    int loop_len = sizeof(int);
    int total_len = node_len + velocity_len + loop_len;

    std::vector<uchar> buf( total_len );

    memcpy( buf.data(), path.nodes.data(), node_len );
    memcpy( buf.data() + node_len, path.velocity.data(), velocity_len );
    memcpy( buf.data() + node_len + velocity_len, &path.loop, loop_len );

    if( comm_send_data( comm, "PTH", buf ) != total_len )
    {
        return 1;
    }

    return 0;
}

int follow_path( comm_data &comm, data_receiver_data &dreceiver,
                 const std::string &fname_out, const std::string &fname_log,
                 const cv::Size &map_size, const cv::Point &map_origin,
                 std::vector<float> &slam_data, path_data &path )
{
    cv::Size img_size( map_size.width, map_size.height/2 );
    cv::Size out_size( map_size.width, map_size.height + img_size.height );
    
    cv::VideoWriter vw;
    if( !vw.open( fname_out, cv::VideoWriter::fourcc('X','V','I','D'),
                  30.0, out_size ) )
    {
        fprintf( stderr, "Failed to open output AVI\n" );
        return 1;
    }

    cv::Mat img_out = cv::Mat::zeros( out_size, CV_8UC3 );

    cv::namedWindow( "img_stream" );

    FILE *fp_log = fopen( fname_log.c_str(), "w" );
    if( !fp_log )
    {
        fprintf( stderr, "Failed to open log file\n" );
        return 1;
    }
    fprintf( fp_log, "time,throttle,steering,pos_z,pos_x,velocity,angle"
             "marker0_z,marker0_x,marker0_angle,marker0_id\n" );

    /* flush UDP buffer */
    for( int i = 0; i < 10; i++ )
    {
        std::vector<uchar> stream;
        data_receiver_receive( dreceiver, stream );
    }

    double time_begin = (double) cv::getTickCount();
    int video_frame = 0;

    while( 1 )
    {
        cv::Mat img_stream;
        cv::Mat img_map = cv::Mat::zeros( map_size, CV_8UC3 );
        
        std::vector<uchar> stream;
        if( data_receiver_receive( dreceiver, stream ) == 0 )
        {
            if( dreceiver.cmd == "IMG" )
            {
                img_stream = cv::imdecode( stream, cv::IMREAD_COLOR );
                if( !img_stream.empty() )
                {
                    cv::imshow( "img_stream", img_stream );

                    cv::resize( img_stream,
                                img_out( cv::Rect( 0, 0,
                                                   img_size.width,
                                                   img_size.height ) ),
                                img_size, 0, 0, cv::INTER_LINEAR );
                }
            }
            else if( dreceiver.cmd == "SLM" )
            {
                if( ( stream.size() % 4 ) == 0 )
                {
                    slam_data.resize( stream.size() / 4 );
                    memcpy( slam_data.data(), stream.data(), stream.size() );

                    draw_map( slam_data, img_map, map_origin );
                    draw_from_actual_path( path, img_map, map_origin );

                    for( int i = 0; i < slam_data.size(); i++ )
                    {
                        fprintf( fp_log, "%g,", slam_data[i] );
                    }
                    fprintf( fp_log, "\n" );

                    cv::imshow( "img_map", img_map );

                    img_map.copyTo( img_out( cv::Rect( 0, img_size.height,
                                                       map_size.width,
                                                       map_size.height ) ) );
                }
            }
        }

        /* record video @ 30fps */
        double time = ( ((double) cv::getTickCount()) - time_begin ) / cv::getTickFrequency();
        if( time > (0.033 * video_frame) )
        {
            vw.write( img_out );
            video_frame++;
        }

        int key = cv::waitKey( 1 );
        if( key == 27 )
        {
            if( comm_send_cmd( comm, "EDD" ) == 4 )
            {
                break;
            }
        }
    }

    fclose( fp_log );

    return 0;
}

int main( int argc, char **argv )
{
    comm_data comm;
    data_receiver_data dreceiver;
    std::string server_ip;
    std::string client_ip;

    if( argc != 3 )
    {
        fprintf( stderr, "Usage: %s <server_ip> <client_ip>\n", argv[0] );
        return 1;
    }
    else
    {
        server_ip = argv[1];
        client_ip = argv[2];
    }

    if( comm_client_init_connect( comm, server_ip, client_ip, 9876 ) )
    {
        return 1;
    }

    if( data_receiver_init( dreceiver ) )
    {
        comm_fina( comm );
        return 1;
    }

    const cv::Size map_size( 480, 480 );
    const cv::Point map_origin( map_size.width/4, map_size.height/2 );
    std::vector<float> slam_data;
    if( generate_map( comm, dreceiver, "../work/out_gm.avi",
                      map_size, map_origin, slam_data ) )
    {
        data_receiver_fina( dreceiver, comm );
        comm_send_cmd( comm, "EDD" );
        comm_fina( comm );
        return 1;
    }

    path_data path;
    if( set_path( slam_data, map_size, "../work/out_sp.avi",
                  map_origin, path ) )
    {
        data_receiver_fina( dreceiver, comm );
        comm_send_cmd( comm, "EDD" );
        comm_fina( comm );
        return 1;
    }

    for( int i = 0; i < path.nodes.size(); i++ )
    {
        printf( "(%g,%g)@%g\n", path.nodes[i].x, path.nodes[i].y,
                path.velocity[i] );
    }
    printf( "loop=%d\n", path.loop );

    if( send_path( comm, path ) )
    {
        fprintf( stderr, "Failed to send path\n" );
    }

    if( follow_path( comm, dreceiver, "../work/out_fp.avi", "../work/out_fp.csv",
                     map_size, map_origin, slam_data, path ) )
    {
        data_receiver_fina( dreceiver, comm );
        comm_send_cmd( comm, "EDD" );
        comm_fina( comm );
        return 1;
    }

    data_receiver_fina( dreceiver, comm );
    comm_fina( comm );

    return 0;
}
