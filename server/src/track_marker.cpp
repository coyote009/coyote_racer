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
#include "track_marker.h"

#if 0
#define TM_PARAM_T 0.033
#define TM_PARAM_A 1.6
#define TM_PARAM_B 71.2
#define TM_PARAM_WB 11.2
#elif 0
#define TM_PARAM_T 0.02
#define TM_PARAM_A 0.2
#define TM_PARAM_B 75.0
#define TM_PARAM_WB 11.2
#else
#define TM_PARAM_T 0.02
#define TM_PARAM_A 1.1
#define TM_PARAM_B 260.0
#define TM_PARAM_WB 11.2
#endif

static void f_func( cv::Mat &x_post, cv::Mat &u, cv::Mat &x_pre )
{
    const double T = TM_PARAM_T;
    const double a = TM_PARAM_A;
    const double b = TM_PARAM_B;
    const double wb = TM_PARAM_WB;
    double z, x, v, phi;
    double E, th;


    x_post.copyTo( x_pre );

    z = x_post.at<double>( 0 );
    x = x_post.at<double>( 1 );
    v = x_post.at<double>( 2 );
    phi = x_post.at<double>( 3 );

    E = u.at<double>( 0 );
    th = u.at<double>( 1 );

    x_pre.at<double>( 0 ) = z + T * v * cos( phi );
    x_pre.at<double>( 1 ) = x + T * v * sin( phi );
    x_pre.at<double>( 2 ) = ( 1 - a * T ) * v + b * T * E;
    x_pre.at<double>( 3 ) = phi + T / wb * v * sin( th );
}

static void f_jacobian( cv::Mat &x_post, cv::Mat &u, cv::Mat &dF )
{
    const double T = TM_PARAM_T;
    const double a = TM_PARAM_A;
    const double wb = TM_PARAM_WB;
    double z, x, v, phi;
    double th;


    z = x_post.at<double>( 0 );
    x = x_post.at<double>( 1 );
    v = x_post.at<double>( 2 );
    phi = x_post.at<double>( 3 );

    th = u.at<double>( 1 );

    if( dF.empty() )
    {
        /* most variables' derivatives are just 1 */
        dF = cv::Mat::eye( x_post.rows, x_post.rows, CV_64FC1 );
    }

    /* z = z + v * cos( phi ) */
    dF.at<double>( 0, 2 ) = T * cos( phi );
    dF.at<double>( 0, 3 ) = -T * v * sin( phi );

    /* x = x + v * sin( phi ) */
    dF.at<double>( 1, 2 ) = T * sin( phi );
    dF.at<double>( 1, 3 ) = T * v * cos( phi );

    dF.at<double>( 2, 2 ) = 1.0 - a * T;

    dF.at<double>( 3, 2 ) = T / wb * sin( th );
}

static void init_dynamics( cv::Mat &x_post, cv::Mat &P_post, cv::Mat &dF, cv::Mat &Q, cv::Mat &u )
{
    const double init_vars[4] = { 1.0,
                                  1.0,
                                  20.0 * 20.0,
                                  (1.0 / 180.0 * CV_PI) * (1.0 / 180.0 * CV_PI) };
    //const double proc_noise[4] = { 1.0,
    //                               1.0,
    const double proc_noise[4] = { 5.0,
                                   5.0,
                                   5.0 * 5.0,
                                   (5.0 / 180.0 * CV_PI) * (5.0 / 180.0 * CV_PI) };

    
    x_post = cv::Mat::zeros( 4, 1, CV_64FC1 );

    P_post = cv::Mat::zeros( 4, 4, CV_64FC1 );
    for( int i=0; i<4; i++ )
    {
        P_post.at<double>( i, i ) = init_vars[i];
    }

    f_jacobian( x_post, u, dF );

    Q = cv::Mat::zeros( 4, 4, CV_64FC1 );
    for( int i=0; i<4; i++ )
    {
        Q.at<double>( i, i ) = proc_noise[i];
    }
}

static void obs_to_map( cv::Mat &x_post, std::vector<fm_marker> &markers,
                        cv::Mat &map )
{
    double phi;
    cv::Mat rot;
    cv::Mat vec_zx;
    cv::Mat vec_map;


    phi = x_post.at<double>( 3 );

    rot = ( cv::Mat_<double>( 2, 2 ) <<
            cos( phi ), -sin( phi ),
            sin( phi ),  cos( phi ) );

    map.create( markers.size() * 3, 1, CV_64FC1 );

    vec_zx = x_post( cv::Rect( 0, 0, 1, 2 ) );

    for( int i=0; i<markers.size(); i++ )
    {
        vec_map = map( cv::Rect( 0, i*3, 1, 2 ) );
        vec_map = rot * cv::Mat(markers[i].coord) + vec_zx;
        map.at<double>( i*3+2 ) = markers[i].angle + phi;
    }
}

static void enlarge_mat( cv::Mat &mat, int num_add, double new_diag_val )
{
    cv::Mat tmp_mat;
    int size_org;

    
    size_org = mat.rows;

    tmp_mat = mat;
    mat = cv::Mat::zeros( (size_org + num_add), (size_org + num_add), CV_64FC1 );
    tmp_mat.copyTo( mat( cv::Rect( 0, 0, size_org, size_org ) ) );
    for( int i=0; i<num_add; i++ )
    {
        mat.at<double>( size_org + i, size_org + i ) = new_diag_val;
    }
}

static void update_marker_coordinates( cv::Mat &state, std::vector<int> &matched_obs_idx, std::vector<tm_marker> &markers )
{
    for( int i=0; i<markers.size(); i++ )
    {
        markers[i].pt = state( cv::Rect( 0, (4 + i*3), 1, 2 ) ).clone();
        markers[i].fmarker_idx = matched_obs_idx[i];
    }
}

static void add_to_map( cv::Mat &x_post, cv::Mat &P_post, cv::Mat &dF, cv::Mat &Q,
                        std::vector<tm_marker> &tmarkers,
                        std::vector<fm_marker> &fmarkers )
{
    //const double init_var = 3.0 * 3.0;
    //const double init_var = 5.0 * 5.0;
    const double var_coef = 3.0 * 2500.0;
    const double proc_noise = 0.04;
    //const double proc_noise = 0.09;
    cv::Mat tmp_mat;
    cv::Mat map;
    int size_org;
    int size_new;


    if( fmarkers.size() == 0 )
    {
        return;
    }

    size_org = x_post.rows;
    size_new = size_org + fmarkers.size() * 3;
    
    obs_to_map( x_post, fmarkers, map );
    x_post.resize( size_new );
    map.copyTo( x_post( cv::Rect( 0, size_org, 1, fmarkers.size() * 3 ) ) );

    enlarge_mat( dF, (fmarkers.size() * 3), 1.0 );
    enlarge_mat( Q, (fmarkers.size() * 3), proc_noise );

    for( int i=0; i<fmarkers.size(); i++ )
    {
        tm_marker tmarker;
        tmarker.id = fmarkers[i].id;
        tmarkers.push_back( tmarker );

        double area = cv::contourArea( fmarkers[i].corners );
        double var = var_coef / area;
        var *= var;
        enlarge_mat( P_post, 3, var );
    }
}

static void predict( cv::Mat &x_post, cv::Mat &P_post, cv::Mat &dF, cv::Mat &Q,
                     cv::Mat &x_pre, cv::Mat &P_pre, cv::Mat &u )
{
    f_func( x_post, u, x_pre );
    P_pre = dF * P_post * dF.t() + Q;
}

static void h_func_base( cv::Mat &x_pre, int map_idx, cv::Mat &y )
{
    double phi;
    cv::Mat rot_inv;
    cv::Mat vec_car;
    cv::Mat vec_map;


    phi = x_pre.at<double>( 3 );

    rot_inv = ( cv::Mat_<double>( 2, 2 ) <<
                 cos( phi ), sin( phi ),
                -sin( phi ), cos( phi ) );

    vec_car = x_pre( cv::Rect( 0, 0, 1, 2 ) );
    vec_map = x_pre( cv::Rect( 0, (4 + map_idx*3), 1, 2 ) );

    y = rot_inv * ( vec_map - vec_car );
}

static void h_func( cv::Mat &x_pre, std::vector<int> map_idxs, cv::Mat &y )
{
    double phi;
    cv::Mat rot_inv;
    cv::Mat vec_car;
    cv::Mat vec_map;
    double map_p;


    phi = x_pre.at<double>( 3 );

    rot_inv = ( cv::Mat_<double>( 2, 2 ) <<
                 cos( phi ), sin( phi ),
                -sin( phi ), cos( phi ) );

    vec_car = x_pre( cv::Rect( 0, 0, 1, 2 ) );

    y.create( map_idxs.size() * 3, 1, CV_64FC1 );

    for( int i=0; i<map_idxs.size(); i++ )
    {
        vec_map = x_pre( cv::Rect( 0, (4 + map_idxs[i]*3), 1, 2 ) );
        y( cv::Rect( 0, i*3, 1, 2 ) ) = rot_inv * ( vec_map - vec_car );

        map_p = x_pre.at<double>( 4 + map_idxs[i]*3 + 2 );
        y.at<double>( i*3 + 2 ) = map_p - phi;
    }
}

static void h_jacobian_base( cv::Mat &x_pre, int map_idx, cv::Mat &dH_part )
{
    double z, x, v, phi;
    double map_z, map_x, map_p;


    z = x_pre.at<double>( 0 );
    x = x_pre.at<double>( 1 );
    v = x_pre.at<double>( 2 );
    phi = x_pre.at<double>( 3 );

    map_z = x_pre.at<double>( 4 + map_idx * 3 );
    map_x = x_pre.at<double>( 4 + map_idx * 3 + 1 );
    map_p = x_pre.at<double>( 4 + map_idx * 3 + 2 );

    dH_part = cv::Mat::zeros( 3, x_pre.rows, CV_64FC1 );

    dH_part.at<double>( 0, 0 ) = -cos( phi );
    dH_part.at<double>( 0, 1 ) = -sin( phi );
    dH_part.at<double>( 0, 3 ) = -sin( phi ) * ( map_z - z ) + cos( phi ) * ( map_x - x );
    dH_part.at<double>( 0, 4 + map_idx * 3 ) = cos( phi );
    dH_part.at<double>( 0, 4 + map_idx * 3 + 1 ) = sin( phi );
    dH_part.at<double>( 1, 0 ) = sin( phi );
    dH_part.at<double>( 1, 1 ) = -cos( phi );
    dH_part.at<double>( 1, 3 ) = -cos( phi ) * ( map_z - z ) - sin( phi ) * ( map_x - x );
    dH_part.at<double>( 1, 4 + map_idx * 3 ) = -sin( phi );
    dH_part.at<double>( 1, 4 + map_idx * 3 + 1 ) = cos( phi );
    dH_part.at<double>( 2, 3 ) = -1.0;
    dH_part.at<double>( 2, 4 + map_idx * 3 + 2 ) = 1.0;
}

static inline int num_map( cv::Mat &x )
{
    return ( ( x.rows - 4 ) / 3 );
}

#if 0
static void match( cv::Mat &x_pre, std::vector<fm_marker> &markers,
                   std::vector<int> &matched_obs_idx, std::vector<int> &matched_map_idx )
{
    const double max_match_dist = 30.0;
    std::vector<double> matched_obs_dist;


    matched_obs_idx.resize( num_map( x_pre ), -1 );
    matched_obs_dist.resize( num_map( x_pre ) );
    matched_map_idx.resize( markers.size(), -1 );
    
    for( int i=0; i<num_map( x_pre ); i++ )
    {
        cv::Mat y_part;
        double min_obs_dist;
        int min_obs_idx;


        h_func_base( x_pre, i, y_part );

        min_obs_dist = DBL_MAX;

        for( int j=0; j<markers.size(); j++ )
        {
            double norm;


            norm = cv::norm( y_part - cv::Mat(markers[j].coord) );
            if( norm < min_obs_dist )
            {
                min_obs_dist = norm;
                min_obs_idx = j;
            }
        }

        if( min_obs_dist < max_match_dist )
        {
            int matched_map_to_obs;


            matched_map_to_obs = matched_map_idx[min_obs_idx];

            if( matched_map_to_obs < 0 )
            {
                matched_obs_idx[i] = min_obs_idx;
                matched_obs_dist[i] = min_obs_dist;
                matched_map_idx[min_obs_idx] = i;
            }
            else if( matched_obs_dist[matched_map_to_obs] > min_obs_dist )
            {
                matched_obs_idx[matched_map_to_obs] = -1;

                matched_obs_idx[i] = min_obs_idx;
                matched_obs_dist[i] = min_obs_dist;
                matched_map_idx[min_obs_idx] = i;
            }
        }
    }
}
#else
static void match( std::vector<tm_marker> &tmarkers, std::vector<fm_marker> &fmarkers,
                   std::vector<int> &matched_obs_idx, std::vector<int> &matched_map_idx )
{
    matched_obs_idx.resize( tmarkers.size(), -1 );
    matched_map_idx.resize( fmarkers.size(), -1 );

    for( int i=0; i<(int)tmarkers.size(); i++ )
    {
        for( int j=0; j<(int)fmarkers.size(); j++ )
        {
            if( tmarkers[i].id == fmarkers[j].id )
            {
                matched_obs_idx[i] = j;
                matched_map_idx[j] = i;
                break;
            }
        }
    }
}
#endif

static int prepare_update( cv::Mat &x_pre, std::vector<fm_marker> &markers,
                           std::vector<int> &matched_obs_idx,
                           cv::Mat &y, cv::Mat &dH, cv::Mat &R,
                           cv::Mat &y_pred )
{
    //const double obs_var = 4.0 * 4.0;
    const double obs_var_coef = 3.0 * 2500.0;
    int num_matched_obs;
    std::vector<int> matched_maps;


    num_matched_obs = 0;
    for( int i=0; i<matched_obs_idx.size(); i++ )
    {
        if( matched_obs_idx[i] >= 0 )
        {
            num_matched_obs++;
        }
    }

    if( num_matched_obs == 0 )
    {
        return 1;
    }
    
    y.create( (num_matched_obs * 3), 1, CV_64FC1 );
    dH.create( (num_matched_obs * 3), x_pre.rows, CV_64FC1 );

    R = cv::Mat::zeros( (num_matched_obs * 3), (num_matched_obs * 3), CV_64FC1 );

    for( int map_idx=0, y_idx=0; map_idx<matched_obs_idx.size(); map_idx++ )
    {
        cv::Mat dH_part;
        

        if( matched_obs_idx[map_idx] < 0 )
        {
            continue;
        }

        y.at<double>( y_idx * 3 ) = markers[matched_obs_idx[map_idx]].coord[0];
        y.at<double>( y_idx * 3 + 1 ) = markers[matched_obs_idx[map_idx]].coord[1];
        y.at<double>( y_idx * 3 + 2 ) = markers[matched_obs_idx[map_idx]].angle;

        dH_part = dH( cv::Rect( 0, y_idx * 3, x_pre.rows, 3 ) );
        h_jacobian_base( x_pre, map_idx, dH_part );

        matched_maps.push_back( map_idx );

        double area = cv::contourArea( markers[matched_obs_idx[map_idx]].corners );
        double var = obs_var_coef / area;
        var *= var;
        for( int i = 0; i < 3; i++ )
        {
            R.at<double>( y_idx * 3 + i, y_idx * 3 + i ) = var;
        }

        y_idx++;
    }

    h_func( x_pre, matched_maps, y_pred );

    for( int i=0; i<num_matched_obs; i++ )
    {
        double &p = y.at<double>( i * 3 + 2 );
        double &p_pred = y_pred.at<double>( i * 3 + 2 );


        while( fabs( p - p_pred ) > CV_PI )
        {
            p = (p > p_pred) ? (p - 2.0*CV_PI) : (p + 2.0*CV_PI);
        }
    }

    return 0;
}

static double update( cv::Mat &x_pre, cv::Mat &x_post, cv::Mat &P_pre, cv::Mat &P_post,
                      cv::Mat &y, cv::Mat &y_pred,
                      cv::Mat &dH, cv::Mat &R )
{
    cv::Mat K;
    cv::Mat tmp_inv;
    double inv_cond_num;

    inv_cond_num = cv::invert( (dH * P_pre * dH.t() + R), tmp_inv, cv::DECOMP_SVD );
    
    K = P_pre * dH.t() * tmp_inv;
    x_post = x_pre + K * ( y - y_pred );
    P_post = P_pre - K * dH * P_pre;

    return inv_cond_num;
}

static void add_new_obs( std::vector<int> &matched_obs_idx, std::vector<int> &matched_map_idx,
                         std::vector<fm_marker> &fmarkers,
                         cv::Mat &x_post, cv::Mat &P_post, cv::Mat &dF, cv::Mat &Q,
                         std::vector<tm_marker> &tmarkers )
{
    std::vector<fm_marker> new_fmarkers;


    for( int i=0; i<matched_map_idx.size(); i++ )
    {
        if( matched_map_idx[i] < 0 )
        {
            new_fmarkers.push_back( fmarkers[i] );

            matched_map_idx[i] = (int) matched_obs_idx.size();
            matched_obs_idx.push_back( i );
        }
    }

    add_to_map( x_post, P_post, dF, Q, tmarkers, new_fmarkers );
}

void tm_exec( tm_data &tm, std::vector<fm_marker> &markers, cv::Vec2d &u )
{
    cv::Mat u_mat = cv::Mat( u );
    std::vector<int> matched_obs_idx;
    std::vector<int> matched_map_idx;

    
    if( tm.x_post.empty() )
    {
        init_dynamics( tm.x_post, tm.P_post, tm.dF, tm.Q, u_mat );
        add_to_map( tm.x_post, tm.P_post, tm.dF, tm.Q, tm.markers, markers );

        for( int i=0; i<(int)markers.size(); i++ )
        {
            matched_obs_idx.push_back( i );
            matched_map_idx.push_back( i ); //<-not needed?
        }

        tm.inv_cond_num = -1;
    }
    else
    {
        cv::Mat y, y_pred;


        f_jacobian( tm.x_post, u_mat, tm.dF );
        predict( tm.x_post, tm.P_post, tm.dF, tm.Q,
                 tm.x_pre, tm.P_pre, u_mat );
        
#if 0
        match( tm.x_pre, markers, matched_obs_idx, matched_map_idx );
#else
        match( tm.markers, markers, matched_obs_idx, matched_map_idx );
#endif

        if( prepare_update( tm.x_pre, markers, matched_obs_idx,
                            y, tm.dH, tm.R, y_pred ) )
        {
            tm.x_pre.copyTo( tm.x_post );
            tm.P_pre.copyTo( tm.P_post );
            tm.inv_cond_num = -1;
        }
        else
        {
            tm.inv_cond_num = update( tm.x_pre, tm.x_post, tm.P_pre, tm.P_post,
                                      y, y_pred, tm.dH, tm.R );
        }

        add_new_obs( matched_obs_idx, matched_map_idx, markers, tm.x_post, tm.P_post, tm.dF, tm.Q, tm.markers );
    }

    update_marker_coordinates( tm.x_post, matched_obs_idx, tm.markers );
}
