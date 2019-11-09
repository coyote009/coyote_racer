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

#include "follow_path.h"

int fp_reached_end( const cv::Vec2f &pt_car, follow_path_target &target )
{
    cv::Vec2f vec_path = target.pt_dst - target.pt_src;
    float path_len = cv::norm( vec_path );
    vec_path /= path_len;

    float car_progress = (pt_car - target.pt_src).dot( vec_path );

    if( car_progress > (path_len * 0.8) )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void fp_compute_control( const cv::Vec2f &pt_car, float velocity_car, float angle_car,
                         follow_path_target &target, float throttle_eq_coef,
                         float &throttle, float &steering )
{
    cv::Vec2f vec_path = target.pt_dst - target.pt_src;
    float path_len = cv::norm( vec_path );
    vec_path /= path_len;

    cv::Vec2f vec_normal( -vec_path(1), vec_path(0) );

    cv::Vec2f src_to_car = pt_car - target.pt_src;

    /* compute distance to path */
    float dist_to_path = src_to_car.dot( vec_normal );

    /* compute angle difference */
    float mod_angle_car = angle_car;
    while( fabs(mod_angle_car) > CV_PI )
    {
        mod_angle_car = (mod_angle_car > 0.0) ?
            (mod_angle_car - 2.0*CV_PI) : (mod_angle_car + 2.0*CV_PI);
    }

    float angle_path = atan2( vec_path(1), vec_path(0) );
    float diff_angle = mod_angle_car - angle_path;
    diff_angle = ( diff_angle >  CV_PI ) ? ( diff_angle - (2*CV_PI) ) : diff_angle;
    diff_angle = ( diff_angle < -CV_PI ) ? ( diff_angle + (2*CV_PI) ) : diff_angle;

    /* compute velocity difference */
    float car_progress = src_to_car.dot( vec_path );
    float car_progress_ratio = car_progress / path_len;
    float target_velocity = target.velocity_src +
                            car_progress_ratio * ( target.velocity_dst - target.velocity_src );
    float diff_velocity = velocity_car - target_velocity;

#if 1
    /* compute distance rate for differential control */
    float dist_rate = velocity_car * ( cos( mod_angle_car ) * vec_normal( 0 ) +
                                       sin( mod_angle_car ) * vec_normal( 1 ) );

    const float gain_velocity = -0.01;
    //const float gain_dist = -0.1;
    //const float gain_dist_rate = -0.1;
    //const float gain_angle = -1.0;
    const float gain_dist = -0.03;
    const float gain_dist_rate = -0.03;
    const float gain_angle = -0.3;

    throttle = throttle_eq_coef * target_velocity + gain_velocity * diff_velocity;
    steering = gain_dist * dist_to_path + gain_dist_rate * dist_rate + gain_angle * diff_angle;
#else
    /* compute control */
    //const float gain_velocity = -0.06; /*!!!!!!!! Make these variables parameters !!!!!!!!*/
    //const float gain_dist = -0.1;
    //const float gain_angle = -1.0;

    const float gain_velocity = -0.01; /*!!!!!!!! Make these variables parameters !!!!!!!!*/
    const float gain_dist = -0.2;
    const float gain_angle = -1.0;

    throttle = throttle_eq_coef * target_velocity + gain_velocity * diff_velocity;
    steering = gain_dist * dist_to_path + gain_angle * diff_angle;
#endif

    throttle = ( throttle >  1.0 ) ?  1.0 : throttle;
    throttle = ( throttle < -1.0 ) ? -1.0 : throttle;
    steering = ( steering >  1.0 ) ?  1.0 : steering;
    steering = ( steering < -1.0 ) ? -1.0 : steering;
}
