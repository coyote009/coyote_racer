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

#ifndef TRACK_MARKER_H_
#define TRACK_MARKER_H_

#include <opencv2/opencv.hpp>
#include "find_marker.h"

struct tm_marker
{
    int id;
    cv::Vec2d pt;
    int fmarker_idx;
};

struct tm_data
{
    /*
      x_t+1 = f( x_t, u_t ) + w
      y_t   = h( x_t ) + v

      w ~ N(0,Q)
      v ~ N(0,R)
      x ~ N(x,P)

      x = [ z; x; v; phi; m0z; m0x; m0p; ... ]  *m0p is the rotation angle of the marker
      y = [ m0zc; m0xc; m0pc; ... ] <= local (car) coord

      | m0zc | = |  cos(phi) sin(phi) || m0z - z |
      | m0xc |   | -sin(phi) cos(phi) || m0x - x |
      | m0pc | = m0p - phi

      conversely

      | m0z | = | cos(phi) -sin(phi) || m0zc | + | z |
      | m0x |   | sin(phi)  cos(phi) || m0xc |   | x |
      | m0p | = m0pc + phi

      <<dynamics>>
      z_dot = v * cos( phi )
      x_dot = v * sin( phi )
      v_dot = -a * v + b * E
      phi_dot = 1/wb * v * sin( th )

      <<discrete dyanmics>>
      z_t+1 = z_t + T * v_t * cos( phi_t )
      x_t+1 = x_t + T * v_t * sin( phi_t )
      v_t+1 = ( 1 - a*T ) * v_t + b*T * E_t
      phi_t+1 = phi_t + T/wb * v_t * sin( th_t )

      <<jacobian>>
      1  0  T * cos( phi )     -T * v * sin( phi )
      0  1  T * sin( phi )      T * v * cos( phi )
      0  0  1 - a * T           0
      0  0  T/wb * sin( th )    1
     */
    
    cv::Mat x_pre;
    cv::Mat x_post;
    cv::Mat P_pre;
    cv::Mat P_post;

    cv::Mat dF;
    cv::Mat dH;

    cv::Mat Q;
    cv::Mat R;

    std::vector<tm_marker> markers;

    double inv_cond_num;
};

void tm_exec( tm_data &tm, std::vector<fm_marker> &markers, cv::Vec2d &u );

#endif
