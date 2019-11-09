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

#ifndef FOLLOW_PATH_H_
#define FOLLOW_PATH_H_

#include <opencv2/opencv.hpp>

struct follow_path_target
{
    cv::Vec2f pt_src;
    cv::Vec2f pt_dst;
    float velocity_src;
    float velocity_dst;
};

int fp_reached_end( const cv::Vec2f &pt_car, follow_path_target &target );
void fp_compute_control( const cv::Vec2f &pt_car, float velocity_car, float angle_car,
                         follow_path_target &target, float throttle_eq_coef,
                         float &throttle, float &steering );

#endif
