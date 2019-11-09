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

#ifndef FIND_MARKER_H_
#define FIND_MARKER_H_

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

struct fm_marker
{
    int id;
    std::vector<cv::Point2f> corners;
    cv::Vec2d coord;  // marker coordinates (z,x)
    double angle;
};

struct fm_data
{
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> param;
    double marker_size;

    int max_id;
    std::vector<int> ng_ids;
};

void fm_init( fm_data &fm, int max_id = 31, const std::vector<int> &ng_ids = std::vector<int>() );
void fm_find_marker( const fm_data &fm, const cv::Mat &img_in, std::vector<fm_marker> &markers,
                     const cv::Mat &mat_cam, const cv::Mat &vec_dist, const cv::Mat &xi = cv::Mat() );

#endif
