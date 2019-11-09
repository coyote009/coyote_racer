/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2015, Baisheng Lai (laibaisheng@gmail.com), Zhejiang University,
// all rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

/**
 * This module was accepted as a GSoC 2015 project for OpenCV, authored by
 * Baisheng Lai, mentored by Bo Li.
 *
 * The omnidirectional camera in this module is denoted by the catadioptric
 * model. Please refer to Mei's paper for details of the camera model:
 *
 *      C. Mei and P. Rives, "Single view point omnidirectional camera
 *      calibration from planar grids", in ICRA 2007.
 *
 * The implementation of the calibration part is based on Li's calibration
 * toolbox:
 *
 *     B. Li, L. Heng, K. Kevin  and M. Pollefeys, "A Multiple-Camera System
 *     Calibration Toolbox Using A Feature Descriptor-Based Calibration
 *     Pattern", in IROS 2013.
 */

/*
  Modified by coyote009
  
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

void undistort_points( cv::InputArray distorted, cv::OutputArray undistorted,
                       cv::InputArray K, cv::InputArray D, cv::InputArray xi, cv::InputArray R)
{
    CV_Assert(distorted.type() == CV_64FC2 || distorted.type() == CV_32FC2);
    CV_Assert(R.empty() || (!R.empty() && (R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3)
        && (R.depth() == CV_64F || R.depth() == CV_32F)));
    CV_Assert((D.depth() == CV_64F || D.depth() == CV_32F) && D.total() == 4);
    CV_Assert(K.size() == cv::Size(3, 3) && (K.depth() == CV_64F || K.depth() == CV_32F));
    CV_Assert(xi.total() == 1 && (xi.depth() == CV_64F || xi.depth() == CV_32F));

    if( distorted.depth() == CV_32F )
    {
        undistorted.create(distorted.size(), CV_32FC3);
    }
    else
    {
        undistorted.create(distorted.size(), CV_64FC3);
    }

    cv::Vec2d f, c;
    double s = 0.0;
    if (K.depth() == CV_32F)
    {
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0,0), camMat(1,1));
        c = cv::Vec2f(camMat(0,2), camMat(1,2));
        s = (double)camMat(0,1);
    }
    else if (K.depth() == CV_64F)
    {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0,0), camMat(1,1));
        c = cv::Vec2d(camMat(0,2), camMat(1,2));
        s = camMat(0,1);
    }

    cv::Vec4d kp = D.depth()==CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>():(cv::Vec4d)*D.getMat().ptr<cv::Vec4d>();
    cv::Vec2d k = cv::Vec2d(kp[0], kp[1]);
    cv::Vec2d p = cv::Vec2d(kp[2], kp[3]);

    double _xi = xi.depth() == CV_32F ? (double)*xi.getMat().ptr<float>() : *xi.getMat().ptr<double>();
    cv::Matx33d RR = cv::Matx33d::eye();
    // R is om
    if(!R.empty() && R.total()*R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        cv::Rodrigues(rvec, RR);
    }
    else if (!R.empty() && R.size() == cv::Size(3,3))
    {
        R.getMat().convertTo(RR, CV_64F);
    }

    const cv::Vec2d *srcd = distorted.getMat().ptr<cv::Vec2d>();
    const cv::Vec2f *srcf = distorted.getMat().ptr<cv::Vec2f>();

    cv::Vec3d *dstd = undistorted.getMat().ptr<cv::Vec3d>();
    cv::Vec3f *dstf = undistorted.getMat().ptr<cv::Vec3f>();

    int n = (int)distorted.total();
    for (int i = 0; i < n; i++)
    {
        cv::Vec2d pi = distorted.depth() == CV_32F ? (cv::Vec2d)srcf[i]:(cv::Vec2d)srcd[i];    // image point
        cv::Vec2d pp((pi[0]*f[1]-c[0]*f[1]-s*(pi[1]-c[1]))/(f[0]*f[1]), (pi[1]-c[1])/f[1]); //plane
        cv::Vec2d pu = pp;    // points without distortion

        // remove distortion iteratively
        for (int j = 0; j < 20; j++)
        {
            double r2 = pu[0]*pu[0] + pu[1]*pu[1];
            double r4 = r2*r2;
            pu[0] = (pp[0] - 2*p[0]*pu[0]*pu[1] - p[1]*(r2+2*pu[0]*pu[0])) / (1 + k[0]*r2 + k[1]*r4);
            pu[1] = (pp[1] - 2*p[1]*pu[0]*pu[1] - p[0]*(r2+2*pu[1]*pu[1])) / (1 + k[0]*r2 + k[1]*r4);
        }

        // project to unit sphere
        double r2 = pu[0]*pu[0] + pu[1]*pu[1];
        double a = (r2 + 1);
        double b = 2*_xi*r2;
        double cc = r2*_xi*_xi-1;
        double Zs = (-b + sqrt(b*b - 4*a*cc))/(2*a);
        cv::Vec3d Xw = cv::Vec3d(pu[0]*(Zs + _xi), pu[1]*(Zs +_xi), Zs);

        // rotate
        Xw = RR * Xw;

        // project back to sphere
        cv::Vec3d Xs = Xw / cv::norm(Xw);

        if (undistorted.depth() == CV_32F)
        {
            dstf[i] = cv::Vec3f((float)Xs[0], (float)Xs[1], (float)Xs[2]);
        }
        else if (undistorted.depth() == CV_64F)
        {
            dstd[i] = cv::Vec3d(Xs[0], Xs[1], Xs[2]);
        }
    }
}
