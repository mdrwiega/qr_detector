/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "opencv/cv.h"
#include "zbar.h"

namespace qr_detector {

struct Tag
{
    std::string message;
    std::vector<cv::Point> polygon;
};

using Tags = std::vector<Tag>;

/**
 * QR codes detector
 */
class Detector
{
public:
    Detector();

    /**
     * Detects tags in image.
     *
     * @param[in] image - image with qr tags
     * @param[in] timeout - max time for tags detection in ms
     */
    Tags detect(const cv::Mat& image, size_t timeout = 100);

private:
    zbar::ImageScanner scanner;
};

}
