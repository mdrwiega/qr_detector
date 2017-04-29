/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include "qr_detector/detector.h"

namespace qr_detector {

class QrDetectorNodelet : public nodelet::Nodelet
{
public:
    QrDetectorNodelet();
    virtual ~QrDetectorNodelet();

private:
    virtual void onInit();
    void connectCb();
    void disconnectCb();
    void imageCb(const sensor_msgs::ImageConstPtr& image);

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber imgSubscriber;
    ros::Publisher tagsPublisher;
    Detector detector;
};

}
