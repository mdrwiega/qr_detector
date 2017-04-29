/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "qr_detector/qr_detector_nodelet.h"

#include "pluginlib/class_list_macros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"

PLUGINLIB_EXPORT_CLASS(qr_detector::QrDetectorNodelet, nodelet::Nodelet);

namespace qr_detector {

QrDetectorNodelet::QrDetectorNodelet()
    : it(nh)
{ }

QrDetectorNodelet::~QrDetectorNodelet()
{
    imgSubscriber.shutdown();
}

void QrDetectorNodelet::onInit()
{
    nh = getNodeHandle();

    tagsPublisher = nh.advertise<std_msgs::String>("qr_codes", 10,
                                                   boost::bind(&QrDetectorNodelet::connectCb, this),
                                                   boost::bind(&QrDetectorNodelet::disconnectCb, this));

    NODELET_INFO_STREAM("Initialising nodelet... [" << nh.getNamespace() << "]");
}

void QrDetectorNodelet::connectCb()
{
    if (!imgSubscriber && tagsPublisher.getNumSubscribers() > 0)
    {
        NODELET_INFO("Connecting to image topic.");
        imgSubscriber = it.subscribe("image", 1, &QrDetectorNodelet::imageCb, this);
    }
}

void QrDetectorNodelet::disconnectCb()
{
    if (tagsPublisher.getNumSubscribers() == 0)
    {
        NODELET_INFO("Unsubscribing from image topic.");
        imgSubscriber.shutdown();
    }
}

void QrDetectorNodelet::imageCb(const sensor_msgs::ImageConstPtr &image)
{
    cv_bridge::CvImageConstPtr cv_image;

    try
    {
        cv_image = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    auto tags = detector.detect(cv_image->image, 10);
    for (auto& tag : tags)
    {
        tagsPublisher.publish(tag.message);
    }
}

}
