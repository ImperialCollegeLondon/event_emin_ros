#include "event_emin_ros/event_emin_ros.h"

namespace event_emin_ros
{
template <typename Derived>
void
getCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& msg,
              EventEMin::MatrixBase<Derived>& camParams, unsigned int& height,
              unsigned int& width)
{
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      camParams(i, j) = msg->K[i * 3 + j];
    }
  }
  camParams(1, 1) = -camParams(1, 1);

  ROS_INFO("[Camera Info] camera parameters: %f %f %f %f", camParams(0, 0),
           camParams(1, 1), camParams(0, 2), camParams(1, 2));

  height = msg->height;
  width = msg->width;
}

template <typename M>
GlobalMotion<M>::GlobalMotion(ros::NodeHandle& nh, ros::NodeHandle& nhPriv)
    : gotCameraInfo_(false),
      firstEventMessage_(true),
      iEvents_(0),
      height_(0),
      width_(0)
{
  if (ros::names::remap("camera_info") == "camera_info")
  {
    ROS_WARN("[Configuration] topic 'camera_info' has not been remapped");
  }
  if (ros::names::remap("events") == "events")
  {
    ROS_WARN("[Configuration] topic 'events' has not been remapped");
  }

  std::string ns = ros::this_node::getNamespace();
  if (ns == "/")
  {
    ns = "/event_emin_ros";
  }

  nhPriv.param<int>("nEvents", nEvents_, 10000);
  nhPriv.param<int>("wSize", wSize_, 4);
  ROS_INFO("[Configuration] number of events: %d", nEvents_);
  ROS_INFO("[Configuration] spatial window size: %d", wSize_);

  cameraInfoSub_ = nh.subscribe("camera_info", 1,
                                &GlobalMotion<M>::cameraInfoCallback, this);
  ros::Rate r(5);
  for (int i = 0; !gotCameraInfo_ && !ros::isShuttingDown(); ++i)
  {
    ROS_INFO("[Configuration] waiting for camera info %d", i);
    ros::spinOnce();
    r.sleep();
  }

  const Vector<T, NDims> scale(camParams_.diagonal().template head<NDims>());

  dispersion_ = new Dispersion(
      camParams_, scale, typename Dispersion::Params(T(1.0e-6), 10, wSize_),
      nEvents_, {width_, height_});
  varsMsg_.vars.resize(NVars);

  eventsSub_ =
      nh.subscribe("events", 100, &GlobalMotion<M>::eventsCallback, this);
  varsPub_ = nh.advertise<Vars>(ns + "/vars", 100);
}

template <typename M>
GlobalMotion<M>::~GlobalMotion(void)
{
  shutdown();
}

template <typename M>
void
GlobalMotion<M>::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  getCameraInfo(msg, camParams_, height_, width_);

  ROS_INFO("[Camera Info] received camera info... shutting down subscriber");
  cameraInfoSub_.shutdown();
  gotCameraInfo_ = true;
}

template <typename M>
void
GlobalMotion<M>::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (msg->events.size() > 0)
  {
    if (width_ != msg->width || height_ != msg->height)
    {
      ROS_ERROR("[Events] size mismatch: %dx%d - %dx%d", width_, height_,
                msg->width, msg->height);
      return;
    }

    if (firstEventMessage_)
    {
      tsInit_ = ros::Time::now();
      tsMessageInit_ = msg->events[0].ts;
      tsRef_ = tsMessageInit_;
      firstEventMessage_ = false;
    }

    const ros::Duration tsDelay((ros::Time::now() - tsInit_) -
                                (msg->events[0].ts - tsMessageInit_));
    const T inc = T(1.0) / std::min(computeExp(-T(5.0 * tsDelay.toSec())), 1.0);
    // ROS_INFO("%f %f", inc, tsDelay.toSec());
    dispersion_->setInc(inc);

    const unprojectEvent<T, NDims> unprojEvent;
    Vector<T, NDims> c, ct;
    for (T i = T(0.0); i < static_cast<T>(msg->events.size()); i += inc)
    {
      const int ii = std::floor(i);
      c(0) = msg->events[ii].x;
      c(1) = msg->events[ii].y;
      unprojEvent(camParams_, c, ct);
      dispersion_->run(ct, (msg->events[ii].ts - tsMessageInit_).toSec());
      ++iEvents_;
      if (nEvents_ <= iEvents_)
      {
        varsMsg_.header.stamp = tsRef_;
        for (int l = 0; l < NVars; ++l)
        {
          varsMsg_.vars[l] = dispersion_->vars()(l);
        }
        varsPub_.publish(varsMsg_);

        tsRef_ = msg->events[ii].ts;
        iEvents_ = 0;
      }
    }
  }
}

template <typename M>
void
GlobalMotion<M>::shutdown(void)
{
  if (dispersion_ != nullptr)
  {
    delete dispersion_;
  }
  eventsSub_.shutdown();
  varsPub_.shutdown();
}

template class GlobalMotion<IncrementalAffinity<float> >;
template class GlobalMotion<IncrementalIsometry<float> >;
template class GlobalMotion<IncrementalRotation<float> >;
template class GlobalMotion<IncrementalSimilarity<float> >;
template class GlobalMotion<IncrementalTranslation2D<float> >;
}  // namespace event_emin_ros
