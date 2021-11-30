#ifndef EVENT_EMIN_ROS_H
#define EVENT_EMIN_ROS_H

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <string>

#include "EventEMin.h"
#include "event_emin_ros/Vars.h"

namespace event_emin_ros
{
using namespace EventEMin;

template <typename M>
class GlobalMotion
{
 public:
  typedef M Model;
  typedef typename Model::T T;
  typedef IncrementalPotential<Model> Dispersion;

  static constexpr int NDims = Model::NDims, NVars = Model::NVars;

 private:
  ros::Subscriber cameraInfoSub_;
  ros::Subscriber eventsSub_;

  ros::Publisher varsPub_;

  bool gotCameraInfo_, firstEventMessage_;
  ros::Time tsInit_, tsMessageInit_;
  ros::Time tsRef_;

  int nEvents_, iEvents_;
  int wSize_;

  unsigned int height_, width_;

  Matrix<T, 3, 3> camParams_;

 protected:
  Dispersion* dispersion_;

  Vars varsMsg_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GlobalMotion(ros::NodeHandle& nh, ros::NodeHandle& nhPriv);
  ~GlobalMotion(void);

 protected:
  void
  cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void
  eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

 private:
  void
  shutdown(void);
};
}  // namespace event_emin_ros

#endif  // EVENT_EMIN_ROS_H
