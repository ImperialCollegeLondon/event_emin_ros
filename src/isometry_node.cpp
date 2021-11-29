#include "event_emin_ros/event_emin_ros.h"

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "event_emin_ros", ros::init_options::AnonymousName);
  ros::NodeHandle nh, nhPriv("~");
  event_emin_ros::GlobalMotion<EventEMin::IncrementalIsometry<float> >
      globalMotion(nh, nhPriv);
  ros::spin();
  return 0;
}
