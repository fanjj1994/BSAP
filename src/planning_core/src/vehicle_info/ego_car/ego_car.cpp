#include "ego_car.h"
#include "curve.h"

namespace Planning
{
  EgoCar::EgoCar(const EgoCarStruct &egoCarConfig, const std::string &mapFrame)
  {
    vehicleChildFrame = egoCarConfig.frame_;
    vehicleID = egoCarConfig.id_;
    vehicleLength = egoCarConfig.length_;
    vehicleWidth = egoCarConfig.width_;
    vehicleTheta = egoCarConfig.pose_theta_;
    vehicleVelocity = egoCarConfig.set_speed_;

    vehiclePose.header.frame_id = mapFrame;
    vehiclePose.header.stamp = rclcpp::Clock().now();
    vehiclePose.pose.position.x = egoCarConfig.pose_x_;
    vehiclePose.pose.position.y = egoCarConfig.pose_y_;
    vehiclePose.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, vehicleTheta);
    vehiclePose.pose.orientation.x = qtn.getX();
    vehiclePose.pose.orientation.y = qtn.getY();
    vehiclePose.pose.orientation.z = qtn.getZ();
    vehiclePose.pose.orientation.w = qtn.getW();

    RCLCPP_INFO(rclcpp::get_logger("ego_car"), "EgoCar is initialized, frame: %s", vehicleChildFrame.c_str());
  }

  void EgoCar::vehicleCartesianToFrenet(const base_msgs::msg::Referline &referline)
  {
    CartesianState cartesianState;
    ProjectedPointInfo projectedPoint;
    FrenetState frenetState;
    cartesianState.x = vehiclePose.pose.position.x;
    cartesianState.y = vehiclePose.pose.position.y;
    cartesianState.theta = vehicleTheta;
    cartesianState.speed = vehicleVelocity;
    cartesianState.acceleration = vehicleAcceleration;
    cartesianState.curvature = vehicleKappa;
    Curve::figureOutProjectedPoint(referline, vehiclePose, projectedPoint);
    Curve::CartesianToFrenet(cartesianState, projectedPoint, frenetState);
    s = frenetState.s;
    l = frenetState.l;
    ds_dt = frenetState.ds_dt;
    dl_dt = frenetState.dl_dt;
    dl_ds = frenetState.dl_ds;
    dds_dt = frenetState.dds_dt;
    ddl_dt = frenetState.ddl_dt;
    ddl_ds = frenetState.ddl_ds;
  }

}// namespace Planning