#include "tp_base.h"
#include "curve.h"

namespace Planning
{
  TP::TP(const uint8 id)
  {
    vehicleInfoConfigReader = std::make_unique<ConfigReader>();
    vehicleInfoConfigReader->readVehiclesConfig();
    vehicleInfoConfigReader->readPNCMapConfig();
    const EgoCarStruct &tpConfig = vehicleInfoConfigReader->getVehiclePairs().at(id);
    vehicleChildFrame = tpConfig.frame_;
    vehicleID = tpConfig.id_;
    vehicleLength = tpConfig.length_;
    vehicleWidth = tpConfig.width_;
    vehicleTheta = tpConfig.pose_theta_;
    vehicleVelocity = tpConfig.set_speed_;

    vehiclePose.header.frame_id = vehicleInfoConfigReader->getPNCMap().frame_;
    vehiclePose.header.stamp = rclcpp::Clock().now();
    vehiclePose.pose.position.x = tpConfig.pose_x_;
    vehiclePose.pose.position.y = tpConfig.pose_y_;
    vehiclePose.pose.position.z = 0.0;

    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, vehicleTheta);
    vehiclePose.pose.orientation.x = qtn.getX();
    vehiclePose.pose.orientation.y = qtn.getY();
    vehiclePose.pose.orientation.z = qtn.getZ();
    vehiclePose.pose.orientation.w = qtn.getW();

    RCLCPP_INFO(rclcpp::get_logger("tp"), "TP %d is initialized, frame: %s", vehicleID, vehicleChildFrame.c_str());
  }

  void TP::vehicleCartesianToFrenet(const base_msgs::msg::Referline &referline)
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