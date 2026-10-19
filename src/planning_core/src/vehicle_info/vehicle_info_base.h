#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "base_msgs/msg/referline.hpp"

namespace Planning
{
  class VehicleInfoBase
  {
  public:
    VehicleInfoBase() = default;
    VehicleInfoBase(const VehicleInfoBase&) = delete;
    virtual ~VehicleInfoBase() = default;

    virtual void vehicleCartesianToFrenet(const base_msgs::msg::Referline& referenceline) = 0;

    inline std::string getVehicleChildFrame() const { return vehicleChildFrame; }
    inline uint8 getVehicleID() const { return vehicleID; }
    inline float64 getVehicleLength() const { return vehicleLength; }
    inline float64 getVehicleWidth() const { return vehicleWidth; }
    inline float64 getVehicleTheta() const { return vehicleTheta; }
    inline float64 getVehicleKappa() const { return vehicleKappa; }
    inline float64 getVehicleVelocity() const { return vehicleVelocity; }
    inline float64 getVehicleAcceleration() const { return vehicleAcceleration; }
    inline geometry_msgs::msg::PoseStamped getVehiclePose() const { return vehiclePose; }
    inline float64 getS() const { return s; }
    inline float64 getL() const { return l; }
    inline float64 getDsDt() const { return ds_dt; }
    inline float64 getDlDt() const { return dl_dt; }
    inline float64 getDlDs() const { return dl_ds; }
    inline float64 getDdsDt() const { return dds_dt; }
    inline float64 getDdlDt() const { return ddl_dt; }
    inline float64 getDdlDs() const { return ddl_ds; }

    inline void updateVehiclePose(const geometry_msgs::msg::PoseStamped& currentVehiclePose)
    {
      vehiclePose = currentVehiclePose;
    }

  protected:
    std::string vehicleChildFrame = "";
    uint8 vehicleID = 0U;
    float64 vehicleLength = 0.0;
    float64 vehicleWidth = 0.0;
    geometry_msgs::msg::PoseStamped vehiclePose;
    float64 vehicleTheta = 0.0;
    float64 vehicleKappa = 0.0;
    float64 vehicleVelocity = 0.0;
    float64 vehicleAcceleration = 0.0;
    float64 s = 0.0;
    float64 l = 0.0;
    float64 ds_dt = 0.0;
    float64 dl_dt = 0.0;
    float64 dl_ds = 0.0;
    float64 dds_dt = 0.0;
    float64 ddl_ds = 0.0;
    float64 ddl_dt = 0.0;
  };

} // namespace Planning
#endif // VEHICLE_INFO_BASE_H_
