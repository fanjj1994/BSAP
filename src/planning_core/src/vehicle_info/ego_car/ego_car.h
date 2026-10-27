#ifndef EGO_CAR_H_
#define EGO_CAR_H_

#include "vehicle_info_base.h"
#include "config_reader.h"

namespace Planning
{
  // 自车
  class EgoCar : public VehicleInfoBase
  {
  public:
    EgoCar(const EgoCarStruct &egoCarConfig, const std::string &mapFrame);
    EgoCar(const EgoCar &) = delete;
    EgoCar &operator= (const EgoCar &) = delete;
    ~EgoCar() = default;

    // cartesian to frenet
    void vehicleCartesianToFrenet(const base_msgs::msg::Referline &referline) override;
  };
} //namespace Planning
#endif // EGO_CAR_H_
