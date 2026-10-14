#ifndef EGO_CAR_H_
#define EGO_CAR_H_

#include "vehicle_info_base.h"
#include "config_reader.h"

namespace Planning
{

  class EgoCar : public VehicleInfoBase
  {
  public:
    EgoCar(const EgoCarStruct &egoCarConfig, const std::string &mapFrame);
    EgoCar(const EgoCar &) = delete;
    EgoCar &operator= (const EgoCar &) = delete;
    ~EgoCar() = default;
  };
} //namespace Planning
#endif // EGO_CAR_H_
