#include "PlanePawnEvents.h"

PlanePawnEvents::ActuatorsSignal& PlanePawnEvents::getActuatorSignal()
{
    return actuator_signal_;
}