/**
 * @file <argos3/plugins/robots/foot-bot/simulator/kheperaiv_turret_encoder_default_sensor.h>
 *
 * @author Chandler Garcia & Yasmine Aoua
 */

#ifndef KHEPERAIV_TURRET_ENCODER_DEFAULT_SENSOR_H
#define KHEPERAIV_TURRET_ENCODER_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CKheperaIVTurretEncoderDefaultSensor;
}

#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_turret_encoder_sensor.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_encoder_default_sensor.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_entity.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CKheperaIVTurretEncoderDefaultSensor : public CCI_KheperaIVTurretEncoderSensor,
                                              public CSimulatedSensor {

   public:

      CKheperaIVTurretEncoderDefaultSensor();

      virtual ~CKheperaIVTurretEncoderDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Update();

      virtual void Reset();

      virtual void Enable();

      virtual void Disable();

   private:

      CKheperaIVTurretEntity* m_pcTurretEntity;

   };

}

#endif
