/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_default_actuator.h>
 *
 * @author Chandler Garcia & Yasmine Aoua
 */

#ifndef KHEPERAIV_TURRET_DEFAULT_ACTUATOR_H
#define KHEPERAIV_TURRET_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CKheperaIVTurretDefaultActuator;
}

#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_turret_actuator.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CKheperaIVTurretDefaultActuator : public CSimulatedActuator,
                                         public CCI_KheperaIVTurretActuator {

   public:

      CKheperaIVTurretDefaultActuator();
      virtual ~CKheperaIVTurretDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void SetRotation(const CRadians& c_angle);
      virtual void SetRotationSpeed(SInt32 n_speed_pulses);
      virtual void SetMode(ETurretModes e_mode);

      virtual void Update();
      virtual void Reset();

   private:

      CKheperaIVTurretEntity* m_pcTurretEntity;
      UInt32 m_unDesiredMode;

   };

}

#endif
