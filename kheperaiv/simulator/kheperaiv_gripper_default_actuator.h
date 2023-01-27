/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_gripper_default_actuator.h>
 *
 * @author Chandler Garcia & Yasmine Aoua
 */

#ifndef KHEPERAIV_GRIPPER_DEFAULT_ACTUATOR_H
#define KHEPERAIV_GRIPPER_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CKheperaIVGripperDefaultActuator;
}

#include <argos3/plugins/robots/kheperaiv/control_interface/ci_kheperaiv_gripper_actuator.h>
#include <argos3/plugins/simulator/entities/gripper_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CKheperaIVGripperDefaultActuator : public CSimulatedActuator,
                                          public CCI_KheperaIVGripperActuator {

   public:

      CKheperaIVGripperDefaultActuator();

      virtual ~CKheperaIVGripperDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Update();
      virtual void Reset();

      virtual void EnableCheckForObjectGrippedRoutine() {}
      virtual void DisableCheckForObjectGrippedRoutine() {}

   private:

      CGripperEquippedEntity* m_pcGripperEquippedEntity;

   };

}

#endif
