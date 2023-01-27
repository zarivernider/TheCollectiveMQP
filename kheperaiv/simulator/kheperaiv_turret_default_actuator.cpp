/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_default_actuator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kheperaiv_turret_default_actuator.h"

namespace argos {

	 const Real RPM_TO_RADIANS_PER_SEC = ARGOS_PI / 30.0f;

   /****************************************/
   /****************************************/

   CKheperaIVTurretDefaultActuator::CKheperaivTurretDefaultActuator() :
      m_pcTurretEntity(nullptr),
      m_unDesiredMode(CKheperaivTurretEntity::MODE_OFF) {}

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcTurretEntity = &(c_entity.GetComponent<CKheperaIVTurretEntity>("turret"));
      m_pcTurretEntity->Enable();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::SetRotation(const CRadians& c_angle) {
      m_pcTurretEntity->SetDesiredRotation(c_angle);
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::SetRotationSpeed(SInt32 n_speed_pulses) {
      m_pcTurretEntity->SetDesiredRotationSpeed(RPM_TO_RADIANS_PER_SEC * n_speed_pulses);
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::SetMode(ETurretModes e_mode) {
      m_unDesiredMode = e_mode;
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::Update() {
      m_pcTurretEntity->SetMode(m_unDesiredMode);
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretDefaultActuator::Reset() {
      m_unDesiredMode = CKheperaIVTurretEntity::MODE_OFF;
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CKheperaIVTurretDefaultActuator,
                     "kheperaiv_turret", "default",
                     "Carlo Pinciroli [ilpincy@gmail.com]",
                     "1.0",
                     "The kheperaiv turret actuator.",
                     "This actuator controls the kheperaiv turret. For a complete\n"
                     "description of its usage, refer to the ci_kheperaiv_turret_actuator\n"
                     "file.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <kheperaiv_turret implementation=\"default\" />\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Usable"
      );

}
