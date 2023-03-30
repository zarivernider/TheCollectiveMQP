/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_turret_encoder_default_sensor.cpp>
 *
 * @author Chandler Garcia & Yasmine Aoua
 */

#include "kheperaiv_turret_encoder_default_sensor.h"
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKheperaIVTurretEncoderDefaultSensor::CKheperaIVTurretEncoderDefaultSensor() :
      m_pcTurretEntity(nullptr) {}

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEncoderDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcTurretEntity = &(c_entity.GetComponent<CKheperaIVTurretEntity>("turret"));

      /* sensor is enabled by default */
      Enable();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEncoderDefaultSensor::Update() {
      /* sensor is disabled--nothing to do */
      if (IsDisabled()) {
        return;
      }
      m_cRotation = m_pcTurretEntity->GetRotation();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEncoderDefaultSensor::Enable() {
     m_pcTurretEntity->Enable();
     CCI_Sensor::Enable();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEncoderDefaultSensor::Disable() {
     m_pcTurretEntity->Disable();
     CCI_Sensor::Disable();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEncoderDefaultSensor::Reset() {
      m_cRotation = CRadians::ZERO;
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CKheperaIVTurretEncoderDefaultSensor,
                   "kheperaiv_turret_encoder", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The foot-bot turret encoder sensor.",
                   "This sensor accesses the foot-bot turret encoder. For a complete\n"
                   "description of its usage, refer to the ci_kheperaiv_turret_encoder_sensor\n"
                   "file.\n\n"

                   "This sensor is enabled by default.\n\n"

                   "REQUIRED XML CONFIGURATION\n\n"

                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <kheperaiv_turret_encoder implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None for the time being.\n",
                   "Usable"
      );

}
