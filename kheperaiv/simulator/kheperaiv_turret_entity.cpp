/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/KheperaIV_turret_entity.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kheperaiv_turret_entity.h"
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/simulator/physics_engine/physics_engine.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKheperaIVTurretEntity::CKheperaIVTurretEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent),
      m_psAnchor(nullptr) {
      Reset();
      Disable();
   }
   
   /****************************************/
   /****************************************/

   CKheperaIVTurretEntity::CKheperaIVTurretEntity(CComposableEntity* pc_parent,
                                              const std::string& str_id,
                                              SAnchor& s_anchor) :
      CEntity(pc_parent, str_id),
      m_psAnchor(&s_anchor) {
      Reset();
      Disable();
   }
   
   /****************************************/
   /****************************************/

   void CKheperaIVTurretEntity::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void KheperaIVTurretEntity::Reset() {
      m_unMode = MODE_OFF;
      m_cDesRot = CRadians::ZERO;
      m_cOldRot = CRadians::ZERO;
      m_fDesRotSpeed = 0.0;
      m_fCurRotSpeed = 0.0;
      if (m_psAnchor) {
         m_psAnchor->OffsetOrientation = CQuaternion();
      }
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEntity::Update() {
      /* Calculate rotation speed */
      CRadians cZAngle, cYAngle, cXAngle;
      m_psAnchor->OffsetOrientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      m_fCurRotSpeed =
         NormalizedDifference(cZAngle, m_cOldRot).GetValue() *
         CPhysicsEngine::GetInverseSimulationClockTick();
      /* Save rotation for next time */
      m_cOldRot = cZAngle;
   }

   /****************************************/
   /****************************************/

   CRadians CKheperaIVTurretEntity::GetRotation() const {
      CRadians cZAngle, cYAngle, cXAngle;
      m_psAnchor->OffsetOrientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      return cZAngle;
   }

   /****************************************/
   /****************************************/

   Real CKheperaIVTurretEntity::GetRotationSpeed() const {
      return m_fCurRotSpeed;
   }

   /****************************************/
   /****************************************/

   const CRadians& CKheperaIVTurretEntity::GetDesiredRotation() const {
      return m_cDesRot;
   }

   /****************************************/
   /****************************************/

   Real CKheperaIVTurretEntity::GetDesiredRotationSpeed() const {
      return m_fDesRotSpeed;
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEntity::SetDesiredRotation(const CRadians& c_rotation) {
      m_cDesRot = c_rotation;
      m_cDesRot.SignedNormalize();
   }

   /****************************************/
   /****************************************/

   void CKheperaIVTurretEntity::SetDesiredRotationSpeed(Real f_speed) {
      m_fDesRotSpeed = f_speed;
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CKheperaIVTurretEntity);

   /****************************************/
   /****************************************/

}
