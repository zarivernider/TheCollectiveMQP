/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/dynamics2d_kheperaiv_model.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef DYNAMICS2D_KHEPERAIV_MODEL_H
#define DYNAMICS2D_KHEPERAIV_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DKheperaIVModel; // Ask Pinci if this should be here
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>

namespace argos {

   class CDynamics2DKheperaIVModel : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DKheperaIVModel(CDynamics2DEngine& c_engine,
                              CKheperaIVEntity& c_entity);
      virtual ~CDynamics2DKheperaIVModel();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

      // void UpdateOriginAnchor(SAnchor& s_anchor); // TODO : Do we need this? Probably not?

      void UpdateTurretAnchor(SAnchor& s_anchor);

      inline cpBody* GetActualGripperBody() {
         return m_ptActualGripperBody;
      }

      inline const cpBody* GetActualGripperBody() const {
         return m_ptActualGripperBody;
      }

   private:

      CKheperaIVEntity& m_cKheperaIVEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

      void TurretPassiveToActive();
      void TurretActiveToPassive();

      CDynamics2DGripper*                    m_pcGripper;
      CDynamics2DGrippable*                  m_pcGrippable;
      
      cpFloat  m_fMass;
      cpShape* m_ptBaseShape;
      cpBody*  m_ptActualBaseBody;
      cpBody*  m_ptActualGripperBody;
      cpBody*  m_ptControlGripperBody;
      cpConstraint* m_ptGripperControlAngularMotion;
      cpConstraint* m_ptBaseGripperLinearMotion;
      cpConstraint* m_ptBaseGripperAngularMotion;

      CGripperEquippedEntity& m_cGripperEntity;

      UInt8 m_unLastTurretMode;
      Real m_fPreviousTurretAngleError;

   };

}

#endif
