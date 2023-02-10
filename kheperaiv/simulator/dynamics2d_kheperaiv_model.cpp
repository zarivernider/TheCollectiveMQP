/**
 * @file <argos3/plugins/robots/kheperaiv/simulator/dynamics2d_kheperaiv_model.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "dynamics2d_kheperaiv_model.h"
#include "kheperaiv_measures.h"
#include "kheperaiv_turret_entity.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_gripping.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real KHEPERAIV_MASS                = 0.4f; // base khepera mass
   static const Real KHEPERAIV_MASS_W_TURRET       = 0.4f; // TODO: khepera mass with our new base
   static const Real KHEPERAIV_MAX_FORCE           = 1.5f;
   static const Real KHEPERAIV_MAX_TORQUE          = 1.5f;

   static const Real KHEPERAIV_TURRET_MAX_FORCE    = 1.5f; // TODO: add updated ones for the added weight of our new base
   static const Real KHEPERAIV_TURRET_MAX_TORQUE   = 1.5f; // TODO: add updated ones for the added weight of our new base

   enum KHEPERAIV_WHEELS {
      KHEPERAIV_LEFT_WHEEL = 0,
      KHEPERAIV_RIGHT_WHEEL = 1
   };

   enum ETurretModes {
      MODE_OFF,
      MODE_PASSIVE,
      MODE_SPEED_CONTROL,
      MODE_POSITION_CONTROL,
   };

   /****************************************/
   /****************************************/

   CDynamics2DKheperaIVModel::CDynamics2DKheperaIVModel(CDynamics2DEngine& c_engine,
                                                        CKheperaIVEntity& c_entity) :
      CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
      m_cKheperaIVEntity(c_entity),
      m_cWheeledEntity(m_cKheperaIVEntity.GetWheeledEntity()),
      m_cGripperEntity(c_entity.GetGripperEquippedEntity()), // TODO : Make sure this is correctly placed
      m_cDiffSteering(c_engine,
                      KHEPERAIV_MAX_FORCE,
                      KHEPERAIV_MAX_TORQUE,
                      KHEPERAIV_WHEEL_DISTANCE,
                      c_entity.GetConfigurationNode()),
      m_pcGripper(nullptr), // TODO : Do we need this? Pretty sure we don't since we aren't doing anything accurate
      m_pcGrippable(nullptr),
      m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
      /* Create the body with initial position and orientation */
      cpBody* ptBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(KHEPERAIV_MASS,
                                  cpMomentForCircle(KHEPERAIV_MASS,
                                                    0.0f,
                                                    KHEPERAIV_BASE_RADIUS + KHEPERAIV_BASE_RADIUS,
                                                    cpvzero)));
      const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;
      ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      CRadians cXAngle, cYAngle, cZAngle;
      GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
      cpBodySetAngle(ptBody, cZAngle.GetValue());
      /* Create the body shape */
      cpShape* ptShape =
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpCircleShapeNew(ptBody,
                                          KHEPERAIV_BASE_RADIUS,
                                          cpvzero));
      ptShape->e = 0.0; // No elasticity
      ptShape->u = 0.7; // Lots of friction
      /* Constrain the actual base body to follow the diff steering control */
      m_cDiffSteering.AttachTo(ptBody);
      /* Set the body so that the default methods work as expected */
      SetBody(ptBody, KHEPERAIV_BASE_TOP);

      m_unLastTurretMode(m_cKheperaIVEntity.GetTurretEntity().GetMode()) {
      RegisterAnchorMethod<CDynamics2DKheperaIVBotModel>(
         GetEmbodiedEntity().GetOriginAnchor(),
         &CDynamics2DKheperaIVBotModel::UpdateOriginAnchor);
      RegisterAnchorMethod<CDynamics2DKheperaIVBotModel>(
         GetEmbodiedEntity().GetAnchor("turret"), // TODO : Make sure this is good and keeps same name as what we call the other anchor
         &CDynamics2DKheperaIVBotModel::UpdateTurretAnchor);
      RegisterAnchorMethod<CDynamics2DKheperaIVBotModel>(
         GetEmbodiedEntity().GetAnchor("perspective_camera"),
         &CDynamics2DKheperaIVBotModel::UpdatePerspectiveCameraAnchor);
   
      /* Create the gripper body */     // TODO : Check over everything below this in this function
      /* This shape is grippable */
      m_pcGrippable = new CDynamics2DGrippable(GetEmbodiedEntity(),
                                               m_ptBaseShape);
      /* Constrain the actual base body to follow the diff steering control */
      m_cDiffSteering.AttachTo(m_ptActualBaseBody);
      /* Add the body so that the default methods work as expected */
      AddBody(m_ptActualBaseBody, cpvzero, 0, FOOTBOT_HEIGHT);
      /* Create the gripper body */      
      m_ptActualGripperBody =
         cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                        cpBodyNew(m_fMass / 20.0,
                                  cpMomentForCircle(m_fMass,
                                                    0.0f,
                                                    FOOTBOT_RADIUS + FOOTBOT_RADIUS,
                                                    cpvzero)));
      m_ptActualGripperBody->p = cpv(cPosition.GetX(), cPosition.GetY());
      cpBodySetAngle(m_ptActualGripperBody,
                     cZAngle.GetValue() +
                     m_cFootBotEntity.GetTurretEntity().GetRotation().GetValue());
      /* Create the gripper shape */
      cpShape* ptGripperShape = 
         cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                         cpCircleShapeNew(m_ptActualGripperBody,
                                          0.01f,
                                          cpv(FOOTBOT_RADIUS, 0.0f)));
      m_pcGripper = new CDynamics2DGripper(GetDynamics2DEngine(),
                                           m_cGripperEntity,
                                           ptGripperShape);
      /* Constrain the actual gripper body to follow the actual base body */
      m_ptBaseGripperLinearMotion =
         cpSpaceAddConstraint(GetDynamics2DEngine().GetPhysicsSpace(),
                              cpPivotJointNew2(m_ptActualBaseBody,
                                               m_ptActualGripperBody,
                                               cpvzero,
                                               cpvzero));
      m_ptBaseGripperAngularMotion = cpSpaceAddConstraint(GetDynamics2DEngine().GetPhysicsSpace(),
                                                          cpGearJointNew(m_ptActualBaseBody,
                                                                         m_ptActualGripperBody,
                                                                         0.0f,
                                                                         1.0f));
      m_ptBaseGripperAngularMotion->maxBias = 0.0f; /* disable joint correction */
      m_ptBaseGripperAngularMotion->maxForce = KHEPERAIV_MAX_TORQUE; /* limit the dragging torque */
      /* Add the gripper body so that the default methods work as expected */
      AddBody(m_ptActualGripperBody, cpvzero, 0, KHEPERAIV_BASE_TOP);
      /* Switch to active mode if necessary */
      if(m_unLastTurretMode == MODE_SPEED_CONTROL ||
         m_unLastTurretMode == MODE_POSITION_CONTROL) {
         TurretActiveToPassive();
      }
   }

   /****************************************/
   /****************************************/

   CDynamics2DKheperaIVModel::~CDynamics2DKheperaIVModel() {
      m_cDiffSteering.Detach();
      delete m_pcGripper; // TODO : Check over this, should be accurate and it has all the correct variables
      delete m_pcGrippable;
      switch(m_unLastTurretMode) {
         case MODE_OFF:
         case MODE_PASSIVE:
            cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptBaseGripperLinearMotion);
            cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptBaseGripperAngularMotion);
            cpConstraintFree(m_ptBaseGripperLinearMotion);
            cpConstraintFree(m_ptBaseGripperAngularMotion);
            break;
         case MODE_POSITION_CONTROL:
         case MODE_SPEED_CONTROL:
            cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptBaseGripperLinearMotion);
            cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptGripperControlAngularMotion);
            cpConstraintFree(m_ptBaseGripperLinearMotion);
            cpConstraintFree(m_ptGripperControlAngularMotion);
            cpBodyFree(m_ptControlGripperBody);
            break;
      }
      m_cDiffSteering.Detach();
   }

   /****************************************/
   /****************************************/

   // TODO : DO WE NEED THIS Pinci?
   // void CDynamics2DFootBotModel::MoveTo(const CVector3& c_position,
   //                                      const CQuaternion& c_orientation) {
   //    /* Release grippers and grippees */
   //    m_pcGripper->Release();
   //    m_pcGrippable->ReleaseAll();
   //    /* Move robot */
   //    CDynamics2DMultiBodyObjectModel::MoveTo(c_position,
   //                                            c_orientation);
   // }

   /****************************************/
   /****************************************/

   void CDynamics2DKheperaIVModel::Reset() {
      CDynamics2DSingleBodyObjectModel::Reset();
      m_cDiffSteering.Reset();
      /* Release grippers and gripees */
      m_pcGripper->Release();
      m_pcGrippable->ReleaseAll();
      /* Switch to turret passive mode if needed */
      if(m_unLastTurretMode == MODE_SPEED_CONTROL ||
         m_unLastTurretMode == MODE_POSITION_CONTROL) {
         TurretActiveToPassive();
         m_unLastTurretMode = MODE_OFF;
         GetEmbodiedEntity().DisableAnchor("turret");
      }
      /* Reset the rest */ // TODO : Do we need this Multi Body Object Reset?
      // CDynamics2DMultiBodyObjectModel::Reset();
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKheperaIVModel::UpdateFromEntityStatus() {
      /* Do we want to move? */
      if((m_fCurrentWheelVelocity[KHEPERAIV_LEFT_WHEEL] != 0.0f) ||
         (m_fCurrentWheelVelocity[KHEPERAIV_RIGHT_WHEEL] != 0.0f)) {
         m_cDiffSteering.SetWheelVelocity(m_fCurrentWheelVelocity[KHEPERAIV_LEFT_WHEEL],
                                          m_fCurrentWheelVelocity[KHEPERAIV_RIGHT_WHEEL]);
      }
      else {
         /* No, we don't want to move - zero all speeds */
         m_cDiffSteering.Reset();
      }

      /* Update turret structures if the state changed state in the last step */ // TODO Check over this
      if(m_cKheperaIVEntity.GetTurretEntity().GetMode() != m_unLastTurretMode) {
         /* Enable or disable the anchor */
         if(m_cKheperaIVEntity.GetTurretEntity().GetMode() != MODE_OFF) {
            GetEmbodiedEntity().EnableAnchor("turret");
         }
         else {
            GetEmbodiedEntity().DisableAnchor("turret");
         }
         /* Manage the thing like a state machine */
         switch(m_unLastTurretMode) {
            case MODE_OFF:
            case MODE_PASSIVE:
               switch(m_cKheperaIVEntity.GetTurretEntity().GetMode()) {
                  case MODE_POSITION_CONTROL:
                  case MODE_SPEED_CONTROL:
                     TurretPassiveToActive();
                     break;
                  case MODE_OFF:
                  case MODE_PASSIVE:
                     break;
               }
               break;
            case MODE_SPEED_CONTROL:
            case MODE_POSITION_CONTROL:
               switch(m_cKheperaIVEntity.GetTurretEntity().GetMode()) {
                  case MODE_OFF:
                  case MODE_PASSIVE:
                     TurretActiveToPassive();
                     break;
                  case MODE_POSITION_CONTROL:
                  case MODE_SPEED_CONTROL:
                     break;
               }
               break;
         }
         /* Save the current mode for the next time step */
         m_unLastTurretMode = m_cKheperaIVEntity.GetTurretEntity().GetMode();
      }
      /* Update the turret data */
      switch(m_unLastTurretMode) {
         /* Position control mode is implemented using a PD controller */
         case MODE_POSITION_CONTROL: {
            Real fCurRotErr = NormalizedDifference(
               m_cKheperaIVEntity.GetTurretEntity().GetDesiredRotation(),
               NormalizedDifference(
                  CRadians(m_ptActualGripperBody->a),
                  CRadians(m_ptActualBaseBody->a))).GetValue();
            m_ptControlGripperBody->w =
               m_cDiffSteering.GetAngularVelocity() +
               (PD_P_CONSTANT * fCurRotErr +
                PD_D_CONSTANT * (fCurRotErr - m_fPreviousTurretAngleError) * GetDynamics2DEngine().GetInverseSimulationClockTick());
            m_fPreviousTurretAngleError = fCurRotErr;
            break;
         }
            case MODE_SPEED_CONTROL:
            m_ptControlGripperBody->w =
               m_cDiffSteering.GetAngularVelocity() +
               m_cFootBotEntity.GetTurretEntity().GetDesiredRotationSpeed();
            break;
            case MODE_OFF: // TODO : We need  a Mode off I'm guessing
            case MODE_PASSIVE: // We probably need to implement utilizing the force sensor values here
               if(m_cGripperEntity.IsGripping() &&
                  m_cGripperEntity.IsLocked()) {
                  m_ptBaseGripperAngularMotion->maxForce = 0.0001f; /* limit the dragging torque */
               }
               else {
                  m_ptBaseGripperAngularMotion->maxForce = FOOTBOT_MAX_TORQUE; /* limit the dragging torque */
               }
               break;
      }
   }
   /****************************************/
   /****************************************/

}

void CDynamics2DKheperaIVModel::TurretPassiveToActive() { // TODO : Check these
      /* Delete constraints to actual base body */
      cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptBaseGripperAngularMotion);
      cpConstraintFree(m_ptBaseGripperAngularMotion);
      /* Create gripper control body */
      m_ptControlGripperBody = cpBodyNew(INFINITY, INFINITY);
      /* Create angular constraint from gripper control body to gripper actual body */
      m_ptGripperControlAngularMotion = cpSpaceAddConstraint(GetDynamics2DEngine().GetPhysicsSpace(),
                                                             cpGearJointNew(m_ptActualGripperBody,
                                                                            m_ptControlGripperBody,
                                                                            0.0f,
                                                                            1.0f));
      m_ptGripperControlAngularMotion->maxBias = 0.0f; /* disable joint correction */
      m_ptGripperControlAngularMotion->maxForce = KHEPERAIV_MAX_TORQUE; /* limit the dragging torque */
   }

   /****************************************/
   /****************************************/

   void CDynamics2DKheperaIVModel::TurretActiveToPassive() { // TODO : Check these
      /* Delete constraint from actual gripper body to gripper control body */
      cpSpaceRemoveConstraint(GetDynamics2DEngine().GetPhysicsSpace(), m_ptGripperControlAngularMotion);
      cpConstraintFree(m_ptGripperControlAngularMotion);
      /* Delete control body */
      cpBodyFree(m_ptControlGripperBody);
      /* Create constraints from actual gripper body to actual base body */
      m_ptBaseGripperAngularMotion = cpSpaceAddConstraint(GetDynamics2DEngine().GetPhysicsSpace(),
                                                          cpGearJointNew(m_ptActualBaseBody,
                                                                         m_ptActualGripperBody,
                                                                         0.0f,
                                                                         1.0f));
      m_ptBaseGripperAngularMotion->maxBias = 0.0f; /* disable joint correction */
      m_ptBaseGripperAngularMotion->maxForce = KHEPERAIV_MAX_TORQUE; /* limit the dragging torque */ // TODO Need to make sure this is accurate in measures
   }

   /****************************************/
   /****************************************/


   void CDynamics2DKheperaIVModel::UpdateTurretAnchor(SAnchor& s_anchor) { // TODO : Check these
      s_anchor.Position.SetX(m_ptActualGripperBody->p.x);
      s_anchor.Position.SetY(m_ptActualGripperBody->p.y);
      s_anchor.Orientation.FromAngleAxis(CRadians(m_ptActualGripperBody->a), CVector3::Z);
      s_anchor.OffsetOrientation.FromAngleAxis(
         NormalizedDifference(
            CRadians(m_ptActualGripperBody->a),
            CRadians(m_ptActualBaseBody->a)),
         CVector3::Z);
   }

   /****************************************/
   /****************************************/

   // TODO : Ask Pinci about this
   // void CDynamics2DFootBotModel::UpdateOriginAnchor(SAnchor& s_anchor) {
   //    s_anchor.Position.SetX(m_ptActualBaseBody->p.x);
   //    s_anchor.Position.SetY(m_ptActualBaseBody->p.y);
   //    s_anchor.Orientation.FromAngleAxis(CRadians(m_ptActualBaseBody->a), CVector3::Z);
   // }



   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CKheperaIVEntity, CDynamics2DKheperaIVModel); // Unsure what this does

   /****************************************/
   /****************************************/
}