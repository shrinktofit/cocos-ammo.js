
#ifndef CC_DISCRETE_DYNAMICS_WORLD_H
#define CC_DISCRETE_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btScalar.h"
#include "ccOverlapFilterCallback.h"

namespace cc
{

ATTRIBUTE_ALIGNED16(class)
ccDiscreteDynamicsWorld : public btDiscreteDynamicsWorld {
private:
  ccOverlapFilterCallback m_overlapFilterCallback;

public:
  BT_DECLARE_ALIGNED_ALLOCATOR();

  ccDiscreteDynamicsWorld(btDispatcher * dispatcher,
                          btBroadphaseInterface * pairCache,
                          btConstraintSolver * constraintSolver,
                          btCollisionConfiguration * collisionConfiguration);

  void setAllowSleep(bool v) { gDisableDeactivation = !v; }

  void setDeactivationTime(btScalar v) { gDeactivationTime = v; }

  void setNarrowPhaseMethod(int v) {
    m_dispatchInfo.m_enableSatConvex = v == 1;
  }

  void setAllowCcdPenetration(btScalar v) {
    m_dispatchInfo.m_allowedCcdPenetration = v;
  }

  /// apply gravity, call this once per timestep
  virtual void applyGravity();
  virtual void createPredictiveContacts(btScalar timeStep);
};

}

#endif // CC_DISCRETE_DYNAMICS_WORLD_H