
#include "ccDiscreteDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "ccConvexCastCallBack.h"

namespace cc
{

ccDiscreteDynamicsWorld::ccDiscreteDynamicsWorld(
    btDispatcher *dispatcher, btBroadphaseInterface *pairCache,
    btConstraintSolver *constraintSolver,
    btCollisionConfiguration *collisionConfiguration)
    : btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver,
                              collisionConfiguration) {
  // gDeactivationTime = btScalar(1);
  m_dispatchInfo.m_allowedCcdPenetration = btScalar(0.01);
  getPairCache()->setOverlapFilterCallback(&m_overlapFilterCallback);
  getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
}

void ccDiscreteDynamicsWorld::applyGravity() {
  ///@todo: iterate over awake simulation islands!
  for (int i = 0; i < m_nonStaticRigidBodies.size(); i++) {
    btRigidBody *body = m_nonStaticRigidBodies[i];
    // compute unconstraint velocity for dynamics
    if (body->isActive() && !body->isKinematicObject()) {
      if (body->getLinearDamping() == 1.) {
        (const_cast<btVector3 &>(body->getTotalForce())).setZero();
      } else {
        body->applyGravity();
      }
      if (body->getAngularDamping() == 1.) {
        (const_cast<btVector3 &>(body->getTotalTorque())).setZero();
      }
    }
  }
}

void ccDiscreteDynamicsWorld::createPredictiveContacts(btScalar timeStep) {
  BT_PROFILE("createPredictiveContacts");
  releasePredictiveContacts();

  int numBodies = m_nonStaticRigidBodies.size();
  if (numBodies > 0) {
    btTransform predictedTrans;
    for (int i = 0; i < numBodies; i++) {
      btRigidBody *body = m_nonStaticRigidBodies[i];
      body->setHitFraction(1.f);
      if (body->isActive() && (!body->isStaticOrKinematicObject())) {
        body->predictIntegratedTransform(timeStep, predictedTrans);
        const btTransform &transform = body->getWorldTransform();
        btScalar squareMotion = (predictedTrans.getOrigin() - transform.getOrigin()).length2();

        if (getDispatchInfo().m_useContinuous && body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion) {
          BT_PROFILE("predictive convexSweepTest");
          if (body->getCollisionShape()->isConvex()) {
            ccNotMeClosestConvexResultCallback sweepResults(body, this, transform.getOrigin(), predictedTrans.getOrigin());

            // btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
            btSphereShape tmpSphere(body->getCcdSweptSphereRadius());

            sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
            sweepResults.m_collisionFilterMask = body->getBroadphaseProxy()->m_collisionFilterMask;
            btTransform modifiedPredictedTrans = predictedTrans;
            modifiedPredictedTrans.setBasis(transform.getBasis());

            convexSweepTest(&tmpSphere, transform, modifiedPredictedTrans, sweepResults);

            if (sweepResults.hasHit()) {
              btVector3 distVec = (predictedTrans.getOrigin() - transform.getOrigin()) * sweepResults.m_closestHitFraction;
              btScalar distance = distVec.dot(-sweepResults.m_hitNormalWorld);

              btPersistentManifold *manifold = m_dispatcher1->getNewManifold(body, sweepResults.m_hitCollisionObject);
              btMutexLock(&m_predictiveManifoldsMutex);
              m_predictiveManifolds.push_back(manifold);
              btMutexUnlock(&m_predictiveManifoldsMutex);

              btVector3 worldPointB = transform.getOrigin() + distVec;
              btVector3 localPointB = sweepResults.m_hitCollisionObject->getWorldTransform().inverse() * worldPointB;

              btManifoldPoint newPoint(btVector3(0, 0, 0), localPointB, sweepResults.m_hitNormalWorld, distance);
              newPoint.m_shape0 = body->getCollisionShape();
              newPoint.m_shape1 = sweepResults.m_hit;
              int index = manifold->addManifoldPoint(newPoint, true);
              btManifoldPoint &pt = manifold->getContactPoint(index);
              pt.m_combinedRestitution = 0;
              pt.m_combinedFriction = btManifoldResult::calculateCombinedFriction(body, sweepResults.m_hitCollisionObject);
              pt.m_positionWorldOnA = transform.getOrigin();
              pt.m_positionWorldOnB = worldPointB;
            }
          }
        }
      }
    }
  }
}

}