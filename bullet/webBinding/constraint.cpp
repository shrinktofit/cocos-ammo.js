
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btFixedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "macro.h"

// extern "C"
// {

    int DLL_EXPORT TypedConstraint_getFixedBody()
    {
        return int(&btTypedConstraint::getFixedBody());
    }

    float DLL_EXPORT TypedConstraint_getDbgDrawSize(int ptr)
    {
        btTypedConstraint *c = (btTypedConstraint *)ptr;
        return c->getDbgDrawSize();
    }

    void DLL_EXPORT TypedConstraint_setDbgDrawSize(int ptr, float size)
    {
        btTypedConstraint *c = (btTypedConstraint *)ptr;
        c->setDbgDrawSize(size);
    }

    int DLL_EXPORT HingeConstraint_new(int ptr0, int ptr1, int ptr2, int ptr3)
    {
        btRigidBody *rb0 = (btRigidBody *)ptr0;
        btRigidBody *rb1 = (btRigidBody *)ptr1;
        btTransform *t0 = (btTransform *)ptr2;
        btTransform *t1 = (btTransform *)ptr3;
        btHingeConstraint *c = new btHingeConstraint(*rb0, *rb1, *t0, *t1);
        return (int)c;
    }

    void DLL_EXPORT HingeConstraint_setFrames(int ptr, int ptr0, int ptr1)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        btTransform *t0 = (btTransform *)ptr0;
        btTransform *t1 = (btTransform *)ptr1;
        c->setFrames(*t0, *t1);
    }

    void DLL_EXPORT HingeConstraint_setLimit(int ptr, float low, float high, float softness = 0.9f, float biasFactor = 0.3f, float relaxationFactor = 1.0f)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->setLimit(low, high, softness, biasFactor, relaxationFactor);
    }

    void DLL_EXPORT HingeConstraint_setAngularOnly(int ptr, bool angularOnly)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->setAngularOnly(angularOnly);
    }

    void DLL_EXPORT HingeConstraint_enableMotor(int ptr, bool enableMotor)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->enableMotor(enableMotor);
    }

    void DLL_EXPORT HingeConstraint_setMotorVelocity(int ptr, float targetVelocity)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->setMotorTargetVelocity(targetVelocity);
    }
    
    void DLL_EXPORT HingeConstraint_setMaxMotorImpulse(int ptr, float maxImpulse)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->setMaxMotorImpulse(maxImpulse);
    }

    void DLL_EXPORT HingeConstraint_setMotorTarget(int ptr, float targetAngle, float dt)
    {
        btHingeConstraint *c = (btHingeConstraint *)ptr;
        c->setMotorTarget(targetAngle, dt);
    }

    int DLL_EXPORT P2PConstraint_new(int ptr0, int ptr1, int ptr2, int ptr3)
    {
        btRigidBody *rb0 = (btRigidBody *)ptr0;
        btRigidBody *rb1 = (btRigidBody *)ptr1;
        btVector3 *t0 = (btVector3 *)ptr2;
        btVector3 *t1 = (btVector3 *)ptr3;
        btPoint2PointConstraint *c = new btPoint2PointConstraint(*rb0, *rb1, *t0, *t1);
        return (int)c;
    }

    void DLL_EXPORT P2PConstraint_setPivotA(int ptr, int ptr0)
    {
        btPoint2PointConstraint *c = (btPoint2PointConstraint *)ptr;
        c->setPivotA(*(btVector3 *)ptr0);
    }

    void DLL_EXPORT P2PConstraint_setPivotB(int ptr, int ptr0)
    {
        btPoint2PointConstraint *c = (btPoint2PointConstraint *)ptr;
        c->setPivotB(*(btVector3 *)ptr0);
    }

    int DLL_EXPORT FixedConstraint_new(int ptr0, int ptr1, int ptr2, int ptr3)
    {
        btRigidBody *rb0 = (btRigidBody *)ptr0;
        btRigidBody *rb1 = (btRigidBody *)ptr1;
        btTransform *t0 = (btTransform *)ptr2;
        btTransform *t1 = (btTransform *)ptr3;
        btFixedConstraint *c = new btFixedConstraint(*rb0, *rb1, *t0, *t1);
        return (int)c;
    }

    void DLL_EXPORT FixedConstraint_setFrames(int ptr, int ptr0, int ptr1)
    {
        btFixedConstraint *c = (btFixedConstraint *)ptr;
        btTransform *t0 = (btTransform *)ptr0;
        btTransform *t1 = (btTransform *)ptr1;
        c->setFrames(*t0, *t1);
    }

    void DLL_EXPORT TypedConstraint_setMaxImpulseThreshold (int ptr, float maxImpulse) {
        btTypedConstraint *c = (btTypedConstraint *)ptr;
        c->setBreakingImpulseThreshold((btScalar) maxImpulse);
    }

    int DLL_EXPORT Generic6DofSpring2Constraint_new(int ptr0, int ptr1, int ptr2, int ptr3, int rotateOrder)
    {
        btRigidBody *rb0 = (btRigidBody *)ptr0;
        btRigidBody *rb1 = (btRigidBody *)ptr1;
        btTransform *t0 = (btTransform *)ptr2;
        btTransform *t1 = (btTransform *)ptr3;
        btGeneric6DofSpring2Constraint *c = new btGeneric6DofSpring2Constraint(*rb0, *rb1, *t0, *t1, (RotateOrder)rotateOrder);
        return (int)c;
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setFrames(int ptr, int ptr0, int ptr1)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        btTransform *t0 = (btTransform *)ptr0;
        btTransform *t1 = (btTransform *)ptr1;
        c->setFrames(*t0, *t1);
    }

    void  DLL_EXPORT Generic6DofSpring2Constraint_setLimit(int ptr, int index, float lo, float hi)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setLimit(index, lo, hi);
    }

    // motor
    void DLL_EXPORT Generic6DofSpring2Constraint_enableMotor(int ptr, int index, bool onOff)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->enableMotor(index, onOff);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setMaxMotorForce(int ptr, int index, float force)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setMaxMotorForce(index, force);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setTargetVelocity(int ptr, int index, float velocity)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setTargetVelocity(index, velocity);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setServo(int ptr, int index, bool onOff)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setServo(index, onOff);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setServoTarget(int ptr, int index, float target)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setServoTarget(index, target);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_enableSpring(int ptr, int index, bool onOff)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->enableSpring(index, onOff);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setStiffness(int ptr, int index, float stiffness)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setStiffness(index, stiffness);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setDamping(int ptr, int index, float damping)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setDamping(index, damping);
    }

    void DLL_EXPORT Generic6DofSpring2Constraint_setBounce(int ptr, int index, float bounce)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setBounce(index, bounce);
    }
    
    void DLL_EXPORT Generic6DofSpring2Constraint_setEquilibriumPoint(int ptr, int index, float val)
    {
        btGeneric6DofSpring2Constraint *c = (btGeneric6DofSpring2Constraint *)ptr;
        c->setEquilibriumPoint(index, val);
    }
    
    
// }
