#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btMotionState.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "macro.h"

// extern "C"
// {
    extern int DLL_EXPORT EmptyShape_static();

    // btRigidBodyConstructionInfo
    int DLL_EXPORT RigidBodyConstructionInfo_static(float mass, int motionState, int collisionShape, int localInertia)
    {
        return (int)new btRigidBody::btRigidBodyConstructionInfo(
            mass, (btMotionState *)motionState, (btCollisionShape *)collisionShape, *(btVector3 *)localInertia);
    }

    // btRigidBody
    int DLL_EXPORT RigidBody_new(float mass, int ms)
    {
        btVector3 localInertia{1.6666666269302368, 1.6666666269302368, 1.6666666269302368};
        if (mass == 0)
            localInertia.setValue(0, 0, 0);
        btRigidBody::btRigidBodyConstructionInfo constructionInfo{
            mass, (btMotionState *)ms, (btCollisionShape *)EmptyShape_static(), localInertia};

        btRigidBody *body = new btRigidBody(constructionInfo);
        return (int)body;
    }

    void DLL_EXPORT RigidBody_setCenterOfMassTransform(int ptr, btTransform xform)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setCenterOfMassTransform(xform);
    }

    void DLL_EXPORT RigidBody_setSleepingThresholds(int ptr, float linear, float angular)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setSleepingThresholds(linear, angular);
    }

    float DLL_EXPORT RigidBody_getLinearSleepingThreshold(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return rigid->getLinearSleepingThreshold();
    }

    float DLL_EXPORT RigidBody_getAngularSleepingThreshold(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return rigid->getAngularSleepingThreshold();
    }

    void DLL_EXPORT RigidBody_setDamping(int ptr, float lin_damping, float ang_damping)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setDamping(lin_damping, ang_damping);
    }

    void DLL_EXPORT RigidBody_setMass(int rb_ptr, float mass)
    {
        btRigidBody *rb = (btRigidBody *)rb_ptr;
        btVector3 inertia{1.6666666269302368, 1.6666666269302368, 1.6666666269302368};
        btCollisionShape *cs = (btCollisionShape *)rb->getCollisionShape();
        if (cs->isCompound())
        {
            btCompoundShape *CollisionShape_com = (btCompoundShape *)cs;
            if (CollisionShape_com->getNumChildShapes() > 0)
                CollisionShape_com->calculateLocalInertia(mass, inertia);
        }
        else
        {
            cs->calculateLocalInertia(mass, inertia);
        }
        rb->setMassProps(mass, inertia);
    }

    void DLL_EXPORT RigidBody_setMassProps(int rb_ptr, float mass, int v3_ptr)
    {
        btRigidBody *rb = (btRigidBody *)rb_ptr;
        btVector3 *v = (btVector3 *)v3_ptr;
        rb->setMassProps(mass, *v);
    }

    void DLL_EXPORT RigidBody_setLinearFactor(int ptr, int factor)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setLinearFactor(*(btVector3 *)factor);
    }

    void DLL_EXPORT RigidBody_setAngularFactor(int ptr, int factor)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setAngularFactor(*(btVector3 *)factor);
    }

    int DLL_EXPORT RigidBody_getLinearVelocity(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return (int)&rigid->getLinearVelocity();
    }

    int DLL_EXPORT RigidBody_getAngularVelocity(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return (int)&rigid->getAngularVelocity();
    }

    void DLL_EXPORT RigidBody_setLinearVelocity(int ptr, int lin_vel)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setLinearVelocity(*(btVector3 *)lin_vel);
    }

    void DLL_EXPORT RigidBody_setAngularVelocity(int ptr, int ang_vel)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setAngularVelocity(*(btVector3 *)ang_vel);
    }

    int DLL_EXPORT RigidBody_getGravity(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return (int)&rigid->getGravity();
    }

    void DLL_EXPORT RigidBody_setGravity(int ptr, int acceleration)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setGravity(*(btVector3 *)acceleration);
    }

    void DLL_EXPORT RigidBody_applyTorque(int ptr, int torque)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyTorque(*(btVector3 *)torque);
    }

    void DLL_EXPORT RigidBody_applyForce(int ptr, int force, int rel_pos)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyForce(*(btVector3 *)force, *(btVector3 *)rel_pos);
    }

    void DLL_EXPORT RigidBody_applyCentralForce(int ptr, int force)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyCentralForce(*(btVector3 *)force);
    }

    void DLL_EXPORT RigidBody_applyTorqueImpulse(int ptr, int torque)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyTorqueImpulse(*(btVector3 *)torque);
    }

    void DLL_EXPORT RigidBody_applyImpulse(int ptr, int impulse, int rel_pos)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyImpulse(*(btVector3 *)impulse, *(btVector3 *)rel_pos);
    }

    void DLL_EXPORT RigidBody_applyCentralImpulse(int ptr, int impulse)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->applyCentralImpulse(*(btVector3 *)impulse);
    }

    void DLL_EXPORT RigidBody_updateInertiaTensor(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->updateInertiaTensor();
    }

    int DLL_EXPORT RigidBody_getTotalForce(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return (int)&rigid->getTotalForce();
    }

    int DLL_EXPORT RigidBody_getTotalTorque(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return (int)&rigid->getTotalTorque();
    }

    int DLL_EXPORT RigidBody_getFlags(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return rigid->getFlags();
    }

    void DLL_EXPORT RigidBody_setFlags(int ptr, int flags)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->setFlags(flags);
    }

    void DLL_EXPORT RigidBody_clearForces(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->clearForces();
    }

    bool DLL_EXPORT RigidBody_wantsSleeping(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        return rigid->wantsSleeping();
    }

    void DLL_EXPORT RigidBody_clearState(int ptr)
    {
        btRigidBody *rigid = (btRigidBody *)ptr;
        rigid->clearState();
    }

    int DLL_EXPORT RigidBody_getMotionState(int ptr)
    {
        btRigidBody *colObj = (btRigidBody *)ptr;
        return (int)colObj->getMotionState();
    }
// }
