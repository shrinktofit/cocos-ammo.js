
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "macro.h"

// extern "C"
// {
    // btCollisionObject
    int DLL_EXPORT CollisionObject_new()
    {
        btCollisionObject *o =new btCollisionObject();
        return (int)o;
    }

    bool DLL_EXPORT CollisionObject_isStaticObject(int ptr)
    {
        return ((btCollisionObject *)ptr)->isStaticObject();
    }

    bool DLL_EXPORT CollisionObject_isKinematicObject(int ptr)
    {
        return ((btCollisionObject *)ptr)->isKinematicObject();
    }

    bool DLL_EXPORT CollisionObject_isStaticOrKinematicObject(int ptr)
    {
        return ((btCollisionObject *)ptr)->isStaticOrKinematicObject();
    }

    void DLL_EXPORT CollisionObject_setContactProcessingThreshold(int ptr, int contactProcessingThreshold)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setContactProcessingThreshold(contactProcessingThreshold);
    }

    void DLL_EXPORT CollisionObject_forceActivationState(int ptr, int newState)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->forceActivationState(newState);
    }

    void DLL_EXPORT CollisionObject_activate(int ptr, bool forceActivation)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->activate(forceActivation);
    }

    bool DLL_EXPORT CollisionObject_isActive(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->isActive();
    }

    void DLL_EXPORT CollisionObject_setMaterial(int ptr, float r, float f, float rf, float sf)
    {
        btCollisionObject *o = (btCollisionObject *)ptr;
        o->setRestitution(r);
        o->setFriction(f);
        o->setRollingFriction(rf);
        o->setSpinningFriction(sf);
    }

    int DLL_EXPORT CollisionObject_getCollisionFlags(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getCollisionFlags();
    }

    void DLL_EXPORT CollisionObject_setCollisionFlags(int ptr, int flags)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setCollisionFlags(flags);
    }

    int DLL_EXPORT CollisionObject_getWorldTransform(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return (int)&c0->getWorldTransform();
    }

    void DLL_EXPORT CollisionObject_setCollisionShape(int ptr, int shape)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setCollisionShape((btCollisionShape *)shape);
    }

    int DLL_EXPORT CollisionObject_getCollisionShape(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return (int)c0->getCollisionShape();
    }

    float DLL_EXPORT CollisionObject_getCcdMotionThreshold(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getCcdMotionThreshold();
    }

    void DLL_EXPORT CollisionObject_setCcdMotionThreshold(int ptr, float ccdMotionThreshold)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setCcdMotionThreshold(ccdMotionThreshold);
    }

    float DLL_EXPORT CollisionObject_getCcdSweptSphereRadius(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getCcdSweptSphereRadius();
    }

    void DLL_EXPORT CollisionObject_setCcdSweptSphereRadius(int ptr, float radius)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setCcdSweptSphereRadius(radius);
    }

    int DLL_EXPORT CollisionObject_getUserIndex(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getUserIndex();
    }

    void DLL_EXPORT CollisionObject_setUserIndex(int ptr, int index)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        c0->setUserIndex(index);
    }

    int DLL_EXPORT CollisionObject_isSleeping(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getActivationState() == ISLAND_SLEEPING;
    }

    int DLL_EXPORT CollisionObject_getActivationState(int ptr)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr;
        return c0->getActivationState();
    }

    void DLL_EXPORT CollisionObject_setIgnoreCollisionCheck(int ptr0, int ptr1, bool v)
    {
        btCollisionObject *c0 = (btCollisionObject *)ptr0;
        btCollisionObject *c1 = (btCollisionObject *)ptr1;
        c0->setIgnoreCollisionCheck(c1, v);
    }
// }