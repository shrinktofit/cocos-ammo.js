
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "macro.h"
#include <malloc.h>

namespace cc
{
typedef btAlignedObjectArray<int> ccIntArray;
typedef btAlignedObjectArray<btScalar> ccScalarArray;
typedef btAlignedObjectArray<btVector3> ccVector3Array;
typedef btAlignedObjectArray<const btCollisionObject *> ccCollisionObjectArray;
}; // namespace cc

// extern "C"
// {
    // memory

    int DLL_EXPORT _malloc(int bytes)
    {
        return (int)malloc(bytes);
    }

    void DLL_EXPORT _free(int ptr)
    {
        free((void *)ptr);
    }

    float DLL_EXPORT _read_f32(int ptr)
    {
        return *((float *)ptr);
    }

    void DLL_EXPORT _write_f32(int ptr, float v)
    {
        *(float *)ptr = v;
    }

    // btVector3

    int DLL_EXPORT Vec3_new(float x, float y, float z)
    {
        return (int)new btVector3(x, y, z);
    }

    void DLL_EXPORT Vec3_set(int ptr, float x, float y, float z)
    {
        btVector3 *vec3 = (btVector3 *)ptr;
        vec3->setValue(x, y, z);
    }

    float DLL_EXPORT Vec3_x(int ptr)
    {
        btVector3 *vec3 = (btVector3 *)ptr;
        return vec3->x();
    }

    float DLL_EXPORT Vec3_y(int ptr)
    {
        btVector3 *vec3 = (btVector3 *)ptr;
        return vec3->y();
    }

    float DLL_EXPORT Vec3_z(int ptr)
    {
        btVector3 *vec3 = (btVector3 *)ptr;
        return vec3->z();
    }

    // btQuaternion

    int DLL_EXPORT Quat_new(float x, float y, float z, float w)
    {
        return (int)new btQuaternion(x, y, z, w);
    }

    void DLL_EXPORT Quat_set(int ptr, float x, float y, float z, float w)
    {
        btQuaternion *qua = (btQuaternion *)ptr;
        qua->setValue(x, y, z, w);
    }

    float DLL_EXPORT Quat_x(int ptr)
    {
        btQuaternion *qua = (btQuaternion *)ptr;
        return qua->x();
    }

    float DLL_EXPORT Quat_y(int ptr)
    {
        btQuaternion *qua = (btQuaternion *)ptr;
        return qua->y();
    }

    float DLL_EXPORT Quat_z(int ptr)
    {
        btQuaternion *qua = (btQuaternion *)ptr;
        return qua->z();
    }

    float DLL_EXPORT Quat_w(int ptr)
    {
        btQuaternion *qua = (btQuaternion *)ptr;
        return qua->w();
    }

    // btTransform

    int DLL_EXPORT Transform_new()
    {
        btTransform *t = new btTransform();
        t->setIdentity();
        return (int)t;
    }

    void DLL_EXPORT Transform_setIdentity(int ptr)
    {
        ((btTransform *)ptr)->setIdentity();
    }

    void DLL_EXPORT Transform_setOrigin(int ptr, int origin)
    {
        btTransform *trans = (btTransform *)ptr;
        trans->setOrigin(*(btVector3 *)origin);
    }

    void DLL_EXPORT Transform_setRotation(int ptr, int q)
    {
        btTransform *trans = (btTransform *)ptr;
        trans->setRotation(*(btQuaternion *)q);
    }

    int DLL_EXPORT Transform_getOrigin(int ptr)
    {
        btTransform *trans = (btTransform *)ptr;
        return (int)&trans->getOrigin();
    }

    int DLL_EXPORT Transform_getRotation(int ptr, int ptr2)
    {
        btTransform *trans = (btTransform *)ptr;
        btQuaternion *quat = (btQuaternion *)ptr2;
        trans->getBasis().getRotation(*quat);
        return ptr2;
    }

    // ccVector3Array

    int DLL_EXPORT Vec3_array_at(int ptr, int n)
    {
        cc::ccVector3Array *vec3Array = (cc::ccVector3Array *)ptr;
        return (int)&vec3Array->at(n);
    }

    // ccIntArray

    int DLL_EXPORT int_array_size(int ptr)
    {
        cc::ccIntArray *intArray = (cc::ccIntArray *)ptr;
        return (int)intArray->size();
    }

    int DLL_EXPORT int_array_at(int ptr, int n)
    {
        cc::ccIntArray *intArray = (cc::ccIntArray *)ptr;
        return (int)intArray->at(n);
    }
// }
