
#include "LinearMath/btMotionState.h"
#include "btBulletDynamicsCommon.h"
#include "extensions/btCharacterController.h"

#include "macro.h"

enum EBulletType
{
    EBulletTypeVec3 = 0,
    EBulletTypeQuat,
    EBulletTypeTransform,
    EBulletTypeMotionState,
    EBulletTypeCollisionObject,
    EBulletTypeCollisionShape,
    EBulletTypeCharacterController,
    EBulletTypeStridingMeshInterface,
    EBulletTypeTriangleMesh,
    EBulletTypeCollisionDispatcher,
    EBulletTypeDbvtBroadPhase,
    EBulletTypeSequentialImpulseConstraintSolver,
    EBulletTypeCollisionWorld,
    EBulletTypeTypedConstraint,
    EBulletTypeDebugDraw
};

#define SAFE_DELETE(p)     \
        if (p) {           \
            delete (p);    \
            (p) = nullptr; \
        }                  \
    
// extern "C"
// {
    void DLL_EXPORT _safe_delete(int ptr, int bulletType)
    {
        switch (bulletType) {
            case EBulletTypeVec3: {
                btVector3 *p = (btVector3 *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeQuat: {
                btQuaternion *p = (btQuaternion *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeTransform: {
                btTransform *p = (btTransform *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeMotionState: {
                btMotionState *p = (btMotionState *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeCollisionObject: {
                btCollisionObject *p = (btCollisionObject *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeCollisionShape: {
                btCollisionShape *p = (btCollisionShape *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeStridingMeshInterface: {
                btStridingMeshInterface *p = (btStridingMeshInterface *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeTriangleMesh: {
                btTriangleMesh *p = (btTriangleMesh *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeCollisionDispatcher: {
                btCollisionDispatcher *p = (btCollisionDispatcher *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeDbvtBroadPhase: {
                btDbvtBroadphase *p = (btDbvtBroadphase *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeSequentialImpulseConstraintSolver: {
                btSequentialImpulseConstraintSolver *p = (btSequentialImpulseConstraintSolver *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeCollisionWorld: {
                btCollisionWorld *p = (btCollisionWorld *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeTypedConstraint: {
                btTypedConstraint *p = (btTypedConstraint *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeCharacterController: {
                btCharacterController *p = (btCharacterController *)ptr;
                SAFE_DELETE(p);
                break;
            }
            case EBulletTypeDebugDraw: {
                btIDebugDraw *p = (btIDebugDraw *)ptr;
                SAFE_DELETE(p);
                break;
            }
            default:
                break;
        }
    }

  
// }