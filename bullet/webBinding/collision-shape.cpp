
#include "extensions/ccCompoundShape.h"
#include "extensions/ccMaterial.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "LinearMath/btVector3.h"
#include "btBulletCollisionCommon.h"
#include "macro.h"

// extern "C"
// {
    using cc::ccMaterial;
    using cc::ccCompoundShape;

    // btCollisionShape

    void DLL_EXPORT CollisionShape_setMaterial(int ptr, int ptr1)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        ccMaterial *mat = (ccMaterial *)ptr1;
        shape->setMaterial(mat);
    }

    int DLL_EXPORT CollisionShape_getMaterial(int ptr)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        return (int)shape->getMaterial();
    }

    void DLL_EXPORT CollisionShape_setUserPointer(int ptr, int ptr1)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        shape->setUserPointerAsInt(ptr1);
    }

    int DLL_EXPORT CollisionShape_getUserPointer(int ptr)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        return shape->getUserPointerAsInt();
    }

    int DLL_EXPORT CollisionShape_getLocalScaling(int ptr)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        return (int)&shape->getLocalScaling();
    }

    void DLL_EXPORT CollisionShape_setLocalScaling(int ptr, int scaling)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        shape->setLocalScaling(*(btVector3 *)scaling);
    }

    void DLL_EXPORT CollisionShape_calculateLocalInertia(int ptr, float mass, int inertia)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        shape->calculateLocalInertia(mass, *(btVector3 *)inertia);
    }

    bool DLL_EXPORT CollisionShape_isCompound(int ptr)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        return shape->isCompound();
    }

    void DLL_EXPORT CollisionShape_getAabb(int ptr, int t, int v0, int v1)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        shape->getAabb(*(btTransform *)t, *(btVector3 *)v0, *(btVector3 *)v1);
    }

    float DLL_EXPORT CollisionShape_getLocalBoundingSphere(int ptr)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        return shape->getLocalBoundingSphere();
    }

    void DLL_EXPORT CollisionShape_setMargin(int ptr, float margin)
    {
        btCollisionShape *shape = (btCollisionShape *)ptr;
        shape->setMargin(margin);
    }

    // btBoxShape
    int DLL_EXPORT BoxShape_new(int boxHalfExtents)
    {
        return (int)new btBoxShape(*(btVector3 *)boxHalfExtents);
    }

    void DLL_EXPORT BoxShape_setUnscaledHalfExtents(int ptr, int hf)
    {
        btBoxShape *box = (btBoxShape *)ptr;
        box->setUnscaledHalfExtents(*(btVector3 *)hf);
    }

    // btSphereShape
    int DLL_EXPORT SphereShape_new(float radius)
    {
        return (int)new btSphereShape(radius);
    }

    void DLL_EXPORT SphereShape_setUnscaledRadius(int ptr, float radius)
    {
        btSphereShape *sphere = (btSphereShape *)ptr;
        sphere->setUnscaledRadius(radius);
    }

    // btCapsuleShape
    int DLL_EXPORT CapsuleShape_new(float radius, float height)
    {
        return (int)new btCapsuleShape(radius, height);
    }

    void DLL_EXPORT CapsuleShape_updateProp(int ptr, float r, float g, int d)
    {
        btCapsuleShape *cs = (btCapsuleShape *)ptr;
        cs->updateProp(r, g, d);
    }

    // btConvexInternalShape
    int DLL_EXPORT ConvexInternalShape_getImplicitShapeDimensions(int ptr)
    {
        btConvexInternalShape *s = (btConvexInternalShape *)ptr;
        return (int)(&(s->getImplicitShapeDimensions()));
    }

    // btCylinderShape
    int DLL_EXPORT CylinderShape_new(int halfExtents)
    {
        return (int)new btCylinderShape(*(btVector3 *)halfExtents);
    }

    void DLL_EXPORT CylinderShape_updateProp(int ptr, float r, float g, int d)
    {
        btCylinderShape *cs = (btCylinderShape *)ptr;
        cs->updateProp(r, g, d);
    }

    // btConeShape
    int DLL_EXPORT ConeShape_new(float radius, float height)
    {
        return (int)new btConeShape(radius, height);
    }

    void DLL_EXPORT ConeShape_setRadius(int ptr, float radius)
    {
        ((btConeShape *)ptr)->setRadius(radius);
    }

    void DLL_EXPORT ConeShape_setHeight(int ptr, float height)
    {
        ((btConeShape *)ptr)->setHeight(height);
    }

    void DLL_EXPORT ConeShape_setConeUpIndex(int ptr, int index)
    {
        ((btConeShape *)ptr)->setConeUpIndex(index);
    }

    // btStaticPlaneShape
    int DLL_EXPORT StaticPlaneShape_new(int planeNormal, float planeConstant)
    {
        return (int)new btStaticPlaneShape(*(btVector3 *)planeNormal, planeConstant);
    }

    void DLL_EXPORT StaticPlaneShape_setPlaneConstant(int ptr, float constant)
    {
        btStaticPlaneShape *plane = (btStaticPlaneShape *)ptr;
        plane->setPlaneConstant(constant);
    }

    int DLL_EXPORT StaticPlaneShape_getPlaneNormal(int ptr)
    {
        btStaticPlaneShape *plane = (btStaticPlaneShape *)ptr;
        return int(&plane->getPlaneNormal());
    }

    // btCompoundShape
    int DLL_EXPORT ccCompoundShape_new()
    {
        return (int)new ccCompoundShape();
    }

    int DLL_EXPORT CompoundShape_getNumChildShapes(int ptr)
    {
        btCompoundShape *compShape = (btCompoundShape *)ptr;
        return compShape->getNumChildShapes();
    }

    void DLL_EXPORT CompoundShape_addChildShape(int ptr, int localTransform, int shape)
    {
        btCompoundShape *compShape = (btCompoundShape *)ptr;
        compShape->addChildShape(*(btTransform *)localTransform, (btCollisionShape *)shape);
    }

    void DLL_EXPORT CompoundShape_removeChildShape(int ptr, int shape)
    {
        btCompoundShape *compShape = (btCompoundShape *)ptr;
        compShape->removeChildShape((btCollisionShape *)shape);
    }

    void DLL_EXPORT CompoundShape_removeChildShapeByIndex(int ptr, int childShapeIndex)
    {
        btCompoundShape *compShape = (btCompoundShape *)ptr;
        compShape->removeChildShapeByIndex(childShapeIndex);
    }

    int DLL_EXPORT CompoundShape_getChildShape(int ptr, int index)
    {
        btCompoundShape *compShape = (btCompoundShape *)ptr;
        return (int)compShape->getChildShape(index);
    }

    void DLL_EXPORT CompoundShape_updateChildTransform(int ptr, int ptr0, int newChildTransform,
                                                       bool shouldRecalculateLocalAabb)
    {
        ccCompoundShape *compShape = (ccCompoundShape *)ptr;
        btCollisionShape *childShape = (btCollisionShape *)ptr0;
        compShape->updateChildTransform(childShape, *(btTransform *)newChildTransform, shouldRecalculateLocalAabb);
    }

    // btTriangleMesh
    int DLL_EXPORT TriangleMesh_new()
    {
        return (int)new btTriangleMesh();
    }

    void DLL_EXPORT TriangleMesh_addTriangle(int ptr, int vertex1, int vertex2, int vertex3,
                                             bool removeDuplicateVertices)
    {
        btTriangleMesh *mesh = (btTriangleMesh *)ptr;
        mesh->addTriangle(*(btVector3 *)vertex1, *(btVector3 *)vertex2, *(btVector3 *)vertex3, removeDuplicateVertices);
    }

    // btBvhTriangleMeshShape
    int DLL_EXPORT BvhTriangleMeshShape_new(int ptr, bool useQuantizedAabbCompression, bool buildBvh = true)
    {
        btStridingMeshInterface *mifPtr = (btStridingMeshInterface *)ptr;
        return (int)new btBvhTriangleMeshShape(mifPtr, useQuantizedAabbCompression, buildBvh);
    }

    int DLL_EXPORT BvhTriangleMeshShape_getOptimizedBvh(int ptr)
    {
        btBvhTriangleMeshShape *meshptr = (btBvhTriangleMeshShape *)ptr;
        return (int)meshptr->getOptimizedBvh();
    }

    void DLL_EXPORT BvhTriangleMeshShape_setOptimizedBvh(int ptr, int ptr1, float scaleX, float scaleY, float scaleZ)
    {
        btBvhTriangleMeshShape *meshptr = (btBvhTriangleMeshShape *)ptr;
        btOptimizedBvh *bvhptr = (btOptimizedBvh *)ptr1;
        meshptr->setOptimizedBvh(bvhptr, btVector3( scaleX, scaleY, scaleZ));
    }

    int DLL_EXPORT ScaledBvhTriangleMeshShape_new(int ptr, float scaleX, float scaleY, float scaleZ)
    {
        btBvhTriangleMeshShape *meshptr = (btBvhTriangleMeshShape *)ptr;
        return (int)new btScaledBvhTriangleMeshShape(meshptr, btVector3( scaleX, scaleY, scaleZ));
    }

    int DLL_EXPORT ConvexTriangleMeshShape_new(int ptr)
    {
        btStridingMeshInterface *mifPtr = (btStridingMeshInterface *)ptr;
        return (int)new btConvexTriangleMeshShape(mifPtr);
    }

    // btEmptyShape
    int DLL_EXPORT EmptyShape_static()
    {
        static btEmptyShape empty;
        return (int)&empty;
    }

    // btBU_Simplex1to4
    int DLL_EXPORT SimplexShape_new()
    {
        return int(new btBU_Simplex1to4());
    }

    void DLL_EXPORT SimplexShape_addVertex(int ptr, int pt)
    {
        btBU_Simplex1to4 *shape = (btBU_Simplex1to4 *)ptr;
        shape->addVertex(*(btVector3 *)pt);
    }

    int DLL_EXPORT TerrainShape_new(int i, int j, int ptr, float hs, float min, float max)
    {
        btHeightfieldTerrainShape *s =
            new btHeightfieldTerrainShape(i, j, (void *)ptr, hs, min, max, 1, PHY_FLOAT, false);

        return int(s);
    }
// }