
#ifndef CC_COMPOUND_SHAPE_H
#define CC_COMPOUND_SHAPE_H

// #include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "LinearMath/btAlignedObjectArray.h"

class btCollisionShape;
namespace cc 
{
    ATTRIBUTE_ALIGNED16(class) ccCompoundShape	: public btCompoundShape
    {
    public:
        BT_DECLARE_ALIGNED_ALLOCATOR();
        
        void	updateChildTransform(btCollisionShape* shape, const btTransform& newChildTransform, bool shouldRecalculateLocalAabb = true);
    };
}

#endif //CC_COMPOUND_SHAPE_H