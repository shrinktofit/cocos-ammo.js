
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btMotionState.h"
#include "LinearMath/btTransform.h"
#include "macro.h"

extern "C"
{
    void syncPhysicsToGraphics(const int id) DLL_IMPORT(syncPhysicsToGraphics);
}

namespace cc
{
class ccMotionState : public btMotionState
{
  public:
    const int m_id;
    btTransform m_graphicsWorldTrans;

    ccMotionState(const int id, const btTransform &startTrans = btTransform::getIdentity())
    : m_id(id), m_graphicsWorldTrans(startTrans)
    {
    }

    virtual void getWorldTransform(btTransform &centerOfMassWorldTrans) const
    {
        centerOfMassWorldTrans = m_graphicsWorldTrans;
    }

    virtual void setWorldTransform(const btTransform &centerOfMassWorldTrans)
    {
        m_graphicsWorldTrans = centerOfMassWorldTrans;
        syncPhysicsToGraphics(m_id);
    }
};
} // namespace cc

extern "C"
{

    void DLL_EXPORT MotionState_getWorldTransform(int ptr, int ptr2)
    {
        btMotionState *p = (btMotionState *)ptr;
        btTransform *trans = (btTransform *)ptr2;
        p->getWorldTransform(*trans);
    }

    void DLL_EXPORT MotionState_setWorldTransform(int ptr, int ptr2)
    {
        btMotionState *p = (btMotionState *)ptr;
        btTransform *trans = (btTransform *)ptr2;
        p->setWorldTransform(*trans);
    }

    // default motion state
    int DLL_EXPORT DefaultMotionState_new(int ptr)
    {
        btDefaultMotionState *ms = new btDefaultMotionState(*(btTransform *)ptr);
        return int(ms);
    }

    // ccMotionState
    int DLL_EXPORT ccMotionState_new(int id, int ptr1)
    {
        return (int)new cc::ccMotionState(id, *(btTransform *)ptr1);
    }
}