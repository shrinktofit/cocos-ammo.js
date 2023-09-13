#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletDynamicsCommon.h"
#include "macro.h"

extern "C"
{
    void onDebugDrawLine(const int from, const int to, int color) DLL_IMPORT(onDebugDrawLine);
    void onClearLines() DLL_IMPORT(onClearLines);
    void onFlushLines() DLL_IMPORT(onFlushLines);
    // void onControllerHitExt(const int hit, const int controller) DLL_IMPORT(onControllerHit);
}

#define BT_LINE_BATCH_SIZE 512
ATTRIBUTE_ALIGNED16(class)
btDebugDraw : public btIDebugDraw
{
    int m_debugMode = btIDebugDraw::DBG_NoDebug;//btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb;

    DefaultColors m_ourColors;

public:
    BT_DECLARE_ALIGNED_ALLOCATOR()
    virtual ~btDebugDraw(){
    };

    virtual btDebugDraw::DefaultColors getDefaultColors() const{
        return m_ourColors;
    };
    ///the default implementation for setDefaultColors has no effect. A derived class can implement it and store the colors.
    virtual void setDefaultColors(const DefaultColors& colors){
        m_ourColors = colors;
    }
    virtual void drawLine(const btVector3& from1, const btVector3& to1, const btVector3& color1){
        onDebugDrawLine((int)(&from1), (int)(&to1), (int)(&color1));
    }
    virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, 
            btScalar distance, int lifeTime, const btVector3& color) {
		drawLine(PointOnB, PointOnB + normalOnB * distance, color);
		drawLine(PointOnB, PointOnB + normalOnB * 0.01, color);
	}
    virtual void reportErrorWarning(const char* warningString){}
    virtual void draw3dText(const btVector3& location, const char* textString){}
    virtual void setDebugMode(int debugMode)	{
		m_debugMode = debugMode;
	}
    virtual int getDebugMode() const {
		return m_debugMode;
	}
    virtual void clearLines(){
        onClearLines();
    }
    virtual void flushLines(){
        onFlushLines();
    }
    
};

// extern "C"
// {
    // btDebugDraw
    int DLL_EXPORT DebugDraw_new()
    {
        return (int)new btDebugDraw();
    }

    void DLL_EXPORT DebugDraw_setDebugMode(int ptr, int debugMode)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        draw->setDebugMode(debugMode);
    }

    int DLL_EXPORT DebugDraw_getDebugMode(int ptr)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        return draw->getDebugMode();
    }

    void DLL_EXPORT DebugDraw_setActiveObjectColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_activeObject.setX(r);
        colors.m_activeObject.setY(g);
        colors.m_activeObject.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setDeactiveObjectColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_deactivatedObject.setX(r);
        colors.m_deactivatedObject.setY(g);
        colors.m_deactivatedObject.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setWantsDeactivationObjectColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_wantsDeactivationObject.setX(r);
        colors.m_wantsDeactivationObject.setY(g);
        colors.m_wantsDeactivationObject.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setDisabledDeactivationObjectColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_disabledDeactivationObject.setX(r);
        colors.m_disabledDeactivationObject.setY(g);
        colors.m_disabledDeactivationObject.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setDisabledSimulationObjectColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_disabledSimulationObject.setX(r);
        colors.m_disabledSimulationObject.setY(g);
        colors.m_disabledSimulationObject.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setAABBColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_aabb.setX(r);
        colors.m_aabb.setY(g);
        colors.m_aabb.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setContactPointColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_contactPoint.setX(r);
        colors.m_contactPoint.setY(g);
        colors.m_contactPoint.setZ(b);
        draw->setDefaultColors(colors);
    }

    void DLL_EXPORT DebugDraw_setConstraintLimitColor(int ptr, float r, float g, float b)
    {
        btDebugDraw *draw = (btDebugDraw *)ptr;
        btDebugDraw::DefaultColors colors = draw->getDefaultColors();
        colors.m_constraintLimit.setX(r);
        colors.m_constraintLimit.setY(g);
        colors.m_constraintLimit.setZ(b);
        draw->setDefaultColors(colors);
    }
// }