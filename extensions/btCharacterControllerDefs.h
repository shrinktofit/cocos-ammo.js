#pragma once

#include "LinearMath/btVector3.h"

class btCollisionShape;
class btConvexShape;
class btCollisionWorld;
class btCollisionDispatcher;
class btPairCachingGhostObject;
class btCharacterController;

enum SweepPass {
	SWEEP_PASS_UP = 0,
	SWEEP_PASS_SIDE,
	SWEEP_PASS_DOWN,
	SWEEP_PASS_SENSOR
};

//enum SweepTestFlag {
//	STF_SWEEP_UP				= (1<<0),
//	STF_SWEEP_SIDE				= (1<<1),
//	STF_SWEEP_DOWN				= (1<<2),
//	STF_SWEEP_SIDE_DOWN			= (1<<2)|(1<<1),		// Sweep Side, and Down
//	STF_SWEEP_UP_SIDE_DOWN		= (1<<2)|(1<<1)|(1<<0)  // Sweep Side, Up and Down
//};

enum btControllerCollisionFlag {
	BULLET_CONTROLLER_COLLISION_SIDES	= (1 << 0),				//!< Character is colliding to the sides.
	BULLET_CONTROLLER_COLLISION_UP		= (1 << 1),				//!< Character has collision above.
	BULLET_CONTROLLER_COLLISION_DOWN	= (1 << 2)				//!< Character has collision below.
};

struct btSweptContact {
	btVector3			mWorldPos;		// Contact position in world space
	btVector3			mWorldNormal;	// Contact normal in world space
	btScalar			mDistance;		// Contact distance: mtd. ==0 if penetration
	btCollisionObject*	mColliderObject; 
	btCollisionShape*	mColliderShape; // Touched geometry
};

/**
\brief Describes a generic CCT hit.
*/
struct btControllerHit {
	btCharacterController*	controller;		//!< Current controller
	btVector3				worldPos;		//!< Contact position in world space
	btVector3				worldNormal;	//!< Contact normal in world space
	btVector3				dir;			//!< Motion direction
	float					length;			//!< Motion length
};

/**
\brief Describes a hit between a CCT and a shape. Passed to onShapeHit()
@see btUserControllerHitReport.onShapeHit()
*/
struct btControllerShapeHit : public btControllerHit {
	btCollisionShape*	mCollisionShape;
	btCollisionObject*	mCollisionObject;
};

/**
\brief Describes a hit between a CCT and another CCT. Passed to onControllerHit().
@see btUserControllerHitReport.onControllerHit()
*/
struct btControllersHit : public btControllerHit {
	btCharacterController*	other;			//!< Touched controller
};

/**
\brief User callback class for character controller events.
\note Character controller hit reports are only generated when move is called.
@see btControllerDesc.callback
*/
class btUserControllerHitReport
{
public:

	/**
	\brief Called when current controller hits a shape.
	This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
	\param[in] hit Provides information about the hit.
	@see btControllerShapeHit
	*/
	virtual void onShapeHit(const btControllerShapeHit& hit) = 0;

	/**
	\brief Called when current controller hits another controller.
	\param[in] hit Provides information about the hit.
	@see btControllersHit
	*/
	virtual void onControllerHit(const btControllersHit& hit) = 0;

protected:
	virtual ~btUserControllerHitReport() {}
};

struct btControllerShapeType
{
	enum Enum
	{
		/**
		\brief A box controller.
		@see btBoxController btBoxControllerDesc
		*/
		eBOX,

		/**
		\brief A capsule controller
		@see btCapsuleController btCapsuleControllerDesc
		*/
		eCAPSULE,

		eFORCE_DWORD = 0x7fffffff
	};
};

class btCharacterControllerDesc {
public:
	btScalar m_maxSlopeRadians;  // Slope angle that is set (used for returning the exact value)
	btScalar m_stepHeight;
	btScalar m_contactOffset;
	btVector3 m_up;
	btVector3 m_initPos;
	btControllerShapeType::Enum	mType;

	btUserControllerHitReport* m_userControllerHitReport;
};

class btCapsuleCharacterControllerDesc : public btCharacterControllerDesc {
public:
	btCapsuleCharacterControllerDesc() { mType = btControllerShapeType::eCAPSULE; }
	btScalar m_fRadius;
	btScalar m_fHeight;
};

class btBoxCharacterControllerDesc : public btCharacterControllerDesc {
public:
	btBoxCharacterControllerDesc() { mType = btControllerShapeType::eBOX; }
	btScalar m_fHalfHeight;
	btScalar m_fHalfSideExtent;
	btScalar m_fHalfForwardExtent;
};