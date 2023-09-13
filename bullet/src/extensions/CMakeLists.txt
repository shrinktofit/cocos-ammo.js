
INCLUDE_DIRECTORIES(
	${BULLET_PHYSICS_SOURCE_DIR}/src
	${BULLET_PHYSICS_SOURCE_DIR}/../
	${BULLET_PHYSICS_SOURCE_DIR}/../extensions
)

SET(Extensions_SRCS
ccDiscreteDynamicsWorld.cpp
ccKinematicCharacterController.cpp
ccCompoundShape.cpp
btCharacterController.cpp
)

SET(Extensions_HDRS
ccDiscreteDynamicsWorld.h
ccCompoundShape.h
ccMaterial.h
ccKinematicCharacterController.h
ccOverlapFilterCallback.h
ccRayResultCallback.h
ccConvexCastCallBack.h
btCharacterController.h
btCharacterControllerDefs.h
)

ADD_LIBRARY(Extensions ${Extensions_SRCS} ${Extensions_HDRS})
SET_TARGET_PROPERTIES(Extensions PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(Extensions PROPERTIES SOVERSION ${BULLET_VERSION})
IF (BUILD_SHARED_LIBS)
  TARGET_LINK_LIBRARIES(Extensions LinearMath BulletDynamics BulletCollision)
ENDIF (BUILD_SHARED_LIBS)