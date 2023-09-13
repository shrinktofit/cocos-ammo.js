
#include "ccCompoundShape.h"

namespace cc 
{
	void    ccCompoundShape::updateChildTransform(btCollisionShape* shape, const btTransform& newChildTransform, bool shouldRecalculateLocalAabb)
	{
		for(int i = m_children.size()-1; i >= 0 ; i--)
		{
			if(m_children[i].m_childShape == shape)
			{
				btCompoundShape::updateChildTransform(i, newChildTransform, shouldRecalculateLocalAabb);
				break;
			}
		}
	}
}