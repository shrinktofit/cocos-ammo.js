
#ifndef CC_MATERIAL_H
#define CC_MATERIAL_H

namespace cc {
    struct ccMaterial
    {
        float restitution;
        float friction;
        float rollingFriction;
        float spinningFriction;
        ccMaterial(float r, float f, float rf, float sf) 
        : restitution(r), friction(f), rollingFriction(rf), spinningFriction(sf)
        {
        }

        inline ccMaterial combined(const ccMaterial& b) const {
            float r = restitution * b.restitution;
            float f = friction * b.friction;
            float rf = rollingFriction * b.friction + b.rollingFriction * friction;
            float sf = spinningFriction * b.friction + b.spinningFriction * friction;;
	        constexpr float MAX_FRICTION  = 10.;
            if (f < -MAX_FRICTION) f = -MAX_FRICTION;
            if (f > MAX_FRICTION) f = MAX_FRICTION;
            if (rf < -MAX_FRICTION) rf = -MAX_FRICTION;
            if (rf > MAX_FRICTION) rf = MAX_FRICTION;
            if (sf < -MAX_FRICTION) sf = -MAX_FRICTION;
            if (sf > MAX_FRICTION) sf = MAX_FRICTION;
            return ccMaterial{r, f, rf, sf};
        }
    };
}

#endif // CC_MATERIAL_H
