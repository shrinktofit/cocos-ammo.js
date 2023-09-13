
#include "extensions/ccMaterial.h"
#include "macro.h"

// extern "C"
// {
    using cc::ccMaterial;

    int DLL_EXPORT ccMaterial_new() 
    {
        return (int)new ccMaterial(0.1, 0.6, 0.1, 0.1);
    }

    void DLL_EXPORT ccMaterial_set(int ptr, float r, float f, float rf, float sf) {
        ccMaterial* mat = (ccMaterial*) ptr;
        mat->restitution = r;
        mat->friction = f;
        mat->rollingFriction = rf;
        mat->spinningFriction = sf;
    }
// }
