#include "math.h"

struct Plane
{
    vector4D normal;
    float d;

    vector4D calcNormal(vector4D a, vector4D b, vector4D c)
    {
        vector4D ab = b - a;
        vector4D ac = c - a;
        return ab.cross(ac);
    };
    
    Plane(vector4D ia, vector4D ib, vector4D ic)
    {
        normal = calcNormal(ia, ib, ic);
        d = normal.dot(ia);
    };
    
    Plane()
    {
    };
    
};
