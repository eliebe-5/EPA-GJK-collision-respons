
#include "plane.h"
#include "aabb.h"
#include <stdio.h>

struct Ray
{
    vector4D start;
    vector4D direction;

    Ray(vector4D s, vector4D d)
    {
        start = s;
        direction = d;

        //printf("Start: %f, %f, %f\n", start[0], start[1], start[2]);
        //printf("Direction: %f, %f, %f\n", direction[0], direction[1], direction[2]);
        
    }
    
    bool intersect(Plane* p, vector4D* out)
    {

        if ((direction.dot(p->normal)) == 0)
            return false;
        
        float nDotA = p->normal.dot(start);
        float nDotBA = p->normal.dot(direction);

        *out = start + (direction * ((p->d - nDotA)/nDotBA));

        return true;
 
    };

    float max(float a, float b)
    {
        return a > b ? a : b;
    };

    float min(float a, float b)
    {
        return a < b ? a : b;
    };
    
    bool intersect(AABB* box, vector4D* out)
    {
        float tmin, tmax;

        float t1 = (box->x - start[0])/direction[0];
        float t2 = (box->X - start[0])/direction[0];

        tmin = min(t1, t2);
        tmax = max(t1, t2);

        t1 = (box->y - start[1])/direction[1];
        t2 = (box->Y - start[1])/direction[1];

        tmin = max(tmin, min(t1, t2));
        tmax = min(tmax, max(t1, t2));

        t1 = (box->z - start[2])/direction[2];
        t2 = (box->Z - start[2])/direction[2];

        tmin = max(tmin, min(t1, t2));
        tmax = min(tmax, max(t1, t2));

        if(tmax > max(tmin, 0.f))
        {
            *out = start + direction*tmax;
            return true;
        }
        return false;
        
    };
};
