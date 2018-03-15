#include "ray.h"
#include "MeshResource.h"
#include "TextureResource.h"

#include "math.h"

struct Square
{
    Square()
    {
    };
    
    Square(vector4D a, vector4D b, vector4D c, vector4D d)
    {
        p = Plane(a, b, c);
        p0 = a;
        p1 = b;
        p2 = c;
        p3 = d;
        op0 = p0;
        op1 = p1;
        op2 = p2;
        op3 = p3;
    };

    void setRotation(matrix4D rot)
    {
        float x = model.getVal(1, 4);
        float y = model.getVal(2, 4);
        float z = model.getVal(3, 4);
        model = rot;
        
        //printf("%f, %f, %f\n", x, y, z);
        p.normal = model*p.calcNormal(op0, op1, op2);

        model.setVal(1, 4, x);
        model.setVal(2, 4, y);
        model.setVal(3, 4, z);
        
        p0 = model*op0;
        p1 = model*op1;
        p2 = model*op2;
        p3 = model*op3;
    };

    void move(matrix4D translation)
    {
        model = model * translation;
        p0 = model*op0;
        p1 = model*op1;
        p2 = model*op2;
        p3 = model*op3;
        //printf("%f, %f, %f\n", p0[0], p0[1], p0[2]);
    };
    
    bool hit(vector4D point)
    {
        vector4D AB = p1 - p0;
        vector4D BC = p2 - p1;
        vector4D AM = point - p0;
        vector4D BM = point - p1;
        
        if(0 <= AB.dot(AM) && AB.dot(AM) <= AB.dot(AB) &&
            0 <= BC.dot(BM) && BC.dot(BM) <= BC.dot(BC))
            return true;
        return false;
    };
    
    vector4D p0, p1, p2, p3;
    vector4D op0, op1, op2, op3;
    
    Plane p;
    MeshResource mesh;
    TextureResource text;

    matrix4D model;
};
