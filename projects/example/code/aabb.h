#include <string.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include "math.h"

struct AABB
{
    float x, X;
    float y, Y;
    float z, Z;

    vector4D verts[8];
    vector4D ov[8];
    
    AABB(char* path)
    {
        std::vector<float> xs;
        std::vector<float> ys;
        std::vector<float> zs;
        
        FILE * file = fopen(path, "r");
        assert(file != NULL);

        while(true)
        {
            char lineHeader[128];

            int res = fscanf(file, "%s", lineHeader);

            if (res == EOF)
                break;

            if (strcmp(lineHeader, "v") == 0) {

                float t[3];
                fscanf(file, "%f %f %f\n", &t[0], &t[1], &t[2]);
                
                xs.push_back(t[0]);
                ys.push_back(t[1]);
                zs.push_back(t[2]);
            }
        }

        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        std::sort(zs.begin(), zs.end());

        float xmax = xs[xs.size() - 1];
        float ymax = ys[ys.size() - 1];
        float zmax = zs[zs.size() - 1];
        float xmin = xs[0];
        float ymin = ys[0];
        float zmin = zs[0];
        
        verts[0] = vector4D(xmin, ymin, zmin);
        verts[1] = vector4D(xmin, ymin, zmax);
        verts[2] = vector4D(xmin, ymax, zmin);
        verts[3] = vector4D(xmax, ymin, zmin);
        verts[4] = vector4D(xmin, ymax, zmax);
        verts[5] = vector4D(xmax, ymax, zmin);
        verts[6] = vector4D(xmax, ymin, zmax);
        verts[7] = vector4D(xmax, ymax, zmax);

        ov[0] = verts[0];
        ov[1] = verts[1];
        ov[2] = verts[2];
        ov[3] = verts[3];
        ov[4] = verts[4];
        ov[5] = verts[5];
        ov[6] = verts[6];
        ov[7] = verts[7];
        
        x = xs[0];
        y = ys[0];
        z = zs[0];

        X = xs[xs.size() - 1];
        Y = ys[ys.size() - 1];
        Z = zs[zs.size() - 1];
        
    };

    void move(matrix4D translation)
    {
        x = x + translation.getVal(1, 4);
        y = y + translation.getVal(2, 4);
        z = z + translation.getVal(3, 4);

        X = X + translation.getVal(1, 4);
        Y = Y + translation.getVal(2, 4);
        Z = Z + translation.getVal(3, 4);

        for(int i = 0; i < 8; i++)
        {
            verts[i] = verts[i] + translation.getPosition();
        }
    };

    void rotate(matrix4D rotation)
    {
        vector4D com = midPoint();

        std::vector<float> xs;
        std::vector<float> ys;
        std::vector<float> zs;
        
        for(int i = 0; i < 8; i++)
        {
            verts[i] = rotation*ov[i];
            verts[i] = verts[i] + com;
            xs.push_back(verts[i][0]);
            ys.push_back(verts[i][1]);
            zs.push_back(verts[i][2]);
        }

        std::sort(xs.begin(), xs.end());
        std::sort(ys.begin(), ys.end());
        std::sort(zs.begin(), zs.end());

        x = xs[0];
        y = ys[0];
        z = zs[0];

        X = xs[xs.size() - 1];
        Y = ys[ys.size() - 1];
        Z = zs[zs.size() - 1];

    };

    vector4D midPoint()
    {
        return vector4D((x + X)/2, (y + Y)/2, (z + Z)/2);
    };

};
