#include "ShaderObject.h"
#include "TextureResource.h"
#include "MeshResource.h"
#include "math.h"
#include "ray.h"
#include <math.h>
#include <cfloat>
#include <vector>
#include <utility>

struct Model
{
    ShaderObject shader;

    ShaderObject aabbshader;
    
    MeshResource mesh;
    TextureResource texture;
    AABB* aabb;

    float mass;

    matrix4D inertiaO;
    matrix4D inertia;
    
    matrix4D model;

    matrix4D rotDt;
    
    vector4D angularMomentum;
    vector4D spin;

    vector4D momentum;
    vector4D velocity;
    
    Model(char* path)
    {
        mass = 4.f;
        velocity = vector4D(0, 0, 0);

        std::vector<vector4D> verts;
        
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
                
                verts.push_back(vector4D(t[0], t[1], t[2]));
                
            }
        }

        float xx = 0.0f;
        float yy = 0.0f;
        float zz = 0.0f;
        float xy = 0.0f;
        float xz = 0.0f;
        float yz = 0.0f;

        for (int i = 0; i < verts.size(); i++)
        {
            vector4D v = verts[i];
            xx += mass * (v[1]*v[1] + v[2]*v[2]);
            yy += mass * (v[0]*v[0] + v[2]*v[2]);
            zz += mass * (v[0]*v[0] + v[1]*v[1]);

            xy += mass * v[0] * v[1];
            xz += mass * v[0] * v[2];
            yz += mass * v[1] * v[2];
        }
        
        inertiaO = matrix4D( xx, -xy, -xz, 0,
                                   -xy,  yy, -yz, 0,
                                   -xz, -yz,  zz, 0,
                                   0,   0,   0,  1);
    }
    
    void update(float dt)
    {

        matrix4D dtRot = model;        
        dtRot.setVal(1, 4, 0);
        dtRot.setVal(2, 4, 0);
        dtRot.setVal(3, 4, 0);

        float x = model.getVal(1, 4);
        float y = model.getVal(2, 4);
        float z = model.getVal(3, 4);
        
        matrix4D inertiaInv = inertiaO.makeInv();

        matrix4D dtRotT = dtRot.makeTran();
    
        inertia = dtRot*inertiaInv;
        inertia = inertia*dtRotT;
        spin = (inertia*angularMomentum);

        matrix4D skew(
            0, -spin[2], spin[1], 0,
            spin[2], 0, -spin[0], 0,
            -spin[1], spin[0], 0, 0,
            0, 0, 0, 0
            );
        
        matrix4D rotdiff = dtRot * skew;
        
        dtRot.mat[0] = dtRot.mat[0] + rotdiff.mat[0]*dt;
        dtRot.mat[1] = dtRot.mat[1] + rotdiff.mat[1]*dt;
        dtRot.mat[2] = dtRot.mat[2] + rotdiff.mat[2]*dt;
        
        dtRot.mat[4] = dtRot.mat[4] + rotdiff.mat[4]*dt;
        dtRot.mat[5] = dtRot.mat[5] + rotdiff.mat[5]*dt;
        dtRot.mat[6] = dtRot.mat[6] + rotdiff.mat[6]*dt;
        
        dtRot.mat[8] = dtRot.mat[8] + rotdiff.mat[8]*dt;
        dtRot.mat[9] = dtRot.mat[9] + rotdiff.mat[9]*dt;
        dtRot.mat[10] = dtRot.mat[10] + rotdiff.mat[10]*dt;

        dtRot.orthoNorm();

        matrix4D modelInv = model;        
        dtRot.setVal(1, 4, 0);
        dtRot.setVal(2, 4, 0);
        dtRot.setVal(3, 4, 0);
        
        modelInv = modelInv.makeInv();
        
        aabb->rotate(dtRot);
        
        model = dtRot;

        model.setVal(1, 4, x + velocity[0]*dt);
        model.setVal(2, 4, y + velocity[1]*dt);
        model.setVal(3, 4, z + velocity[2]*dt);

        matrix4D move;
        move.setVal(1, 4, velocity[0]*dt);
        move.setVal(2, 4, velocity[1]*dt);
        move.setVal(3, 4, velocity[2]*dt);
        
        aabb->move(move);
    };

    void move(matrix4D move)
    {
        model = move*model;
        aabb->move(move);
    };

    void applyImpulse(vector4D pointOfContact, vector4D direction, float force)
    {
        
        momentum = momentum + direction.norm()*force;
        velocity = momentum*(1/mass);
        
        vector4D com = aabb->midPoint();
        vector4D diff = com - pointOfContact;

        vector4D torque = diff.cross(direction.norm())*force;

        angularMomentum = angularMomentum + torque;
    };

    void drawaabb()
    {
        glLineWidth(2.5f);

        glBegin(GL_LINES);

        glVertex3f(aabb->x, aabb->y, aabb->z);
        glVertex3f(aabb->x, aabb->y, aabb->Z);

        glVertex3f(aabb->x, aabb->y, aabb->z);
        glVertex3f(aabb->x, aabb->Y, aabb->z);

        glVertex3f(aabb->x, aabb->y, aabb->z);
        glVertex3f(aabb->X, aabb->y, aabb->z);

        glVertex3f(aabb->x, aabb->y, aabb->Z);
        glVertex3f(aabb->x, aabb->Y, aabb->Z);

        glVertex3f(aabb->x, aabb->y, aabb->Z);
        glVertex3f(aabb->X, aabb->y, aabb->Z);

        glVertex3f(aabb->X, aabb->y, aabb->z);
        glVertex3f(aabb->X, aabb->Y, aabb->z);

        glVertex3f(aabb->X, aabb->y, aabb->z);
        glVertex3f(aabb->X, aabb->y, aabb->Z);

        glVertex3f(aabb->x, aabb->Y, aabb->z);
        glVertex3f(aabb->X, aabb->Y, aabb->z);

        glVertex3f(aabb->x, aabb->Y, aabb->z);
        glVertex3f(aabb->x, aabb->Y, aabb->Z);

        glVertex3f(aabb->X, aabb->Y, aabb->Z);
        glVertex3f(aabb->x, aabb->Y, aabb->Z);

        glVertex3f(aabb->X, aabb->Y, aabb->Z);
        glVertex3f(aabb->X, aabb->y, aabb->Z);

        glVertex3f(aabb->X, aabb->Y, aabb->Z);
        glVertex3f(aabb->X, aabb->Y, aabb->z);
        
        glEnd();
    };

    vector4D farthestPoint(vector4D direction)
    {
        int farthestIndex = 0;
        float farthestDistance = direction.dot(aabb->verts[0]);
        float temp;
        
        for(int i = 1; i < 8; i++)
        {
            temp = direction.dot(aabb->verts[i]);
            if(temp > farthestDistance)
            {
                farthestDistance = temp;
                farthestIndex = i;
            }
        }

        return aabb->verts[farthestIndex];
    };
    
};

std::vector<std::pair<int, int> > sweep(AABB* boxes, int boxCount, float start, float stop, float interval)
{
    std::vector<std::pair<int, int> > res;

    vector4D pd(start, 0.f, 0.f);
    vector4D pn(1.f, 0.f, 0.f);
    for(; pd[0] < stop; pd[0]+=interval)
    {
        std::vector<int> intersected;

        for(int i = 0; i < boxCount; i++)
        {
            vector4D mid = boxes[i].midPoint();
            vector4D extents;
            extents[0] = boxes[i].X - mid[0];
            extents[1] = boxes[i].Y - mid[1];
            extents[2] = boxes[i].Z - mid[2];

            float r = extents[0]*fabs(pn[0]) + extents[1]*fabs(pn[1]) + extents[2]*fabs(pn[2]);

            float s = pn.dot(mid) - pd.dot(pn);

            if(fabs(s) <= r)
            {
                intersected.push_back(i);
                //printf("Intersected: %i at x: %f\n", i, pd[0]);
            }
        }

        for(int i = 0; i < intersected.size(); i++)
        {
            for(int j = i+1; j < intersected.size(); j++)
            {
                bool exists = false;
                
                for(int k = 0; k < res.size(); k++)
                {
                    if((intersected[i] == res[k].first && intersected[j] == res[k].second) ||
                       (intersected[i] == res[k].second && intersected[j] == res[k].first))
                        exists = true;
                }

                if(!exists)
                {
                    res.push_back(std::pair<int, int>(i, j));
                    
                }
            }
        }
    }
    
    return res;
}

vector4D support(vector4D d, Model &a, Model &b)
{
    //return (a.farthestPoint(direction) - b.farthestPoint(vector4D(0, 0, 0) - direction));

    int maxIndex = 0;
    float maxLength = d.dot(a.aabb->verts[maxIndex]);
    for (int i = 1; i < 8; i++)
    {
        float length = d.dot(a.aabb->verts[i]);
        if (length > maxLength)
        {
            maxIndex = i;
            maxLength = length;
        }
    }

    d = d * -1;

    int minIndex = 0;
    float minLength = d.dot(b.aabb->verts[minIndex]);
    for (int i = 1; i < 8; i++)
    {
        float length = d.dot(b.aabb->verts[i]);
        if (length > minLength)
        {
            minIndex = i;
            minLength = length;
        }
    }

    return a.aabb->verts[maxIndex] - b.aabb->verts[minIndex];
    
}

bool doSimplex(std::vector<vector4D> &simplex, vector4D &direction)
{
    vector4D A;
    vector4D B;
    vector4D C;
    vector4D D;

    vector4D AO;
    vector4D AB;
    vector4D AC;
    vector4D AD;
    
    vector4D temp;
    
    switch(simplex.size())
    {
    case(2):
        A = simplex[1];
        B = simplex[0];
        
        AO = vector4D(0, 0, 0) - A;
        AB = B - A;
        
        direction = (AB.cross(AO)).cross(AB);

        break;
        
    case(3):
        A = simplex[2];
        B = simplex[1];
        C = simplex[0];
        
        AO = vector4D(0, 0, 0) - A;
        AB = B - A;
        AC = C - A;
        
        if(AO.dot(AC.cross(AB).cross(AB)) > 0)
        {
            direction = AB.cross(AO).cross(AB);
            simplex.erase(simplex.begin()+0);
        }
        else
        {
            if(AO.dot(AB.cross(AC).cross(AC)) > 0)
            {
                direction = AC.cross(AO).cross(AC);
                simplex.erase(simplex.begin()+1);
            }
            else
            {
                if(AO.dot(AC.cross(AB)) > 0)
                {
                    direction = AC.cross(AB);
                }
                else
                {
                    direction = vector4D(0, 0, 0) - AC.cross(AB);
                    temp = simplex[1];
                    simplex[1] = simplex[2];
                    simplex[2] = temp;
                }
            }
        }
        
        break;
        
    case(4):

        A = simplex[3];
        B = simplex[2];
        C = simplex[1];
        D = simplex[0];

        AO = vector4D(0, 0, 0) - A;
        AB = B - A;
        AC = C - A;
        AD = D - A;

        if(AO.dot(AC.cross(AB)) > 0)
        {
            direction = AC.cross(AB);
            simplex.erase(simplex.begin()+0);
        }
        else
        {
            if(AO.dot(AD.cross(AC)) > 0)
            {
                direction = AD.cross(AC);
                simplex.erase(simplex.begin()+2);
            }
            else
            {
                if(AO.dot(AB.cross(AD)) > 0)
                {
                    direction = AB.cross(AD);

                    simplex.erase(simplex.begin()+1);
                    temp = simplex[1];
                    simplex[1] = simplex[2];
                    simplex[2] = temp;
                    
                }
                else
                {
                    return true;
                }
            }
        }

        break;

    default:
        printf("Something went very wrong.");
        break;
    }
    
    return false;
}

bool gjk(Model &a, Model &b, std::vector<vector4D> &out)
{
    vector4D S = support(vector4D(0, 0, 1), a, b);
    
    vector4D direction = vector4D(0, 0, 0) - S;

    std::vector<vector4D> simplex;

    simplex.push_back(S);

    while(true)
    {
        vector4D A = support(direction, a, b);
        if(A.dot(direction) < 0)
            return false;
        simplex.push_back(A);

        if(doSimplex(simplex, direction))
        {
            out = simplex;
            
            return true;
        }
    }
}

struct face
{
    vector4D a, b, c;
    vector4D n;
    float d;
};

struct edge
{
    vector4D a, b;
};

int findClosestPlane(std::vector<face> &faces)
{
    int in = 0;
    for(int i = 0; i < faces.size(); i++)
    {
        vector4D AB = faces[i].b - faces[i].a;
        vector4D AC = faces[i].c - faces[i].a;

        faces[i].n = AB.cross(AC);
        faces[i].n = faces[i].n.norm();
        faces[i].d = fabs(faces[i].a.dot(faces[i].n));

        if(faces[i].d < faces[in].d)
        {
            in = i;
        }
    }
    
    return in;
}

void addEdge(edge e, std::vector<edge> &edges)
{
    for (int i = 0; i < edges.size(); i++)
    {
        if (e.a == edges[i].b && e.b == edges[i].a)
        {
            edges.erase(edges.begin()+i);
            return;
        }
    }
    edges.push_back(e);
}

void epa(Model &a, Model &b, std::vector<vector4D> &simplex, vector4D &normal, float &depth, vector4D &colP)
{
    std::vector<face> faces;
    face fa = { simplex[2], simplex[1], simplex[0] };
    face fb = { simplex[3], simplex[2], simplex[0] };
    face fc = { simplex[3], simplex[1], simplex[2] };
    face fd = { simplex[3], simplex[0], simplex[1] };
    faces.push_back(fa);
    faces.push_back(fb);
    faces.push_back(fc);
    faces.push_back(fd);

    int planeI;
    vector4D p;

    while(true)
    {
        planeI = findClosestPlane(faces);
        p = support(faces[planeI].n, a, b);

        if(p.dot(faces[planeI].n) - faces[planeI].d < 0.00001)
        {

            vector4D v0 = faces[planeI].b - faces[planeI].a;
            vector4D v1 = faces[planeI].c - faces[planeI].a;
            vector4D v2 = p - faces[planeI].a;

            float d00 = v0.dot(v0);
            float d01 = v0.dot(v1);
            float d11 = v1.dot(v1);
            float d20 = v2.dot(v0);
            float d21 = v2.dot(v1);
            float denom = d00 * d11 - d01 * d01;

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;

            colP = faces[planeI].a*u + faces[planeI].b*v + faces[planeI].c*w;
            
            normal = vector4D(0, 0, 0) - faces[planeI].n;
            depth = faces[planeI].d / 2;
            
            return;
        }
        else
        {
            std::vector<edge> edges;
            for(int i = faces.size()-1; i >= 0; i--)
            {
                if(faces[i].n.dot(p - faces[i].a) > 0.f)
                {
                    edge ab = { faces[i].a, faces[i].b };
                    edge bc = { faces[i].b, faces[i].c };
                    edge ca = { faces[i].c, faces[i].a };
                    addEdge(ab, edges);
                    addEdge(bc, edges);
                    addEdge(ca, edges);
                    faces.erase(faces.begin()+i);
                }
            }
            
            for(int i = 0; i < edges.size(); i++)
            {
                face nf = { edges[i].a, edges[i].b, p };
                faces.push_back(nf);
            }
        }
    }
}

double rate(double x, double y)
{
    return x * sqrt(y);
}
 
double rk4(double(*f)(double, double), double dx, double x, double y)
{
    double  k1 = dx * f(x, y),
        k2 = dx * f(x + dx / 2, y + k1 / 2),
        k3 = dx * f(x + dx / 2, y + k2 / 2),
        k4 = dx * f(x + dx, y + k3);
    return y + (k1 + 2 * k2 + 2 * k3 + k4) / 6;
}

double euler(double(*f)(double, double), double end_time, double step_time, double acc)
{
    double *y;
    y = new double[(int)(end_time*step_time) + 1];
    y[0] = acc;
    y[1] = y[0] + step_time * f(step_time, y[0]);
    
    for(int step = 2; step <= end_time*step_time; step++)
    {
        y[step] = y[step - 1] + step*step_time*f(step*step_time, y[step - 1]);
    }

    return y[(int)(end_time*step_time)];
    
}

void collisionRespons(Model &a, Model &b, float depth, vector4D normal, vector4D colP, float dt)
{
    //calculated Vars

    vector4D acom = a.aabb->midPoint();
    vector4D bcom = b.aabb->midPoint();
    
    vector4D PA = a.velocity + a.spin.cross(colP - acom);
    vector4D PB = b.velocity + b.spin.cross(colP - bcom);
 
    vector4D difVel = PA - PB;
    float velRelative = difVel.dot(normal);

    if(velRelative > 0.f)
        return;
    
    matrix4D aIinv = a.inertiaO.makeInv();
    matrix4D bIinv = b.inertiaO.makeInv();
    
    float J = (-(1+0.80f)*velRelative)/
        (
            -a.mass - b.mass +
            (
                normal.dot(
                    (aIinv.transformvector4D( a.velocity.cross(normal) ).cross(a.angularMomentum))) +
                normal.dot(
                    (bIinv.transformvector4D( b.velocity.cross(normal) ).cross(b.angularMomentum)))
                )
            );
    
    vector4D JA = normal*J;
    vector4D JB = normal*-J;
 
    vector4D aImp = (colP - acom).cross(JA);
    vector4D bImp = (colP - bcom).cross(JB);

    //a.velocity = a.velocity + normal * euler(rate, dt, dt/10.f, (fabs(J/(a.mass))));

    a.velocity = a.velocity + normal * rk4(rate, dt, a.velocity.len(), fabs(J/(a.mass)));
    
    a.spin = a.inertiaO.makeInv().transformvector4D(aImp);
    a.angularMomentum = a.angularMomentum + aImp;

    printf("KALKUTTA: %f\n", rk4(rate, dt, a.velocity.len(), fabs(J/(a.mass))));
    printf("EULER: %f\n", euler(rate, dt, dt/10.f, (fabs(J/(a.mass)))));
    
    //b.velocity = b.velocity + (vector4D(0, 0, 0) - normal) * euler(rate, dt, dt/10.f, (fabs(J/(a.mass))));
    
    b.velocity = b.velocity + (vector4D(0, 0, 0) - normal) * rk4(rate, dt, b.velocity.len(), fabs(J/(b.mass)));
    
    b.spin = b.inertiaO.makeInv().transformvector4D(bImp);
    b.angularMomentum = b.angularMomentum + bImp;
    
}
