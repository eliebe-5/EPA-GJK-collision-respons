//------------------------------------------------------------------------------
// exampleapp.cc
// (C) 2015-2016 Individual contributors, see AUTHORS file
//------------------------------------------------------------------------------
#include "config.h"
#include "exampleapp.h"
#include <cstring>
#include <string.h>
#include <vector>

#include <chrono>

#include <stdio.h>

#include "model.h"

matrix4D model;

float dt = 0.001;

std::vector<Model> objects;

matrix4D view;
matrix4D projection;

vector4D eye, target, up;

std::vector<Ray> rays;

matrix4D moveMat;

int ac = 0;

bool physics_on = false;
int physics_step = 0;

using namespace Display;
namespace Example
{

//------------------------------------------------------------------------------
/**
 */
ExampleApp::ExampleApp()
{
}

//------------------------------------------------------------------------------
/**
 */
ExampleApp::~ExampleApp()
{
	// empty
}

//------------------------------------------------------------------------------
/**
 */
bool
ExampleApp::Open()
{
    
    Model object("cube.obj");
    
    
    objects.push_back(object);
    objects.push_back(object);
    
    App::Open();
	this->window = new Display::Window;
	window->SetKeyPressFunction([this](int32 key, int32 scancode, int32 action, int32 mods)
                                {
                                    if (key == GLFW_KEY_ESCAPE && (action == GLFW_PRESS || action == GLFW_REPEAT))
                                    {
                                        this->window->Close();
                                    }

                                    if (key == GLFW_KEY_SPACE && (action == GLFW_PRESS))
                                    {
                                        physics_on = !physics_on;
                                        ac = !ac;
                                    }

                                    if (key == GLFW_KEY_ENTER && (action == GLFW_PRESS || action == GLFW_REPEAT))
                                    {
                                        physics_step++;
                                    }
                                    
                                    if (key == GLFW_KEY_W && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(2, 4, 0.003);
                                    }
                                    if (key == GLFW_KEY_A && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(1, 4, -0.003);
                                    }
                                    if (key == GLFW_KEY_S && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(2, 4, -0.003);
                                    }
                                    if (key == GLFW_KEY_D && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(1, 4, 0.003);
                                    }

                                    if (key == GLFW_KEY_Q && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(3, 4, -0.003);
                                    }
                                    if (key == GLFW_KEY_E && (action == GLFW_PRESS))
                                    {
                                        moveMat.setVal(3, 4, 0.003);
                                    }

                                    if (key == GLFW_KEY_W && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(2, 4, 0);
                                    }
                                    if (key == GLFW_KEY_A && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(1, 4, 0);
                                    }
                                    if (key == GLFW_KEY_S && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(2, 4, 0);
                                    }
                                    if (key == GLFW_KEY_D && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(1, 4, 0);
                                    }
                                    if (key == GLFW_KEY_Q && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(3, 4, 0);
                                    }
                                    if (key == GLFW_KEY_E && (action == GLFW_RELEASE))
                                    {
                                        moveMat.setVal(3, 4, 0);
                                    }
                                });

    window->SetMousePressFunction([this](int32 button, int32 pressed, int32)
                                  {
                                      if (pressed && button == GLFW_MOUSE_BUTTON_LEFT)
                                      {
                                          window->SetMouseMoveFunction([this](float64 ym, float64 xm)
                                                                       {
                                                                           float w = 1024.f;
                                                                           float h = 768.f;

                                                                           w = 2*ym/w;
                                                                           h = 2*xm/h;

                                                                           w = (w - 1.f);
                                                                           h = (1.f - h);
                                                                           
                                                                           rays.push_back(Ray(eye, (vector4D(w, h, 0.f) - eye.norm()).norm() ));
                                                                           vector4D temp;

                                                                           bool hits[2];
                                                                           vector4D hitCol[2];
                                                                           
                                                                           for(int i = 0; i < 2; i++)
                                                                               hits[i] = rays[rays.size() - 1].intersect(objects[i].aabb, &hitCol[i]);
                                                                           
                                                                           int nearest = -1;
                                                                           float dist = 10000.f;
                                                                           
                                                                           for(int i = 0; i < 2; i++)
                                                                           {
                                                                               if(hits[i])
                                                                               {
                                                                                   if((hitCol[i] - eye).len() < dist)
                                                                                   {
                                                                                       dist = (hitCol[i] - eye).len();
                                                                                       nearest = i;
                                                                                   }
                                                                               }
                                                                           }
                                                                           
                                                                           if(nearest != -1)
                                                                           {
                                                                               objects[nearest].applyImpulse(hitCol[nearest],
                                                                                                             hitCol[nearest] - eye,
                                                                                                             0.1
                                                                                   );
                                                                           }

                                                                       });
                                      }
                                      else
                                      {
                                          window->SetMouseMoveFunction([this](float64, float64)
                                                                       {
			
                                                                       });
                                      }
		
                                  });

    eye.setVector(0, 0, -1);
    target.setVector(0, 0, 0);
    up.setVector(0, 1, 0);

    vector4D temp = target - eye;

    vector4D zaxis = temp.norm();
    temp = up.cross(zaxis);
    vector4D xaxis = temp.norm();
    vector4D yaxis = zaxis.cross(xaxis);

    view.setVal(1, 1, xaxis.getX());
    view.setVal(1, 2, xaxis.getY());
    view.setVal(1, 3, xaxis.getZ());
    view.setVal(1, 4, -xaxis.dot(eye));

    view.setVal(2, 1, yaxis.getX());
    view.setVal(2, 2, yaxis.getY());
    view.setVal(2, 3, yaxis.getZ());
    view.setVal(2, 4, -yaxis.dot(eye));

    view.setVal(3, 1, zaxis.getX());
    view.setVal(3, 2, zaxis.getY());
    view.setVal(3, 3, zaxis.getZ());
    view.setVal(3, 4, -zaxis.dot(eye));

    view.setVal(4, 1, 0);
    view.setVal(4, 2, 0);
    view.setVal(4, 3, 0);
    view.setVal(4, 4, 1);
    
	float fov = 90.0;
	float near = 0.1;
	float far = 100.0;

	float scale = 1; // / tan((fov * 2) * (MATH_PI / 180.f));

	projection.setVal(1, 1, scale);
	projection.setVal(2, 2, scale);
	projection.setVal(3, 3, far / (far - near));
	projection.setVal(3, 4, -far * near / (far - near));
	projection.setVal(4, 3, 1);
	projection.setVal(4, 4, 0);
    
	if (this->window->Open())
	{
		// set clear color to gray
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        
        for(int i = 0; i < objects.size(); i++)
        {
            objects[i].shader.loadVertexShader("vs");
            objects[i].shader.loadFragmentShader("ps");
            objects[i].shader.loadProgram();

            objects[i].shader.getUniLoc("MVP");

            objects[i].aabbshader.loadVertexShader("vs2");
            objects[i].aabbshader.loadFragmentShader("ps2");
            objects[i].aabbshader.loadProgram();

            objects[i].aabbshader.getUniLoc("VP");
        }
        glEnable(GL_DEPTH_TEST);
        
		return true;
	}
	return false;
}

//------------------------------------------------------------------------------
/**
 */

void
ExampleApp::Run()
{   
    
    for(int i = 0; i < objects.size(); i++)
    {
        if(i == 0)
            objects[i].mesh = *(MeshResource::loadMesh("cube.obj"));
        else
            objects[i].mesh = *(MeshResource::loadMesh("cube2.obj"));
        
        objects[i].texture.generateHandle();
        objects[i].texture.loadImage("besttexture.jpg");
        objects[i].texture.preRender();

        if(i == 0)
            objects[i].aabb = new AABB("cube.obj");
        else
            objects[i].aabb = new AABB("cube2.obj");
    }

	while (this->window->IsOpen())
	{

        auto s1 = std::chrono::high_resolution_clock::now();
        
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		this->window->Update();

        for(int i = 0; i < objects.size(); i++)
        {
            objects[i].shader.setUniMat("MVP", (projection * view) * objects[i].model);
        
            objects[i].mesh.draw();
            objects[i].shader.apply();
        
            objects[i].aabbshader.setUniMat("VP", (projection * view));
        
            objects[i].drawaabb();

            objects[i].aabbshader.apply();
        }
        
        rays.clear();

        objects[ac].move(moveMat);
        objects[0].aabbshader.setUniMat("VP", (projection * view));
        
        if(physics_on || physics_step > 0)
        {
            if(physics_step > 0)
                physics_step--;
            
            std::vector<AABB> aabbv;
            for(int i = 0; i < objects.size(); i++)
            {
                objects[i].update(dt);
                aabbv.push_back(*(objects[i].aabb));
            }
            
            std::vector<std::pair<int, int> > pairs = sweep(aabbv.data(), aabbv.size(), -1.f, 1.f, 0.01f);
            for(int i = 0; i < pairs.size(); i++)
            {
                std::vector<vector4D> out;
                if(gjk(objects[pairs[i].first], objects[pairs[i].second], out))
                {
                    vector4D normal;
                    float depth;
                    vector4D colP;
                                                    
                    epa(objects[pairs[i].first], objects[pairs[i].second], out, normal, depth, colP);

                    collisionRespons(objects[pairs[i].first], objects[pairs[i].second], depth, normal, colP, dt);
                    
                    printf("COLLISION 100\%, %f units deep\n", depth);
                    printf("COLLISION 100\%, Collision %f %f %f \n", colP[0], colP[1], colP[2]);
                    printf("COLLISION 100\%, Normal %f %f %f \n", normal[0], normal[1], normal[2]);
                }
                else
                    printf("COLLISION 0\%\n");
            }
        }
        
        objects[0].aabbshader.apply();
        this->window->SwapBuffers();

        auto s2 = std::chrono::high_resolution_clock::now();
        dt = (double)std::chrono::duration_cast<std::chrono::milliseconds>(s2 - s1).count() * 0.001;
        
	}
}

} // namespace Example
