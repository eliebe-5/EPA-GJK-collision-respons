#pragma once
#include <iostream>
#include <vector>
#include "math.h"
#include "../../engine/core/app.h"
#include "../../engine/config.h"

using namespace std;

struct vertex
{
	vertex() {};
	vertex(float x, float y, float z) { pos.setVector(x, y, z); }
	vector4D pos;
	vector4D UV;
	vector4D norm;
};

class MeshResource
{
public:
	MeshResource() {nrOfVertices = 0; nrOfPoints = 0;};
	~MeshResource();
	void createQuad();
	static MeshResource* createCube();
	static MeshResource* loadMesh(char*);
	void setupVBO(int, vertex*);
	void setupEBO(int, int*);
	void draw();
    
private:	
	GLuint VBO, EBO;
	int nrOfVertices, nrOfPoints;
};
