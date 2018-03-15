
#include "MeshResource.h"

#include <string.h>
#include <string>
#include <sstream>
#include <vector>
#include <map>

void split(const string &s, char delim, vector<string> &elems) {
	stringstream ss;
	ss.str(s);
	string item;
	while (getline(ss, item, delim)) {
		elems.push_back(item);
	}
}

vector<string> split(const string &s, char delim) {
	vector<string> elems;
	split(s, delim, elems);
	return elems;
}

void MeshResource::createQuad()
{
	vertex * VB = new vertex[8];
	int * IB = new int[36];

	vertex v1(-0.5f, -0.5f, 0);
	vertex v2(0.5f, -0.5, 0);
	vertex v3(0.5f, 0.5f, 0);
	vertex v4(-0.5f, 0.5f, 0);


	VB[0] = v1;
	VB[1] = v2;
	VB[2] = v3;
	VB[3] = v4;

	IB[0] = 0;
	IB[1] = 1;
	IB[2] = 2;
	IB[3] = 2;
	IB[4] = 3;
	IB[5] = 0;

	setupVBO(4, VB);
	setupEBO(6, IB);
}

MeshResource* MeshResource::loadMesh(char* path)
{
	std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
	std::vector< vector4D > temp_positions;
	std::vector< vector4D > temp_uvs;
	std::vector< vector4D > temp_normals;

	FILE * file = fopen(path, "r");
	assert(file != NULL);
	
	bool isuv = false;
	bool isnorm = false;


	while (true)
	{

		char lineHeader[128];

		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);

		if (res == EOF)
			break;

		if (strcmp(lineHeader, "v") == 0) {

			float t[3];
			fscanf(file, "%f %f %f\n", &t[0], &t[1], &t[2]);
			vector4D ver(t[0], t[1], t[2]);
			temp_positions.push_back(ver);
		}
		else if (strcmp(lineHeader, "vt") == 0) {

			float t[2];
			fscanf(file, "%f %f\n", &t[0], &t[1]);
			vector4D uv(t[0], t[1], 0.0f);
			temp_uvs.push_back(uv);
		}
		else if (strcmp(lineHeader, "vn") == 0) {

			float t[3];
			fscanf(file, "%f %f %f\n", &t[0], &t[1], &t[2]);
			vector4D norm(t[0], t[1], t[2]);
			temp_normals.push_back(norm);
		}
		else if (strcmp(lineHeader, "f") == 0)
		{
			unsigned int vertexIndex[4], uvIndex[4], normalIndex[4];

			char line[400];
			vector<string> words;

			int matches = fscanf(file, "%[^\n]\n", line);

			words = split(line, ' ');
			while(!words[0].compare(""))
				words.erase(words.begin());

			if (words.size() == 3)
			{

				vector<string> one;
				vector<string> two;
				vector<string> three;
				one = split(words[0], '/');
				two = split(words[1], '/');
				three = split(words[2], '/');

				string::size_type sz;

				vertexIndex[0] = stoi(one[0], &sz);
				vertexIndex[1] = stoi(two[0], &sz);
				vertexIndex[2] = stoi(three[0], &sz);

				vertexIndices.push_back(vertexIndex[0]);
				vertexIndices.push_back(vertexIndex[1]);
				vertexIndices.push_back(vertexIndex[2]);

				if (one.size() > 1)
				{
					if (one[1].compare(""))
					{
						isuv = true;

						uvIndex[0] = stoi(one[1], &sz);
						uvIndex[1] = stoi(two[1], &sz);
						uvIndex[2] = stoi(three[1], &sz);

						uvIndices.push_back(uvIndex[0]);
						uvIndices.push_back(uvIndex[1]);
						uvIndices.push_back(uvIndex[2]);
					}
					if (one.size() > 2)
					{
						if (one[2].compare(""))
						{
							isnorm = true;

							normalIndex[0] = stoi(one[2], &sz);
							normalIndex[1] = stoi(two[2], &sz);
							normalIndex[2] = stoi(three[2], &sz);

							normalIndices.push_back(normalIndex[0]);
							normalIndices.push_back(normalIndex[1]);
							normalIndices.push_back(normalIndex[2]);

						}
					}


				}

			}

			else if (words.size() == 4)
			{
				vector<string> one;
				one = split(words[0], '/');

				vector<string> two;
				vector<string> three;
				vector<string> four;
				two = split(words[1], '/');
				three = split(words[2], '/');
				four = split(words[3], '/');

				string::size_type sz;

				vertexIndex[0] = stoi(one[0], &sz);
				vertexIndex[1] = stoi(two[0], &sz);
				vertexIndex[2] = stoi(three[0], &sz);
				vertexIndex[3] = stoi(four[0], &sz);


				vertexIndices.push_back(vertexIndex[0]);
				vertexIndices.push_back(vertexIndex[1]);
				vertexIndices.push_back(vertexIndex[2]);

				vertexIndices.push_back(vertexIndex[2]);
				vertexIndices.push_back(vertexIndex[3]);
				vertexIndices.push_back(vertexIndex[0]);

				if (one.size() > 1)
				{
					if (one[1].compare(""))
					{
						isuv = true;

						uvIndex[0] = stoi(one[1], &sz);
						uvIndex[1] = stoi(two[1], &sz);
						uvIndex[2] = stoi(three[1], &sz);
						uvIndex[3] = stoi(four[1], &sz);

						uvIndices.push_back(uvIndex[0]);
						uvIndices.push_back(uvIndex[1]);
						uvIndices.push_back(uvIndex[2]);

						uvIndices.push_back(uvIndex[2]);
						uvIndices.push_back(uvIndex[3]);
						uvIndices.push_back(uvIndex[0]);
					}
					if (one.size() > 2)
					{
						if (one[2].compare(""))
						{
							isnorm = true;

							normalIndex[0] = stoi(one[2], &sz);
							normalIndex[1] = stoi(two[2], &sz);
							normalIndex[2] = stoi(three[2], &sz);
							normalIndex[3] = stoi(four[2], &sz);

							normalIndices.push_back(normalIndex[0]);
							normalIndices.push_back(normalIndex[1]);
							normalIndices.push_back(normalIndex[2]);

							normalIndices.push_back(normalIndex[2]);
							normalIndices.push_back(normalIndex[3]);
							normalIndices.push_back(normalIndex[0]);
						}
					}

				}

			}
		}
	}



	vector<vertex> temp;
	map<string, int> mappen;
	map<string, int>::iterator it;

	vector<int> IBV;

	unsigned int p = 0;
	for (unsigned int i = 0; i < vertexIndices.size(); i++)
	{

		unsigned int pind = vertexIndices[i] - 1;

		vertex t;
		t.pos = temp_positions[pind];

		if (isuv)
		{
			unsigned int uvind = uvIndices[i] - 1;
			t.UV = temp_uvs[uvind];
		}
		if (isnorm)
		{
			unsigned int normind = normalIndices[i] - 1;
			t.norm = temp_normals[normind];
		}

		string key;
		ostringstream convert;
		convert << t.pos.getX() << "." << t.pos.getY() << "." << t.pos.getZ() << " "
			<< t.UV.getX() << "." << t.UV.getY() << " "
			<< t.norm.getX() << "." << t.norm.getY() << "." << t.norm.getZ();
		key = convert.str();	

		it = mappen.find(key);
		if (it == mappen.end())
		{
			temp.push_back(t);
			mappen[key] = p;
			IBV.push_back(p);
			p++;
		}
		else
		{
			IBV.push_back(it->second);
		}
	}

	vertex * VB = new vertex[temp.size()];

	for (unsigned int i = 0; i < temp.size(); i++)
	{
		VB[i] = temp[i];
	}

	int * IB = new int[IBV.size()];

	for (unsigned int i = 0; i < IBV.size(); i++)
	{
		IB[i] = IBV[i];
	}

	MeshResource *m = new MeshResource;

	m->setupVBO(temp.size(), VB);
	m->setupEBO(IBV.size(), IB);

	delete[] VB;
	delete[] IB;

	return m;
}

MeshResource* MeshResource::createCube()
{
	MeshResource *m = new MeshResource;

	vertex * VB = new vertex[8];
	int * IB = new int[36];

	vertex v1(-0.25f, -0.25f, 0.25);
	vertex v2(0.25f, -0.25, 0.25);
	vertex v3(0.25f, 0.25f, 0.25);
	vertex v4(-0.25f, 0.25f, 0.25);
	vertex v5(-0.25f, -0.25f, -0.25);
	vertex v6(0.25f, -0.25, -0.25);
	vertex v7(0.25f, 0.25f, -0.25);
	vertex v8(-0.25f, 0.25f, -0.25);

	VB[0] = v1;
	VB[1] = v2;
	VB[2] = v3;
	VB[3] = v4;
	VB[4] = v5;
	VB[5] = v6;
	VB[6] = v7;
	VB[7] = v8;

	vector4D uv1(0, 0, 0);
	vector4D uv2(1, 0, 0);
	vector4D uv3(1, 1, 0);
	vector4D uv4(0, 1, 0);

	VB[0].UV = uv1;
	VB[1].UV = uv2;
	VB[2].UV = uv3;
	VB[3].UV = uv4;
	VB[4].UV = uv1;
	VB[5].UV = uv2;
	VB[6].UV = uv3;
	VB[7].UV = uv4;

	IB[0] = 0;
	IB[1] = 1;
	IB[2] = 2;
	IB[3] = 2;
	IB[4] = 3;
	IB[5] = 0;

	IB[6] = 0;
	IB[7] = 4;
	IB[8] = 5;
	IB[9] = 5;
	IB[10] = 1;
	IB[11] = 0;

	IB[12] = 0;
	IB[13] = 3;
	IB[14] = 7;
	IB[15] = 7;
	IB[16] = 4;
	IB[17] = 0;

	IB[18] = 6;
	IB[19] = 5;
	IB[20] = 4;
	IB[21] = 4;
	IB[22] = 7;
	IB[23] = 6;

	IB[24] = 6;
	IB[25] = 7;
	IB[26] = 3;
	IB[27] = 3;
	IB[28] = 2;
	IB[29] = 6;

	IB[30] = 6;
	IB[31] = 2;
	IB[32] = 1;
	IB[33] = 1;
	IB[34] = 5;
	IB[35] = 6;

	m->setupVBO(8, VB);
	m->setupEBO(36, IB);

	delete[] VB;
	delete[] IB;

	return m;
}


void MeshResource::setupVBO(int nrOfP, vertex* VB)
{
	nrOfPoints = nrOfP;
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertex) * nrOfPoints, VB, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshResource::setupEBO(int nrOfVert, int* IB)
{
	nrOfVertices = nrOfVert;

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(int) * nrOfVert, IB, GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void MeshResource::draw()
{
	// do stuff
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float32) * 12, NULL);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float32) * 12, (GLvoid*)(sizeof(float32) * 4));
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(float32) * 12, (GLvoid*)(sizeof(float32) * 8));

	//Draw
	glDrawElements(GL_TRIANGLES, nrOfVertices, GL_UNSIGNED_INT, NULL);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

MeshResource::~MeshResource()
{
    if(nrOfPoints != 0)
    {
        glDeleteBuffers(sizeof(vertex) * nrOfPoints, &VBO);
        glDeleteBuffers(sizeof(int) * nrOfVertices, &EBO);
    }
}
