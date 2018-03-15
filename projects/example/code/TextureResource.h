#pragma once

#include "../../engine/core/app.h"
#include <string>
#include "../../engine/config.h"

using namespace std;

class TextureResource
{
private:
	GLuint texture;
public:
	TextureResource() {};
	~TextureResource() {};
	void generateHandle();
	void loadImage(char *);
	void loadImageTwo(unsigned char *, int, int);
	void preRender();
	unsigned char* getImage(char *);
};
