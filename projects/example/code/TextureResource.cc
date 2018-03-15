#pragma once
#include "TextureResource.h"
#include <iostream>
//#define STB_IMAGE_IMPLEMENTATION
#include "../../exts/nanovg/src/stb_image.h"

void TextureResource::generateHandle()
{
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void TextureResource::loadImage(char* path)
{
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	int x, y, n;
	unsigned char* image = stbi_load(path, &x, &y, &n, 4);

	if (image == nullptr)
		throw(std::string("Failed to load texture"));

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, x, y, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
	stbi_image_free(image);
}

void TextureResource::loadImageTwo(unsigned char* color, int x, int y)
{
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, x, y, 0, GL_RGB, GL_UNSIGNED_BYTE, color);
	glBindTexture(GL_TEXTURE_2D, 0);
}

unsigned char* TextureResource::getImage(char* path)
{
	int x, y, n;
	unsigned char* image = stbi_load(path, &x, &y, &n, 3);

	if (image == nullptr)
		throw(std::string("Failed to load texture"));

	return image;
}

void TextureResource::preRender()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture);
	glGenerateMipmap(GL_TEXTURE_2D);
	glGenerateTextureMipmap(texture);
}
