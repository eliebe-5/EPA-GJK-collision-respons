#include <fstream>
#include <string>
#include <string.h>
#include "ShaderObject.h"

using namespace std;

ShaderObject::~ShaderObject()
{
	if (program != NULL && fragmentShader != NULL)
	{
		glDetachShader(program, fragmentShader);
		glDeleteShader(fragmentShader);
	}
	if (program != NULL && vertexShader != NULL)
	{
		glDetachShader(program, vertexShader);
		glDeleteShader(vertexShader);
	}
		
		
}

void ShaderObject::loadVertexShader(char* path) 
{
	ifstream infile(path);
	string line;
	string vsStr;

	int i = 0;
	while (getline(infile, line))
	{
		line += "\n";

		vsStr += line;
		i++;
	}

	const GLchar * vs = vsStr.c_str();
	

	GLint length = strlen(vs);

	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vs, &length);
	glCompileShader(vertexShader);

	// get error log
	GLint shaderLogSize;
	glGetShaderiv(vertexShader, GL_INFO_LOG_LENGTH, &shaderLogSize);
	if (shaderLogSize > 1)
	{
		GLchar* buf = new GLchar[shaderLogSize];
		glGetShaderInfoLog(vertexShader, shaderLogSize, NULL, buf);
		printf("[SHADER COMPILE ERROR]: %s", buf);
		delete[] buf;
	}
	
}

void ShaderObject::loadFragmentShader(char* path) 
{
	ifstream infile(path);
	string line;
	string psStr;

	int i = 0;
	while (getline(infile, line))
	{
		line += "\n";

		psStr += line;
		i++;
	}

	const GLchar *ps = psStr.c_str();

	GLint length = strlen(ps);

	// setup fragment shader
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	length = strlen(ps);
	glShaderSource(fragmentShader, 1, &ps, &length);
	glCompileShader(fragmentShader);

	// get error log
	GLint shaderLogSize;
	glGetShaderiv(fragmentShader, GL_INFO_LOG_LENGTH, &shaderLogSize);
	if (shaderLogSize > 1)
	{
		GLchar* buf = new GLchar[shaderLogSize];
		glGetShaderInfoLog(fragmentShader, shaderLogSize, NULL, buf);
		printf("[SHADER COMPILE ERROR]: %s", buf);
		delete[] buf;
	}
}

void ShaderObject::loadProgram()
{
	GLint shaderLogSize;

	program = glCreateProgram();

	

	glAttachShader(program, vertexShader);
	glAttachShader(program, fragmentShader);

	glLinkProgram(program);

	glGetProgramiv(program, GL_INFO_LOG_LENGTH, &shaderLogSize);
	if (shaderLogSize > 1)
	{
		GLchar* buf = new GLchar[shaderLogSize];
		glGetProgramInfoLog(program, shaderLogSize, NULL, buf);
		printf("[PROGRAM LINK ERROR]: %s", buf);
		delete[] buf;
	}
}

void ShaderObject::apply()
{
	glUseProgram(program);
}

void ShaderObject::getUniLoc(string name)
{
	auto search = uniMap.find(name);

	if (search != uniMap.end())
	{
		cerr << "Uniform already exists." << endl;
		return;
	}

	GLuint loc = glGetUniformLocation(this->program, name.c_str());

	uniMap.insert(pair<string, GLuint>(name, loc));
}

void ShaderObject::setUniMat(string name, matrix4D mat)
{
	auto search = uniMap.find(name);

	if (search == uniMap.end())
	{
		cerr << "Cant find uniform of that name." << endl;
		return;
	}

	glUniformMatrix4fv(search->second, 1, GL_TRUE, mat.getMat());
}

void ShaderObject::setUniVec(string name, vector4D vec)
{
	auto search = uniMap.find(name);

	if (search == uniMap.end())
	{
		cerr << "Cant find uniform of that name." << endl;
		return;
	}

	glUniform4fv(search->second, 1, vec.getVector());
}
