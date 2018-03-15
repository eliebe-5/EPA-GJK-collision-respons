#include "../../engine/core/app.h"
#include <map>
#include "math.h"

class ShaderObject
{
private:
	GLuint program = 0;
	GLuint vertexShader;
	GLuint fragmentShader;

	std::map<string, GLuint> uniMap;
public:

	ShaderObject() {};
	~ShaderObject();

	void loadVertexShader(char*);
	void loadFragmentShader(char*);
	void loadProgram();
	void apply();

	void getUniLoc(string);

	void setUniMat(string, matrix4D);
	void setUniVec(string, vector4D);

	GLuint getProgram() { return program; }
};
