#pragma once
#include <iostream>
#include <sstream>
#include <climits>
#include <math.h>

using namespace std;

#define MATH_PI 3.14159f

/*
Konstruktor utan iv?rden,
den skapar en 2D-matris fylld med nollor.
*/

/*
Skapar konstanta variablar f?r att
h?lla koll p? storleken av v?ra vektorer
och v?rde p? x och y position i v?ra
vektorer.
*/

const int sizeofVector = 4;
const int x = 0;
const int y = 1;
const int z = 2;
const int w = 3;

/*
Skapar klass vector4D som ineh?ller
en float array f?r att representera vektorn
och diverse funktioner och operatorer f?r
att hantera objektet.
Funktionsf?rklaringar finns i vector4D.cpp
*/

//Deklarering av klass

class vector4D
{
public:
	vector4D();
	vector4D(const vector4D&);
	vector4D(float, float, float);
	vector4D addition(vector4D);
	vector4D subtraction(vector4D);
	float dot(vector4D);
	vector4D cross(vector4D);
	float len() { return sqrt(pow(vector[x], 2) + pow(vector[y], 2) + pow(vector[z], 2)); };
	vector4D norm();
	float getX() { return vector[x]; };
	float getY() { return vector[y]; };
	float getZ() { return vector[z]; };
	float getW() { return vector[w]; };
	float* getVector() { return vector; };
	void setX(float newX) { vector[x] = newX; };
	void setY(float newY) { vector[y] = newY; };
	void setZ(float newZ) { vector[z] = newZ; };
	void setW(float newW) { vector[w] = newW; };
	void setVector(float newX, float newY, float newZ) { vector[x] = newX; vector[y] = newY; vector[z] = newZ; };
	void divideByW() 
	{ 
		vector[x] = vector[x] / vector[w]; 
		vector[y] = vector[y] / vector[w];
		vector[z] = vector[z] / vector[w];
		vector[w] = vector[w] / vector[w];
	};

	//Overload operators
	vector4D operator+(const vector4D&);
	vector4D operator-(const vector4D&);
	vector4D operator*(const vector4D&);
	vector4D operator*(float);
	vector4D& operator=(const vector4D);
	bool operator==(const vector4D&);
	bool operator!=(const vector4D&);
	float operator[](int val) const { return vector[val]; };
	float& operator[](int val) { return vector[val]; };
private:
	float vector[sizeofVector];
	bool isEqual(vector4D);
	friend ostream& operator << (ostream& os, const vector4D&);
	friend istream& operator >> (istream& is, const vector4D&);
};

ostream& operator <<(ostream& os, vector4D&);
istream& operator >> (istream& is, vector4D&);

/*
Konstruktor f?r inga inv?rden,
vi s?tter v?r vector till (0,0).
*/

inline vector4D::vector4D()
{
	setX(0);
	setY(0);
	setZ(0);
	setW(1);
}

/*
Kopieringskonstruktor.
*/


inline vector4D::vector4D(const vector4D& v)
{
	setX(v.vector[x]);
	setY(v.vector[y]);
	setZ(v.vector[z]);
	setW(v.vector[w]);
}


/*
Konstruktor som tar tv? inv?rden
i form av floats. Vi s?tter v?r vektor
till en (x,y).
*/

inline vector4D::vector4D(float inX, float inY, float inZ)
{
	setX(inX);
	setY(inY);
	setZ(inZ);
}

/*
Funktion f?r addering av tv? vektorer.
Tar ett vektor2D objekt som inv?rde och
adderar "this" med in-v?rdet och returnerar
summan.
*/

inline vector4D vector4D::addition(vector4D toAdd)
{
	vector4D val;
	val.setX(getX() + toAdd.getX());
	val.setY(getY() + toAdd.getY());
	val.setZ(getZ() + toAdd.getZ());
	return val;
}

/*
Funktion f?r subtraktion av tv? vektorer.
Tar ett vektor2D objekt som inv?rde och
subtraherar in-v?rdet fr?n "this" och returnerar
skillnaden.
*/

inline vector4D vector4D::subtraction(vector4D toDed)
{
	vector4D val;
	val.setX(getX() - toDed.getX());
	val.setY(getY() - toDed.getY());
	val.setZ(getZ() - toDed.getZ());
	return val;
}

/*
Funktion f?r att ber?kna dot funktion
mellan "this" och en annan vektor.
*/

inline float vector4D::dot(vector4D toDot)
{
	return getX()*toDot.getX() + getY()*toDot.getY() + getZ()*toDot.getZ();
}

inline vector4D vector4D::cross(vector4D toCross)
{
	vector4D val;
	val.setVector(
        getY()*toCross.getZ() - getZ()*toCross.getY(),
        getZ()*toCross.getX() - getX()*toCross.getZ(),
        getX()*toCross.getY() - getY()*toCross.getX()
        );
	return val;
}

inline vector4D vector4D::norm()
{
	vector4D norm;
	float temp = len();
	float length = 1.0f;

	if (temp == 0)
	{
		length = INT_MAX;
	}
	else
	{
		length = 1.0f / temp;
	}

	norm[0] = vector[0] * length;
	norm[1] = vector[1] * length;
	norm[2] = vector[2] * length;
    norm[3] = vector[3] * length;

	return norm;
}

/*
Funktion f?r att j?mf?ra tv? vectorer
*/

inline bool vector4D::isEqual(vector4D toComp)
{
	if (getX() == toComp.getX() && getY() == toComp.getY() && getZ() == toComp.getZ()) {
		return true;
	}
	else {
		return false;
	}
}

//Overload operators

/*
operatorn +, vi anv?nder oss helt enkelt av
funktionen addition.
*/

inline vector4D vector4D::operator+(const vector4D& v)
{
	return(addition(v));
}

/*
operatorn -, vi anv?nder oss helt enkelt av
funktionen subtraction.
*/

inline vector4D vector4D::operator-(const vector4D& v)
{
	return(subtraction(v));
}

/*
operatorn *, vi anv?nder oss helt enkelt av
funktionen dot.
*/

inline vector4D vector4D::operator*(const vector4D& v)
{
	return(cross(v));
}

/*
V?r andra operator *, vi ber?knar skalning
med en konstant som inv?rde.
*/

inline vector4D vector4D::operator*(float konst)
{
	vector4D mult;
	mult.setX(getX()*konst);
	mult.setY(getY()*konst);
	mult.setZ(getZ()*konst);
	return mult;
}

/*
operatorn = s?tter *this till v?rt
inv?rde och retunerar.
*/

inline vector4D& vector4D::operator=(vector4D v)
{
	setVector(v.getX(), v.getY(), v.getZ());
	return *this;
}

/*
== och != anv?nder sig av funktionen
isEqual.
*/

inline bool vector4D::operator==(const vector4D& v)
{
	if (isEqual(v)) {
		return true;
	}
	else {
		return false;
	}
}

inline bool vector4D::operator!=(const vector4D& v)
{
	if (isEqual(v)) {
		return false;
	}
	else {
		return true;
	}
}

/*
Operatorn << retunerar inv?rdet ostream
med vettigt ineh?ll f?r att man ska f?rsta
vad vektorn ineh?ller.
*/

inline ostream& operator<<(ostream& os, vector4D& v)
{
	os << "(" << v.getX() << "," << v.getY() << "," << v.getZ() << ")";
	return os;
}

/*
Operatorn >> plockar ut de tv? f?rsta v?rdena
i istream:en och placerar dom som x och y v?rde
i vektor.
*/

inline istream& operator >> (istream& is, vector4D& v)
{
	float inX;
	float inY;
	float inZ;
	is >> inX >> inY >> inZ;
	v.setX(inX);
	v.setY(inY);
	v.setZ(inZ);
	return is;
}

class matrix4D
{
public:
	matrix4D();
    matrix4D(float xx, float xy, float xz, float xw, float yx, float yy, float yz, float yw, float zx, float zy, float zz, float zw, float wx, float wy, float wz, float ww);
	~matrix4D();
	matrix4D(const matrix4D &);
	float getVal(int, int);
	float* getRow(int);
	float* getCol(int);
	float* getMat();
	int rowVal() { return rows; };
	int colVal() { return cols; };
	void setVal(int, int, float);
	void setRow(int, stringstream&);
	void setCol(int, stringstream&);
	void setMat(stringstream&);
	void setAsRotationX(float);
	void setAsRotationY(float);
	void setAsRotationZ(float);
	void setAsRotation(float, vector4D);
	vector4D transformvector4D(vector4D);
	matrix4D makeTran();
	matrix4D makeInv();
	matrix4D makeScaling(float);
	void printMat();
	float* getArr(){ return mat; }

    void orthoNorm();
    
	void setPosition(float, float, float);
	vector4D getPosition();

	//Overload operators
	matrix4D operator*(matrix4D&);
	vector4D operator*(vector4D v) { return transformvector4D(v); }
	matrix4D& operator=(const matrix4D&);
	float* operator[](int val);

	int rows;
	int cols;
	float *mat;
};

inline matrix4D::matrix4D()
{
	rows = 4;
	cols = 4;
	mat = new float[rows*cols];
	for (int i = 0; i < rows*cols; i++) {
		mat[i] = 0;
	}
	for (int i = 0; i < rows; i++) {
		mat[(rows*i) + i] = 1;
	}
}

inline matrix4D::matrix4D(float xx, float xy, float xz, float xw, float yx, float yy, float yz, float yw, float zx, float zy, float zz, float zw, float wx, float wy, float wz, float ww)
{
	rows = 4;
	cols = 4;
	mat = new float[rows*cols];

    setVal(1, 1, xx);
    setVal(2, 1, xy);
    setVal(3, 1, xz);
    setVal(4, 1, xw);
    
    setVal(1, 2, yx);
    setVal(2, 2, yy);
    setVal(3, 2, yz);
    setVal(4, 2, yw);
    
    setVal(1, 3, zx);
    setVal(2, 3, zy);
    setVal(3, 3, zz);
    setVal(4, 3, zw);

    setVal(1, 4, wx);
    setVal(2, 4, wy);
    setVal(3, 4, wz);
    setVal(4, 4, ww);
}


inline matrix4D::~matrix4D()
{
	delete[] mat;
}

/*
Kopierings konstruktor;
*/

inline matrix4D::matrix4D(const matrix4D &m)
{
	rows = m.rows;
	cols = m.cols;
	mat = nullptr;
	this->operator=(m);
}

/*
Funktion f?r att h?mta ett v?rde
fr?n matrisen.
*/

inline float matrix4D::getVal(int row, int col)
{
	if (row > rows || col > cols) {
		throw "Out of bounds.";
	}
	return mat[(row - 1) * cols + (col - 1)];
}

/*
Funktion f?r att h?mta en hel rad
fr?n matrisen
*/

inline float* matrix4D::getRow(int row)
{
	float* rRow;
	rRow = new float[cols];

	if (row > rows) {
		throw "Out of bounds.";
	}

	for (int i = 0; i < cols; i++) {
		rRow[i] = mat[(row - 1)*cols + i];
	}

	return rRow;
}

/*
Funktion f?r att h?mta en hel kolumn
fr?n matrisen
*/

inline float* matrix4D::getCol(int col)
{
	float* rCol;
	rCol = new float[rows];

	if (col > cols) {
		throw "Out of bounds.";
	}

	for (int i = 0; i < rows; i++) {
		rCol[i] = mat[(col - 1) + cols*i];
	}

	return rCol;
}

/*
Funktion f?r att h?mta hela matrisen.
*/

inline float* matrix4D::getMat()
{
	return mat;
}

/*
Funktion f?r att s?tta ett specifikt v?rde
i matrisen.
*/

inline void matrix4D::setVal(int row, int col, float val)
{
	if (row > rows || col > cols) {
		throw "Out of bounds.";
	}
	mat[(row - 1)*cols + col - 1] = val;
}

/*
Funktion f?r att s?tta en hel rad till vad
man vill ha den som, tar en stringstream som
inv?rde f?r att ge frihet i vad som ska s?ttas in.
*/

inline void matrix4D::setRow(int row, stringstream &is)
{
	float val;
	int i = 0;
	while (is >> val) {
		if (i == cols) {
			cout << "The row was filled and can not add the " << i << "d/th value (and beyond), which is " << val << endl;
			break;
		}
		mat[(row - 1)*cols + i] = val;
		i++;
	}
}

/*
Funktion f?r att s?tta en hel kolumn till vad
man vill ha den som, tar en stringstream som
inv?rde f?r att ge frihet i vad som ska s?ttas in.
*/

inline void matrix4D::setCol(int col, stringstream &is)
{
	float val;
	int i = 0;
	while (is >> val) {
		if (i == cols) {
			cout << "The col was filled and can not add the " << i << "d/th value (and beyond), which is " << val << endl;
			break;
		}
		mat[(col - 1) + cols*i] = val;
		i++;
	}
}

/*
Funktion f?r att s?tta hela matrisen till vad
man vill ha den som, tar en stringstream som
inv?rde f?r att ge frihet i vad som ska s?ttas in.
*/

inline void matrix4D::setMat(stringstream &is)
{
	float val;
	int i = 0;
	while (is >> val) {
		if (i == cols*rows) {
			cout << "The matrix was filled and can not add the " << i << "d/th value (and beyond), which is " << val << endl;
			break;
		}
		mat[i] = val;
		i++;
	}
}

/*
G?r matrisen till en rotationsmatris med
inv?rde angle f?r att best?mma m?ngden rotation.
*/

inline void matrix4D::setAsRotationX(float angle)
{
	angle = angle * MATH_PI / 180;
	float c = cos(angle);
	float s = sin(angle);
	mat[0] = 1;
	mat[1] = 0;
	mat[2] = 0;
	mat[cols] = 0;
	mat[cols * 2] = 0;
	mat[cols + 1] = c;
	mat[cols + 2] = -s;
	mat[cols * 2 + 1] = s;
	mat[cols * 2 + 2] = c;
}

inline void matrix4D::setAsRotationY(float angle)
{
	angle = angle * MATH_PI / 180;
	float c = cos(angle);
	float s = sin(angle);
	mat[0] = c;
	mat[1] = 0;
	mat[2] = s;
	mat[cols] = 0;
	mat[cols * 2] = -s;
	mat[cols + 1] = 1;
	mat[cols + 2] = 0;
	mat[cols * 2 + 1] = 0;
	mat[cols * 2 + 2] = c;
}

inline void matrix4D::setAsRotationZ(float angle)
{
	angle = angle * MATH_PI / 180;
	float c = cos(angle);
	float s = sin(angle);
	mat[0] = c;
	mat[1] = -s;
	mat[2] = 0;
	mat[cols] = s;
	mat[cols * 2] = 0;
	mat[cols + 1] = c;
	mat[cols + 2] = 0;
	mat[cols * 2 + 1] = 0;
	mat[cols * 2 + 2] = 1;
}

inline void matrix4D::setAsRotation(float angle, vector4D vec)
{

    vector4D uv = vec.norm();

	angle = angle * MATH_PI / 180;
	float c = cos(angle);
	float s = sin(angle);
	setVal(1, 1, c + uv[0]*uv[0]*(1 - c));
	setVal(1, 2, uv[0]*uv[1]*(1 - c) - uv[2]*s);
	setVal(1, 3, uv[0]*uv[2]*(1 - c) + uv[1]*s);
    setVal(2, 1, uv[1]*uv[0]*(1 - c) + uv[2]*s);
    setVal(2, 2, c + uv[1]*uv[1]*(1 - c));
    setVal(2, 3, uv[1]*uv[2]*(1 - c) - uv[0]*s);
    setVal(3, 1, uv[2]*uv[0]*(1 - c) - uv[1]*s);
    setVal(3, 2, uv[2]*uv[1]*(1 - c) + uv[0]*s);
    setVal(3, 3, c + uv[2]*uv[2]*(1 - c));
}

/*
Funktion f?r att transformerar en vector
(f?rslagsvis m.h.a. en rotationsmatris)
*/

inline vector4D matrix4D::transformvector4D(vector4D vector)
{

	vector4D prod;

	for (int j = 0; j < 4; j++) {
		float val = 0;
		for (int k = 0; k < 4; k++) {
			val += getVal(j + 1, k + 1) * vector[k];
		}
		prod[j] = val;
	}

	return prod;

}

/*
G?r helt enkelt om 'this' till ett
transponat och retunerar det.
*/

inline matrix4D matrix4D::makeTran()
{
	matrix4D tran;
	for (int i = 0; i < rows; i++) {
		for (int k = 0; k < cols; k++) {
			tran.setVal(k + 1, i + 1, getVal(i + 1, k + 1));
		}
	}
	return tran;
}

inline matrix4D matrix4D::makeScaling(float scale) 
{ 
	matrix4D scaled;
	scaled.setVal(1, 1, scale);
	scaled.setVal(2, 2, scale);
	scaled.setVal(3, 3, scale);
	return scaled; 
}

inline matrix4D matrix4D::makeInv()
{
	matrix4D inv;

	float det;

	inv.setVal(1, 1, mat[5] * mat[10] * mat[15] -
		mat[5] * mat[11] * mat[14] -
		mat[9] * mat[6] * mat[15] +
		mat[9] * mat[7] * mat[14] +
		mat[13] * mat[6] * mat[11] -
		mat[13] * mat[7] * mat[10]);

	inv.setVal(2, 1, -(mat[4] * mat[10] * mat[15]) +
		mat[4] * mat[11] * mat[14] +
		mat[8] * mat[6] * mat[15] -
		mat[8] * mat[7] * mat[14] -
		mat[12] * mat[6] * mat[11] +
		mat[12] * mat[7] * mat[10]);

	inv.setVal(3, 1, mat[4] * mat[9] * mat[15] -
		mat[4] * mat[11] * mat[13] -
		mat[8] * mat[5] * mat[15] +
		mat[8] * mat[7] * mat[13] +
		mat[12] * mat[5] * mat[11] -
		mat[12] * mat[7] * mat[9]);

	inv.setVal(4, 1, -(mat[4] * mat[9] * mat[14]) +
		mat[4] * mat[10] * mat[13] +
		mat[8] * mat[5] * mat[14] -
		mat[8] * mat[6] * mat[13] -
		mat[12] * mat[5] * mat[10] +
		mat[12] * mat[6] * mat[9]);



	//
	inv.setVal(1, 2, -(mat[1] * mat[10] * mat[15]) +
		mat[1] * mat[11] * mat[14] +
		mat[9] * mat[2] * mat[15] -
		mat[9] * mat[3] * mat[14] -
		mat[13] * mat[2] * mat[11] +
		mat[13] * mat[3] * mat[10]);

	inv.setVal(2, 2, mat[0] * mat[10] * mat[15] -
		mat[0] * mat[11] * mat[14] -
		mat[8] * mat[2] * mat[15] +
		mat[8] * mat[3] * mat[14] +
		mat[12] * mat[2] * mat[11] -
		mat[12] * mat[3] * mat[10]);

	inv.setVal(3, 2, -(mat[0] * mat[9] * mat[15]) +
		mat[0] * mat[11] * mat[13] +
		mat[8] * mat[1] * mat[15] -
		mat[8] * mat[3] * mat[13] -
		mat[12] * mat[1] * mat[11] +
		mat[12] * mat[3] * mat[9]);

	inv.setVal(4, 2, mat[0] * mat[9] * mat[14] -
		mat[0] * mat[10] * mat[13] -
		mat[8] * mat[1] * mat[14] +
		mat[8] * mat[2] * mat[13] +
		mat[12] * mat[1] * mat[10] -
		mat[12] * mat[2] * mat[9]);


	//
	inv.setVal(1, 3, mat[1] * mat[6] * mat[15] -
		mat[1] * mat[7] * mat[14] -
		mat[5] * mat[2] * mat[15] +
		mat[5] * mat[3] * mat[14] +
		mat[13] * mat[2] * mat[7] -
		mat[13] * mat[3] * mat[6]);

	inv.setVal(2, 3, -(mat[0] * mat[6] * mat[15]) +
		mat[0] * mat[7] * mat[14] +
		mat[4] * mat[2] * mat[15] -
		mat[4] * mat[3] * mat[14] -
		mat[12] * mat[2] * mat[7] +
		mat[12] * mat[3] * mat[6]);

	inv.setVal(3, 3, mat[0] * mat[5] * mat[15] -
		mat[0] * mat[7] * mat[13] -
		mat[4] * mat[1] * mat[15] +
		mat[4] * mat[3] * mat[13] +
		mat[12] * mat[1] * mat[7] -
		mat[12] * mat[3] * mat[5]);

	inv.setVal(4, 3, -(mat[0] * mat[5] * mat[14]) +
		mat[0] * mat[6] * mat[13] +
		mat[4] * mat[1] * mat[14] -
		mat[4] * mat[2] * mat[13] -
		mat[12] * mat[1] * mat[6] +
		mat[12] * mat[2] * mat[5]);


	//
	inv.setVal(1, 4, -(mat[1] * mat[6] * mat[11]) +
		mat[1] * mat[7] * mat[10] +
		mat[5] * mat[2] * mat[11] -
		mat[5] * mat[3] * mat[10] -
		mat[9] * mat[2] * mat[7] +
		mat[9] * mat[3] * mat[6]);

	inv.setVal(2, 4, mat[0] * mat[6] * mat[11] -
		mat[0] * mat[7] * mat[10] -
		mat[4] * mat[2] * mat[11] +
		mat[4] * mat[3] * mat[10] +
		mat[8] * mat[2] * mat[7] -
		mat[8] * mat[3] * mat[6]);

	inv.setVal(3, 4, -(mat[0] * mat[5] * mat[11]) +
		mat[0] * mat[7] * mat[9] +
		mat[4] * mat[1] * mat[11] -
		mat[4] * mat[3] * mat[9] -
		mat[8] * mat[1] * mat[7] +
		mat[8] * mat[3] * mat[5]);

	inv.setVal(4, 4, mat[0] * mat[5] * mat[10] -
		mat[0] * mat[6] * mat[9] -
		mat[4] * mat[1] * mat[10] +
		mat[4] * mat[2] * mat[9] +
		mat[8] * mat[1] * mat[6] -
		mat[8] * mat[2] * mat[5]);

	det = mat[0] * inv.getVal(1, 1) + mat[1] * inv.getVal(2, 1) + mat[2] * inv.getVal(3, 1) + mat[3] * inv.getVal(4, 1);

	if (det == 0)
		throw "No inverse";

	float invdet = 1.f / det;

	matrix4D invout;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++) {

			float m;
			m = inv.getVal(i+1, j+1) * invdet;

			invout.setVal(i + 1, j + 1, m);
		}
	}


	return invout;

}

/*
En funktion f?r att enkelt kunna skriva
ut matriserna.
*/

inline void matrix4D::orthoNorm()
{
    vector4D x(mat[0], mat[1], mat[2]);
    vector4D y(mat[4], mat[5], mat[6]);
    vector4D z(mat[8], mat[9], mat[10]);

    vector4D v1 = x.norm();
    vector4D v2 = (y - (v1 * (y.dot(v1)))).norm();
    vector4D v3 = (z - (v1 * (z.dot(v1)))).norm();
    v3 = (v3 - (v2 * (v3.dot(v2)))).norm();

    mat[0] = v1[0]; mat[1] = v1[1]; mat[2] = v1[2];
    mat[4] = v2[0]; mat[5] = v2[1]; mat[6] = v2[2];
    mat[8] = v3[0]; mat[9] = v3[1]; mat[10] = v3[2];
}

inline void matrix4D::printMat()
{
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			cout << mat[i*cols + j] << " ";
		}
		cout << endl;
	}
}

inline void matrix4D::setPosition(float x, float y, float z)
{
	mat[3] = x;
	mat[cols + 3] = y;
	mat[cols * 2 + 3] = z;
}

inline vector4D matrix4D::getPosition() {
	vector4D pos(mat[3], mat[cols + 3], mat[cols * 2 + 3]);
	return pos;
}

//Overload operators

/*
Operator f?r att multiplicera ihop
tv? matriser NxM och XxY d?r N = Y.
*/

inline matrix4D matrix4D::operator*(matrix4D& m)
{
	matrix4D prod;
	for (int j = 0; j < prod.rows; j++) {
		for (int i = 0; i < prod.cols; i++) {
			float val = 0;
			for (int k = 0; k < cols; k++) {
				val += getVal(j + 1, k + 1) * m.getVal(k + 1, i + 1);
			}
			prod.setVal(j + 1, i + 1, val);
		}
	}

	return prod;
}

/*
Tilldelningsoperator.
*/

inline matrix4D& matrix4D::operator=(const matrix4D& m)
{
	if (mat != nullptr)	delete[] mat;

	mat = new float[rows*cols];
	for (int i = 0; i < rows*cols; i++) {
		mat[i] = m.mat[i];
	}
	return *this;
}

inline float* matrix4D::operator[](int val)
{
	float arr[4];

	for (int i = 0; i < 4; i++)
	{
		arr[i] = getVal(val + 1, i + 1);
	}

	return arr;
}
