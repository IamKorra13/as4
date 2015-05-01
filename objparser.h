#ifndef OBJPARSER_H
#define OBJPARSER_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <exception>
#include <cstring>
#include <string>
#include <cstdlib>
#include "vector.h"
// #include "triangle.cpp"

using namespace std;


class ObjParser {
public:
	vector<vector<float> > verticies;
	vector<vector<vector<float> > > faces;
	string file;
	ObjParser();
	ObjParser(string);
	vector<Triangle*> parse();

};

#endif
