#pragma once
#include <vector>
#include "Transition.h"

class ASP
{
public:
	double T_h, H=10000, A;
	double g = 9.81;
	double t = 0;
	double dt = 0.01;
	double c = 0.9;
	Transition tr;


	std::vector <double> coordinates = {};
	std::vector <std::vector<double>> coordinatesList = {}, coordinatesG = {};
	std::vector <double> V;
	bool flag = true;
	void run();
	ASP(std::vector <double> coords, std::vector <double> V0);
	void Ab_f();
	void setX(std::vector <double> X);
	void setV(std::vector <double> V);
	
};

