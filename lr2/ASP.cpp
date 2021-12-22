#include "ASP.h"
#include <cmath>

void ASP::run()
{
	while (H > 0)
	{
		//coordinates[1] = coordinates[1] + V[1]*dt - g * pow(t, 2) *dt / 2;
		V[1] = V[1] - g * dt;

		coordinates[1] = coordinates[1] + V[1]*dt - g * pow(dt, 2)  / 2;
		H = coordinates[1];
		coordinates[0] = coordinates[0] + V[0] * dt;
		coordinates[2] = coordinates[2] + V[2] * dt;
		/*V[1] = V[1] - g * dt;*/
		coordinatesList.insert(coordinatesList.end(), coordinates);
		coordinatesG.insert(coordinatesG.end(), tr.fromStart2Geogr(coordinates));
		t += dt;
	}
}

ASP::ASP(std::vector<double> coords, std::vector<double> V0)
{
	this->coordinates = coords;
	this->V = V0;
}

void ASP::Ab_f()
{
	A = sqrt(pow(V[0], 2) + pow(V[2], 2)) * (sqrt(2 * H) / g);
}

void ASP::setX(std::vector<double> X)
{
	this->coordinates = X;
}

void ASP::setV(std::vector<double> V)
{
	this->V = V;
}
