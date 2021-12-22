#pragma once
#include "Transition.h"
#include <vector>
#include <cmath>
#include <map>
#include <iostream>
#include <fstream>

Transition::Transition() {
	lambda0 = 37.41255708413501;
	phi0 = 55.97313079458042;
}

Transition::Transition(double lambda, double phi) {
	lambda0 = lambda;
	phi0 = phi;
}


std::vector<double> Transition::fromStart2Geogr(std::vector<double> vec)
{
	double x, z, lambda, dlambda, phi, dphi, psi, dx, dz;
	x = 0; z = 0;
	lambda = lambda0;  phi = phi0;
	dx = 111000; // ��� �� �������
	while ((vec[0] - x) > 300 || (vec[2] - z) > 300)
	{
		psi = atan((vec[2] - z) / (vec[0] - x)); // ������� ���� �����
		dlambda = floor(lambda0) + 1 - lambda0; // ������� ����� ������� �������� � ��������� (� ��������)
		dphi = floor(phi0) + 1 - phi0; // ������� ����� ������� ������� � ��������� (� ��������)

		// ��� �� ������:
		if (dphi < 0.5)
			dz = 1000 * table[int(floor(phi0) + 1)];
		else
			dz = 1000 * table[int(floor(phi0))];

		if ((vec[0] - x) < dx and (vec[2] - z) < dz)
		{
			lambda = lambda + ((vec[2] - z) / dz);
			phi = phi + ((vec[0] - x) / dx);
			x = x + (vec[0] - x);
			z = z + (vec[2] - z);
		}
		else
		{
			if (dlambda * dz < dphi * dx) // �.�. ��������� ������ ����� ��� ��������� �������
			{
				x = x + dphi * dx;
				z = z + (tan(psi) * (dphi * dx));
				phi = phi + dphi;
				lambda = lambda + ((tan(psi) * (dphi * dx)) / dz); // ������� �� ������� �������� ������������� �� ������, �.�. ���� ����� ������� �������
			}
			else
			{
				z = z + dlambda * dz;
				x = x + (atan(psi) * (dlambda * dz));
				lambda = lambda + dlambda;
				phi = phi + ((atan(psi) * (dlambda * dz)) / dx);
			}
		}
	}
	std::vector<double> res = { lambda, phi, vec[1] };
	return res;
}

void Transition::setDefault(double lambda, double phi)
{
	lambda0 = lambda;
	phi0 = phi;
}

double Transition::getAngleFromScalars(std::vector<double> x1, std::vector<double> x2)
{
	if (x1.size() == 3)
	{
		double x1len = sqrt(pow(x1[0], 2) + pow(x1[1], 2) + pow(x1[2], 2));
		double x2len = sqrt(pow(x2[0], 2) + pow(x2[1], 2) + pow(x2[2], 2));
		return  acos((x1[0] * x2[0] + x1[1] * x2[1] + x1[2] * x2[2]) / (x1len * x2len));
	}
	else if (x1.size() == 2)
	{
		double x1len = sqrt(pow(x1[0], 2) + pow(x1[1], 2));
		double x2len = sqrt(pow(x2[0], 2) + pow(x2[1], 2));
		if (x2[1] < 0)
			return  -acos((x1[0] * x2[0] + x1[1] * x2[1]) / (x1len * x2len));
		else if (x2[1] > 0)
			return  acos((x1[0] * x2[0] + x1[1] * x2[1]) / (x1len * x2len));
		else if (x2[1] == 0)
			return 0;
	}
}

double Transition::getDistance(std::vector<double> x1, std::vector<double> x2)
{
	if (x1.size() == 3)
	{
		double a = sqrt(pow(x1[0] - x2[0], 2) + pow(x1[1] - x2[1], 2) + pow(x1[2] - x2[2], 2));
		return a;
	}
	if (x1.size() == 2)
	{
		double a = sqrt(pow(x1[0] - x2[0], 2) + pow(x1[1] - x2[1], 2));
		return a;
	}
}


void Transition::WriteFile(std::ofstream& file1, int flag, std::vector<double> vec) {
	
	if (flag == 1) {
		file1 << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
		file1 << "\n<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\" xmlns:kml=\"http://www.opengis.net/kml/2.2\" xmlns:atom=\"http://www.w3.org/2005/Atom\">";
		file1 << "\n<Document>";
		file1 << "\n    ";
		file1 << "<name>�����������-������.kml</name>";
		file1 << "\n    ";
		file1 << "<Style id=\"s_ylw-pushpin\">";
		file1 << "\n        ";
		file1 << "<IconStyle>";
		file1 << "\n            ";
		file1 << "<scale>1.1</scale>";
		file1 << "\n            ";
		file1 << "<Icon>";
		file1 << "\n                ";
		file1 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
		file1 << "\n            ";
		file1 << "</Icon>";
		file1 << "\n            ";
		file1 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
		file1 << "\n        ";
		file1 << "</IconStyle>";
		file1 << "\n        ";
		file1 << "<LineStyle>";
		file1 << "\n            ";
		file1 << "<color>ffffad41</color>";
		file1 << "\n        ";
		file1 << "</LineStyle>";
		file1 << "\n    ";
		file1 << "</Style>";
		file1 << "\n    ";
		file1 << "<Style id=\"s_ylw-pushpin_hl\">";
		file1 << "\n        ";
		file1 << "<IconStyle>";
		file1 << "\n            ";
		file1 << "<scale>1.3</scale>";
		file1 << "\n            ";
		file1 << "<Icon>";
		file1 << "\n                ";
		file1 << "<href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>";
		file1 << "\n            ";
		file1 << "</Icon>";
		file1 << "\n            ";
		file1 << "<hotSpot x=\"20\" y=\"2\" xunits=\"pixels\" yunits=\"pixels\"/>";
		file1 << "\n        ";
		file1 << "</IconStyle>";
		file1 << "\n        ";
		file1 << "<LineStyle>";
		file1 << "\n            ";
		file1 << "<color>ffffad41</color>";
		file1 << "\n        ";
		file1 << "</LineStyle>";
		file1 << "\n    ";
		file1 << "</Style>";
		file1 << "\n    ";
		file1 << "<StyleMap id=\"m_ylw-pushpin\">";
		file1 << "\n        ";
		file1 << "<Pair>";
		file1 << "\n            ";
		file1 << "<key>normal</key>";
		file1 << "\n            ";
		file1 << "<styleUrl>#s_ylw-pushpin</styleUrl>";
		file1 << "\n        ";
		file1 << "</Pair>";
		file1 << "\n        ";
		file1 << "<Pair>";
		file1 << "\n            ";
		file1 << "<key>highlight</key>";
		file1 << "\n            ";
		file1 << "<styleUrl>#s_ylw-pushpin_hl</styleUrl>";
		file1 << "\n        ";
		file1 << "</Pair>";
		file1 << "\n    ";
		file1 << "</StyleMap>";
		file1 << "\n    ";
		file1 << "<Placemark>";
		file1 << "\n        ";
		file1 << "<name>�����������-������</name>";
		file1 << "\n        ";
		file1 << "<styleUrl>#m_ylw-pushpin</styleUrl>";
		file1 << "\n        ";
		file1 << "<LineString>";
		file1 << "\n            ";
		file1 << "<tessellate>1</tessellate>";
		file1 << "\n            ";
		file1 << "<gx:altitudeMode>relativeToSeaFloor</gx:altitudeMode>";
		file1 << "\n            ";
		file1 << "<coordinates>";
		file1 << "\n                ";
	}
	if (flag == 2) {
		file1 << vec[0]; file1 << ",";
		file1 << vec[1]; file1 << ",";
		file1 << vec[2]; file1 << " ";
	}
	if (flag == 3) {
		file1 << "\n            ";
		file1 << "</coordinates>";
		file1 << "\n        ";
		file1 << "</LineString>";
		file1 << "\n    ";
		file1 << "</Placemark>";
		file1 << "\n</Document>";
		file1 << "\n</kml>";
	}
}

double Transition::fromGrad2Rad(double angle)
{
	return (angle * PI) / 180;
}

double Transition::fromRad2Grad(double rads)
{
	return (rads * 180) / PI;
}