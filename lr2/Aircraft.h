#pragma once
#include "SNS.h"
#include "INS.h"
#include <vector>
#include "Transition.h"
#include <mutex>
#include <iostream>
#include <fstream>
using namespace std;


class Aircraft {
private:
	
	Transition tr;
	mutex mutex;
	std::ofstream file1;

	double roll, pitch, yaw;		//���� ������ ��������
	double longitude, latitude;		//������� ������
	double A, theta;				//���� ����� � ��������
	double V, Vx, Vz;				
	vector <double> startSK = { 0,10000,0 };
	vector <double> thetaList = { 0 };
	vector <double> AList = { 0 };

	vector <vector<double>> coordinates;		//���������� �� � ��������� ��
	vector <double> coordinatesG;				//���������� �� � �������������� ��
	vector <vector<double>> PPMs;				//���������� ����� � ��������� ��

	vector <double> Xpr = {0};					//���������� �� ��������
	int index = 0;								//������ �������� �� ���
	double yawmax_pr = 0.14;
	int countPPM = 0;							//���������� �����
	int countOperation = 0;						//���������� ��������

	double t = 0;
	vector<double> distSP = { 3885000, 0 };		//� �� ���. ������ �� ������� �����

public:
	INS ins;
	SNS sns;

	Aircraft();
	Aircraft(double longitude, double latitude, double V0, double A0);
		
	void run();
	void run2();

	vector<double> OPS(int index);
	void OPS2();
	void fillSNS(std::vector<double> vec);
	void fillINS(std::vector<double> vec);

};