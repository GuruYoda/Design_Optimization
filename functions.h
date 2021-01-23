#include <iostream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>
#include <algorithm>
#include <cmath>

#define _USE_MATH_DEFINES
# define M_PI
using namespace std;

/*This function computes the Circular Proximity Function*/
void CPF(vector<float> x, vector<float> y, float* ans)
{
	float err = 0;
	int l = x.size();

	float sum_x = 0;
	float sum_y = 0;
	float sum_sq = 0;

	for (int i = 0; i < l ; i++)
	{
		sum_x += x[i];
		sum_y += y[i];
		sum_sq += x[i] * x[i] + y[i] * y[i];
	}

	vector<float> a;
	vector<float> b;
	vector<float> c;

	for (int i = 0; i < l; i++)
	{
		a.push_back((2 / l) * sum_x - 2 * x[i]);
		b.push_back((2 / l) * sum_y - 2 * y[i]);
		c.push_back(x[i] * x[i] + y[i] * y[i] - (1 / l) * sum_sq);
	}

	float f1 = 0, f2 = 0, f3 = 0, f4 = 0, f5 = 0;

	for (int i = 0; i < l; i++)
	{
		f1 += a[i] * a[i];
		f2 += b[i] * b[i];
		f3 += 2 * a[i] * c[i];
		f4 += 2 * b[i] * c[i];
		f5 += 2 * a[i] * b[i];
	}

	float C1 = 1 / (4 * f1 * f2 * f5 * f5) * (-2 * f2 * f3 + f5 * f4);
	float C2 = 1 / (4 * f1 * f2 * f5 * f5) * (-2 * f1 * f4 + f5 * f3);

	vector<float> R2;
	for (int i = 0; i < l; i++)
	{
		R2.push_back((x[i] - C1) * (x[i] - C1) + (y[i] - C2) * (y[i] - C2));
	}
	float sum_R2 = 0;
	for (int i = 0; i < l; i++)
	{
		sum_R2 += R2[i];
	}
	float mean_R2 = sum_R2 / l;

	//Computing Error
	for (int i = 0; i < l; i++)
	{
		err += ((R2[i] - mean_R2) / mean_R2) * ((R2[i] - mean_R2) / mean_R2);
	}

	float sum_R = 0;
	for (int i = 0; i < l; i++)
	{
		sum_R += sqrt(R2[i]);
	}
	float mean_R = sum_R / l;

	// Adding the valur of error to the ans array
	ans[0] = err;

	// X-cordinate
	ans[1] = C1;

	//Y-cordinate
	ans[2] = C2;

	// Radius
	ans[3] = mean_R;
	
}

/*Function to check if a point lies inside or outside a Polygon*/
bool PointInPolygon(float x, float y, vector<float> Xd, vector<float> Yd)
{
	int i, j, nvert = Xd.size();
	bool c = false;

	for (i = 0, j = nvert - 1; i < nvert; j = i++) {
		if (((Yd[i] >= y) != (Yd[j] >= y)) &&
			(x <= (Xd[j] - Xd[i]) * (y - Yd[i]) / (Yd[j] - Yd[i]) + Xd[i])
			)
			c = !c;
	}
	return c;

}

/*The Objective function takes as input a vector X = [xA, yA, r3, beta] and the cordinates of precision 
   points Xd and Yd, The output is the error value os candidate solution and the complete set of the 
   design variables of the linkage*/

void objFuncn(vector<float> X, vector<float>Xd, vector<float>Yd, float* optAns)
{
	float xA = X[0];
	float yA = X[1];
	float r3 = X[2];
	float beta = X[3];
	int n = Xd.size();
	vector<float> R;
	for (int i = 0; i < n; i++)
	{
		float a = Xd[i] - xA;
		float b = Yd[i] - yA;
		R.push_back(a*a + b*b);
	}

	int max_index = max_element(R.begin(), R.end()) - R.begin();
	float R_max = *max_element(R.begin(), R.end());

	int min_index = min_element(R.begin(), R.end()) - R.begin();
	float R_min = *min_element(R.begin(), R.end());

	bool in = PointInPolygon(xA, yA, Xd, Yd);
	float r2, r5;
	if (in == true)
	{
		r2 = (R_max + R_min) / 2;
		r5 = (R_max - R_min) / 2;
	}
	else
	{
		r2 = (R_max - R_min) / 2;
		r5 = (R_max + R_min) / 2;
	}

	vector<float> xC1;  // x-cordinate of point C in trajectory 01
	vector<float> yC1;  // y-cordinate of point C in trajectory 01
	vector<float> xC2;  // x-cordinate of point C in trajectory 02
	vector<float> yC2;  // y-cordinate of point C in trajectory 02
	float theta2m1, theta5m1, theta3m1, theta2m2, theta5m2, theta3m2, theta2, theta5, theta3;

	for (int i = 0; i < n; i++)
	{
		if ((i < max_index && i > min_index) || (i > max_index && i < min_index))
		{
			// Trajectory 1
			theta2m1 = atan2(Yd[i] - yA, Xd[i] - xA) + acos((r2 * r2 + R[i] * R[i] - r5 * r5) / (2 * r2 * R[i]));
			theta5m1 = atan2(Yd[i] - yA - r2 * sin(theta2m1), Xd[i] - xA - r2 * cos(theta2m1));
			theta3m1 = theta5m1 - beta;
			xC1[i] = xA + r2 * cos(theta2m1) + r3 * cos(theta3m1);
			yC1[i] = yA + r2 * sin(theta2m1) + r3 * sin(theta3m1);

			// Trajectory 2
			theta2m2 = atan2(Yd[i] - yA, Xd[i] - xA) - acos((r2 * r2 + R[i] * R[i] - r5 * r5) / (2 * r2 * R[i]));
			theta5m2 = atan2(Yd[i] - yA - r2 * sin(theta2m2), Xd[i] - xA - r2 * cos(theta2m2));
			theta3m2 = theta5m2 - beta;
			xC2[i] = xA + r2 * cos(theta2m2) + r3 * cos(theta3m2);
			yC2[i] = yA + r2 * sin(theta2m2) + r3 * sin(theta3m2);
		}
		else if (i == max_index)
		{
			theta2 = atan2(Yd[i] - yA, Xd[i] - xA);
			theta5 = theta2;
			theta3 = theta5 - beta;
			xC1[i] = xA + r2 * cos(theta2) + r3 * cos(theta3);
			yC1[i] = yA + r2 * sin(theta2) + r3 * sin(theta3);
			xC2[i] = xC1[i];
			yC2[i] = yC1[i];
		}
		else if (i == min_index)
		{
			if (PointInPolygon(xA, yA, Xd, Yd) == true)
			{
				theta2 = atan2(Yd[i] - yA, Xd[i] - xA);
				theta5 = M_PI + theta2;
				theta3 = theta5 - beta;
				xC1[i] = xA + r2 * cos(theta2) + r3 * cos(theta3);
				yC1[i] = yA + r2 * sin(theta2) + r3 * sin(theta3);
				xC2[i] = xC1[i];
				yC2[i] = yC1[i];
			}
			else
			{
				theta2 = M_PI + atan2(Yd[i] - yA, Xd[i] - xA);
				theta5 = atan2(Yd[i] - yA, Xd[i] - xA);
				theta3 = theta5 - beta;
				xC1[i] = xA + r2 * cos(theta2) + r3 * cos(theta3);
				yC1[i] = yA + r2 * sin(theta2) + r3 * sin(theta3);
				xC2[i] = xC1[i];
				yC2[i] = yC1[i];
			}
		}
		else
		{
			// Trajectory 2
			theta2m1 = atan2(Yd[i] - yA, Xd[i] - xA) + acos((r2 * r2 + R[i] * R[i] - r5 * r5) / (2 * r2 * R[i]));
			theta5m1 = atan2(Yd[i] - yA - r2 * sin(theta2m1), Xd[i] - xA - r2 * cos(theta2m1));
			theta3m1 = theta5m1 - beta;
			xC2[i] = xA + r2 * cos(theta2m1) + r3 * cos(theta3m1);
			yC2[i] = yA + r2 * sin(theta2m1) + r3 * sin(theta3m1);

			//Trajectory 1
			theta2m2 = atan2(Yd[i] - yA, Xd[i] - xA) - acos((r2 * r2 + R[i] * R[i] - r5 * r5) / (2 * r2 * R[i]));
			theta5m2 = atan2(Yd[i] - yA - r2 * sin(theta2m2), Xd[i] - xA - r2 * cos(theta2m2));
			theta3m2 = theta5m2 - beta;
			xC1[i] = xA + r2 * cos(theta2m2) + r3 * cos(theta3m2);
			yC1[i] = yA + r2 * sin(theta2m2) + r3 * sin(theta3m2);

		}
	}

	float trajectory1[4]; 
	CPF(xC1, yC1, trajectory1);

	float trajectory2[4];
	CPF(xC2, yC2, trajectory2);

	float xD, yD, r1, r4, alpha;

	float error;

	if (trajectory1[0] < trajectory2[0]) // Trajectory 1 is more similar to the circular curve
	{
		error = trajectory1[0];
		xD = trajectory1[1];
		yD = trajectory1[2];
		r1 = sqrt((xA - xD) * (xA - xD) + (yA - yD) * (yA - yD));
		r4 = trajectory1[3];
		alpha = atan2(yD - yA, xD - xA);
	}
	else
	{
		error = trajectory2[0];
		xD = trajectory2[1];
		yD = trajectory2[2];
		r1 = sqrt((xA - xD) * (xA - xD) + (yA - yD) * (yA - yD));
		r4 = trajectory2[3];
		alpha = atan2(yD - yA, xD - xA);
	}

	// saving the final design values in a array
	optAns[0] = r1;
	optAns[1] = r2;
	optAns[2] = r3;
	optAns[3] = r4;
	optAns[4] = r5;
	optAns[5] = beta;
	optAns[6] = xA;
	optAns[7] = yA;
	optAns[8] = alpha;

	float s, l, min = 10000, max = -10000, sum = 0;
	for (int i = 0; i < 4 ; i++)
	{
		if (optAns[i] < min)
		{
			min = optAns[i];
			s = min;
		}
		if (optAns[i] > max)
		{
			max = optAns[i];
			l = max;
		}
		sum += optAns[i];
	}

	float pq = sum - (s + l);

	bool condn1 = (s + l >= pq);
	bool condn2 = (optAns[2] != s);
	if (condn1 == true && condn2 == true)
		error += 1000 + 1000;
	else if (condn1 == true && condn2 == false)
		error += 1000;
	else if (condn1 == false && condn2 == true)
		error += 1000;
	else
		error;

	optAns[9] = error;
}


/* Function to compute atan2 Principal arc tangent of y/x, in the interval [-pi,+pi] radians*/

#pragma once
