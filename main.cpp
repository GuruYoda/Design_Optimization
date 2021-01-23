#include "functions.h"

int main() {

	// define variables
	string Order, x, y; //variables from file are here
	vector<float>Order_vec;
	vector<float>x_vec;
	vector<float>y_vec;

	//input filename
	string file;
	cout << "Enter the filename: ";
	cin >> file;

	//number of lines
	int i = 0;

	ifstream coeff(file); //opening the file.
	if (coeff.is_open()) //if the file is open
	{
		//ignore first line
		string line;
		getline(coeff, line);

		while (!coeff.eof()) //while the end of file is NOT reached
		{
			//I have 4 sets {alpha, CD, CL, CY} so use 4 getlines
			getline(coeff, Order, ',');
			Order_vec.push_back(stof(Order));
			getline(coeff, x, ',');
			x_vec.push_back(stof(x));
			getline(coeff, y, '\n'); //new line 
			y_vec.push_back(stof(y));

			i += 1; //increment number of lines
		}
		coeff.close(); //closing the file
		cout << "Number of entries: " << i << endl;
	}
	else cout << "Unable to open file"; //if the file is not open output

	//output values and find CD0, CY0, CL0
	cout << "Order" << "\t" << "x" << "\t" << "y" << endl;
	float x0, y0;
	for (int j = 0; j < i; j++) {
		cout << Order_vec[j] << "\t" << x_vec[j] << "\t" << y_vec[j] << endl;
		if (Order[j] == 0) {
			x0 = x_vec[j];
			y0 = y_vec[j];
		}
	}
	cout << endl;

	float a[4];
	CPF(x_vec, y_vec, a);
	cout << "The value of error is       : " << a[0] << endl;
	cout << "The value of X-cordinate is : " << a[1] << endl;
	cout << "The value of Y-cordinate is : " << a[2] << endl;
	cout << "The value of Radius is      : " << a[3] << endl;

	float b[10];
	vector<float> X;
	X.push_back(10);
	X.push_back(4);
	X.push_back(41);
	X.push_back(1);
	//objFuncn(X, x_vec, y_vec, b);

}
