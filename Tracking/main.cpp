#include <iostream>
#include <stdexcept>
#include "UKF.h"

using namespace std;


int print_mat(Mat m);

int main()
try{
	UKF Kalman;
	Kalman.initialize();
	//Kalman.get_State();
	Kalman.prediction_update();
	Kalman.get_State();
	

	cout<<"Hello, world!\n";
	return 0;
}
catch(runtime_error& e)
{
	cerr<<"runtime_error: "<<e.what();
	return 1;
}


int print_mat(Mat m)
{
	for(int i=0;i<m.rows;++i)
    {
        for(int j =0;j<m.cols;++j)
        {
            cout<<m.at<double>(i,j)<<'\t';
        }

        cout<<endl;
    }
    return 0;
}