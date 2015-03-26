#include <iostream>
#include <stdexcept>
#include <cmath>
#include <random>
#include "UKF.h"

using namespace std;
using namespace cv;

const double time_step = 0.01;  // in sec
const double rho = -2;  
const double sigm_z = 0.01;
const double sigm_phi = 3.14/9;
const double sigm_v = 0.1;
const double sigm_r = 0.1;
const double sigm_b = 3.14/60;
const double sigm_alpha = 3.14/45;
const double sigm_beta = 3.14/45;
const double sigm_gamma = 3.14/30;




// INITIALIZATION OF  SATE  //
//-------------------------//


UKF::UKF()
{
	double T = time_step;		// time step
	double Rho = rho;

	X_state = Mat::zeros(5,1,CV_64F);
	X_rob = Mat::zeros(3,1,CV_64F);

	X_lid = 0.0;		
	X_cam = Mat::zeros(4,1,CV_64F);	

	Q = Mat::zeros(5,5,CV_64F);			
	R_lid = Mat::zeros(2,2,CV_64F);
	R_cam = Mat::zeros(3,3,CV_64F);
	P = 0.1*Mat::eye(5,5,CV_64F);

	X_sigma  = Mat::zeros(5,11,CV_64F);
	Weights = Mat::zeros(1,11,CV_64F);

}

int UKF::initialize()					// interface function for initialization
{
	// X_state = ?
	// X_rob  = ?
	init_process_noise();
	init_observ_noise();
	init_variance();
	return 0;
}

int UKF::init_process_noise()			// initialize  process noise matrices
{
	Q.at<double>(2,2) = sigm_z*sigm_z;
	Q.at<double>(3,3) = sigm_phi*sigm_phi;
	Q.at<double>(4,4) = sigm_v*sigm_v;
	return 0;
}

int UKF::init_observ_noise()			// initialize observation noise matrices
{
	R_lid.at<double>(0,0) = sigm_b*sigm_b;
	R_lid.at<double>(1,1) = sigm_r*sigm_r;

	R_cam.at<double>(0,0) = sigm_alpha*sigm_alpha;
	R_cam.at<double>(1,1) = sigm_beta*sigm_beta;
	R_cam.at<double>(2,2) = sigm_gamma*sigm_gamma;

	return 0;
}

int UKF::init_variance()				// initial state of variance
{
	P += Q;
	return 0;
}



int UKF::get_State()					// returns current state vector and variance
{
	cout<<"State-space vector is: ";
	for(int i=0;i<X_state.rows;++i)
		cout<<X_state.at<double>(i,0)<<endl;

	cout<<" Variance P = \n";
    for(int i=0;i<P.rows;++i)
    {
        for(int j =0;j<P.cols;++j)
        {
            cout<<P.at<double>(i,j)<<'\t';
        }

        cout<<endl;
    }
    return 0;
}





// PREDICTION & OBSERVATION ROUTINES  //
//------------------------------------//


int UKF::prediction_update()
{
	//process_prediction();
	//observ_prediction();
	sigma_point_gen();
	print_sigma_points();
	process_prediction();
	observ_prediction();
	return 0;
}

int UKF::observation_update()
{

	return 0;
}

int UKF::process_prediction()				//  
{
	for (int j =0;j<X_sigma)
	{
		//X_state.at<double>(0,0) += X_state.at<double>(4,0)*T*cos(X_state.at<double>(3,0));
		X_sigma.at<double>(0,j) += X_sigma.at<double>(4,j)*T*cos(X_sigma.at<double>(3,j));
		X_sigma.at<double>(1,j) += X_sigma.at<double>(4,j)*T*sin(X_sigma.at<double>(3,j));
		X_sigma.at<double>(2,j) += normal_distriburion<double> n_z(0,sigm_z);
		X_sigma.at<double>(3,j) += normal_distribution<double> n_phi(0,sigm_phi);
		X_sigma.at<double>(4,j) += normal_distribution<double> n_v(0,sigm_v);

	}
	
	for(int j =0;j<X_sigma.cols; ++j)
	{
		X_state = Mat::zeros(5,1,CV_64F);
		Mat x  = Mat::zeros(5,1,CV_64F);
		
		for(int i=0;i<X_sigma.rows;++i)
			x.at<double>(i,0) = Weights.at<double>(j,0)*X_sigma.at<double>(i,j);
			
		X_state += x;
	}
	
	
	return 0;
}

int UKF::observ_prediction()
{
	return 0;
}

int UKF::sigma_point_gen()
{
	Mat Var = Cholesky::chol(P);
	X_sigma.at<double>(0) = X_state.at<double>(0);
	Weights.at<double>(0) = Rho/(5+Rho); 

	for (int j=1; j < X_sigma.cols/2+1; ++j)
	{
		for(int i = 0; i<X_sigma.rows; ++i)
		{
			print_mat(Var.at<double>(i,j-1));
			X_sigma.at<double>(i,j) = X_state.at<double>(i,0) + Var.at<double>(i,j-1);
			X_sigma.at<double>(i,j+X_sigma.cols/2) = X_state.at<double>(i,0) - Var.at<double>(i,j+X_sigma.cols/2-1);
		}
		
		Weights.at<double>(j) =  1/(2*(5+Rho));
		Weights.at<double>(j+X_state.cols/2) = 1/(2*(5+Rho));
	}
	
	return 0;
}

int UKF::print_sigma_points()
{
	for (int i =0; i<X_sigma.rows;++i)
	{
		for(int j=0;j<X_sigma.cols;++j)
			cout<<X_sigma.at<double>(i,j)<<"\t";
	}
	return 0;
}

Mat Cholesky::chol(Mat A)
{
	Mat L = Mat::zeros(5,5,CV_64F);

	L.at<double>(0,0) = sqrt(A.at<double>(0,0));

	for (int i=1;i<A.rows;++i)
	{
		double sum = 0.0;
		for (int j = 0; j < i; ++j)
		{
			double s = 0.0;
			
			for (int k = 0; k < j; ++k)
				s+= L.at<double>(i,k)*L.at<double>(j,k);

			L.at<double>(i,j) = (1/L.at<double>(j,j))*(A.at<double>(i,j)-s);
			sum+=(L.at<double>(i,j))*(L.at<double>(i,j));
			//cout<<sum<<"\n";

		}

		L.at<double>(i,i) = sqrt(A.at<double>(i,i)-sum);

		//cout<<"sum = "<<sum<<"\t"<<L.at<double>(i,i)<<"\n";
	}

	for(int i=0;i<L.rows;++i)
    {
        for(int j =0;j<L.cols;++j)
        {
            cout<<L.at<double>(i,j)<<'\t';
        }

        cout<<endl;
    }

	return L;
}

int UKF::print_mat(Mat m)
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


