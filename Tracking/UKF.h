#include <opencv2/opencv.hpp>


using namespace std;
using namespace cv;


int print_mat(Mat m);



class UKF
{
private:
	double T;		// time step
	double X_lid;	// lidar's position
	double Rho;		// 

	Mat X_state;	// state-space vector
	Mat X_rob; 		// robot's position
	Mat X_cam;		// camera's pos.

	Mat Q;			// process noise
	Mat R_lid;		// observation noise of lidar
	Mat R_cam;		// observation noise of camera
	Mat P; 			// current variance

	Mat X_sigma;	// sigma-points	
	Mat Weights;    // weights



	int init_process_noise();
	int init_observ_noise();
	int init_variance();

	int sigma_point_gen();
	int process_prediction();
	int observ_prediction();

public: 

	UKF();
	int initialize();
	int prediction_update();
	int observation_update();
	int print_sigma_points();
	int get_State();
};

namespace Cholesky
{
	Mat chol(Mat A);
} 