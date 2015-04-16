#include "map.h"
#include "opencv/cv.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "memory.h"
#include "assert.h"

#include "laser.h"
#include "frontier.h"
#include "transforms.h"

static const char* mapwin = "Robot Map";
static const char* testwin = "Frontier Detection";
static IplImage* mapimg  = NULL;     
static IplImage* mapimga = NULL;     
IplImage* maptest = NULL;     
static int initialized = 0;

int map_init();
int map_show(void);


int isCharted(const int x, const int y)
{
	const CvScalar s = cvGet2D(mapimg, y, x);
	
	
	return s.val[1] > 0 || s.val[2] > 0;
}


int isWall(const int x, const int y)
{
	if (x < 0 || x >= MAP_SIZE_X) return 0;
	if (y < 0 || y >= MAP_SIZE_Y) return 0;
	const CvScalar s = cvGet2D(mapimg, y, x);
	return (s.val[0] > 0) && (s.val[1] > 0) && (s.val[2] > 0);
}

int map_init()
{
	if (initialized) { return 1; }
	mapimg  = cvCreateImage(cvSize(MAP_SIZE_X,MAP_SIZE_Y),8,3);
	mapimga = cvCreateImage(cvSize(MAP_SIZE_X,MAP_SIZE_Y),8,3);
	maptest = cvCreateImage(cvSize(MAP_SIZE_X,MAP_SIZE_Y),8,3);
	cvZero(mapimg);
	// create window
	cvNamedWindow( mapwin, 1 );
	cvNamedWindow( testwin, 1 );
	initialized=1;
	return 0;
}


void setze_wand_dick(double x, double y)
{
	const int width = 2;
	for (int pady = -width/2; pady < width; ++pady) 
	{
		for (int padx = -width/2; padx < width; ++padx)
		{
			cvSet2D( mapimg,
			    MAP_OFFS_Y-(int)(MAP_SCALE*y)+pady,
			    MAP_OFFS_X+(int)(MAP_SCALE*x)+padx,
			    CV_RGB(MAX_GRAY, MAX_GRAY, MAX_GRAY)
			  );
		}
	}
}


void setze_gesehen_dick(double x, double y, int is_frontier)
{
	const int frontier_value = 92;
	const int seen_value = 64;

	const int width = 2;
	for (int pady = -width/2; pady < width; ++pady) 
	{
		for (int padx = -width/2; padx < width; ++padx)
		{
			CvScalar s = cvGet2D( mapimg,
    	        MAP_OFFS_Y-(int)(MAP_SCALE*y)+pady,
	            MAP_OFFS_X+(int)(MAP_SCALE*x)+padx
				);

			
			if (s.val[1] == MAX_GRAY)
				continue;

			
			int green = s.val[1];
			if (green > seen_value)
				continue;

			
			if (is_frontier)
				green = frontier_value;
			else
				green = seen_value;

			cvSet2D( mapimg,
			    MAP_OFFS_Y-(int)(MAP_SCALE*y)+pady,
			    MAP_OFFS_X+(int)(MAP_SCALE*x)+padx,
			    CV_RGB(s.val[2], green, s.val[0])
			  );
		}
	}
}


int map_draw(playerc_ranger_t *ranger, playerc_position2d_t *pos)
{
	if (!initialized) { if (map_init()) return 1; }

   	

	for (uint32_t a=0; a < ranger->ranges_count; ++a)       
	{
		
		double angle = a * LASER_ANGULAR_RESOLUTION_RAD + LASER_MIN_ANGLE_RAD;
		double radius = ranger->ranges[a];

		
		double x, y;
		int is_frontier = transformLaserToMap(angle, radius, pos, &x, &y);

		
		if (is_frontier)
		{
			setze_wand_dick(x, y);
		}

		
		double r = 0;
		const double deltaRadius = 0.1;
		x = y = -1;
		do 
		{
			
			double newX, newY;
			transformLaserToMap(angle, r, pos, &newX, &newY);
			if (newX == x && newY == y)
				continue;
			x = newX;
			y = newY;
			setze_gesehen_dick(x, y, is_frontier);
			r += deltaRadius;
		} while (r < radius - deltaRadius);
	}


	
	double nearestX, nearestY;
	int foundUncharted = checkForOpenSpaces(pos->px, pos->py, &nearestX, &nearestY);

	
	cvSet2D( mapimg,
		    MAP_OFFS_Y-(int)(MAP_SCALE*pos->py),
		    MAP_OFFS_X+(int)(MAP_SCALE*pos->px),
		    CV_RGB(MAX_GRAY,0,0)
		  );

	
	if (foundUncharted)
	{
		printf("%d unkartierte. Nähester: x=%7.5f, y=%7.5f\n", 
			foundUncharted, nearestX, nearestY);

		
		cvCopy(mapimg, mapimga);

		
		CvScalar color;
		color.val[0] = 0;
		color.val[1] = MAX_GRAY;
		color.val[2] = MAX_GRAY;
		color.val[3] = 0;

		CvPoint start, end;
		start.x = MAP_OFFS_X+(int)(MAP_SCALE*pos->px);
		start.y = MAP_OFFS_Y-(int)(MAP_SCALE*pos->py);

		end.x = MAP_OFFS_X+(int)(MAP_SCALE*nearestX);
		end.y = MAP_OFFS_Y-(int)(MAP_SCALE*nearestY);
		cvLine(mapimga, start, end, color, 1, 8, 0);
	}
	else
	{
#if 0
		printf("Keine unkartierten Punkte gefunden.\n");
#endif

		
		cvCopy(mapimg, mapimga);
	}

	
	map_show();
	return (foundUncharted == 0);
}

int map_show()
{
   if (!initialized) { return 1; }
   cvShowImage(mapwin, mapimga);
   cvShowImage(testwin, maptest);
   cvWaitKey(20);
   return 0;
}
 
int map_shutdown()
{
	if (!initialized) { return 1; }
	cvDestroyWindow(mapwin);
	cvDestroyWindow(testwin);
	cvReleaseImage(&mapimg);
	cvReleaseImage(&mapimga);
	cvReleaseImage(&maptest);
	initialized=0;
	return 0;
}

