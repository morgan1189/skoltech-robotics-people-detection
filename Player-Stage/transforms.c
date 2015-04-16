#include "laser.h"
#include "math.h"
#include "transforms.h"


void transformLocalToMap(const double x, const double y, const playerc_position2d_t *const pos, double *mapx, double *mapy)
{
	const double theta = pos->pa;
	const double sint = sin(theta);
	const double cost = cos(theta);
	*mapx =  cost*x - sint*y + pos->px;
	*mapy =  sint*x + cost*y + pos->py;
}


int transformLaserToMap(const double angle, const double radius, const playerc_position2d_t *const pos, double *mapx, double *mapy)
{
	
	if (radius >= LASER_RANGE_MAX - LASER_RANGE_EPSILON)
	{
		return 0;
	}

	
	const double lx = radius * cos(angle);
	const double ly = radius * sin(angle);

	
	transformLocalToMap(lx, ly, pos, mapx, mapy);

	return 1;
}
