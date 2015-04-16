#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <libplayerc/playerc.h>

#define LERP(domainMin, domainMax, x, targetMin, targetMax) \
	((x-domainMin)/(domainMax-domainMin) * targetMax + targetMin)


#define LERP_SATURATE(domainMin, domainMax, x, targetMin, targetMax) \
	((((x > domainMax) ? domainMax : ((x < domainMin) ? domainMin : x)) -domainMin)/(domainMax-domainMin) * targetMax + targetMin)


void transformLocalToMap(double x, double y, const playerc_position2d_t *const pos, double *mapx, double *mapy);


int transformLaserToMap(double angle, double radius, const playerc_position2d_t *const pos, double *mapx, double *mapy);

#endif
