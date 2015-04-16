#ifndef MAP_H
#define MAP_H MAP_H

#include <libplayerc/playerc.h>
#include <opencv/highgui.h>

#define MAP_SIZE_X 500
#define MAP_SIZE_Y 500
#define MAP_OFFS_X 250
#define MAP_OFFS_Y 250
#define MAP_SCALE 30.0

#ifndef MAX_GRAY
#define MAX_GRAY 255
#endif 

int map_draw(playerc_ranger_t *ranger, playerc_position2d_t *pos);
int map_shutdown(void);


int isCharted(const int x, const int y);


int isWall(const int x, const int y);

#endif

