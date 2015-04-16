

#include "map.h"
#include "opencv/cv.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "limits.h"

extern IplImage* maptest;    

typedef struct {
	int y;				
	int startx;			
	int endx;			
	int rightUncharted;	
} scanlinerange_t;


typedef struct _scanlinerange_chain {
	scanlinerange_t scanline;		
	_scanlinerange_chain *next;	
} scanlinerange_chain_t;



static inline int shouldBeVisited(const int x, const int y)
{
	if (x < 0 || x >= MAP_SIZE_X) return 0;
	if (y < 0 || y >= MAP_SIZE_Y) return 0;
	const CvScalar s = cvGet2D(maptest, y, x);
	return (s.val[0] == 0) && !isWall(x, y);
}


static inline int markAsVisited(const int x, const int y, const int isCharted)
{
	if (x < 0 || x >= MAP_SIZE_X) return 0;
	if (y < 0 || y >= MAP_SIZE_Y) return 0;
	cvSet2D(maptest, y, x, CV_RGB((isCharted == 0 ? MAX_GRAY : 0), (isCharted == 0 ? MAX_GRAY : 0), (isCharted == 0 ? MAX_GRAY : 64)));

#if 0
	cvShowImage(testwin, maptest);
	cvWaitKey(1);
#endif

	return 0;
}


int buildScanLine(const int scanStartX, const int scanStartY, scanlinerange_t *range)
{
	int startX = scanStartX;
	int endX   = scanStartX;
	int leftUncharted  = 0;
	int rightUncharted = 0;

	
	for (int x = scanStartX; x >= 0; --x)
	{
		if (!shouldBeVisited(x, scanStartY)) 
		{
			break;
		}

		
		int charted = isCharted(x, scanStartY);

		
		markAsVisited(x, scanStartY, charted);

		if (!charted)
		{
			leftUncharted = 1;
			break;
		}
		startX = x;
	}

	
	for (int x = scanStartX+1; x < MAP_SIZE_X; ++x)
	{
		if (!shouldBeVisited(x, scanStartY)) 
		{
			break;
		}

		
		int charted = isCharted(x, scanStartY);

		
		markAsVisited(x, scanStartY, charted);

		if (!charted)
		{
			rightUncharted = 1;
			break;
		}
		endX = x;
	}

	
	range->y = scanStartY;	
	range->startx = startX;
	range->endx = endX;
	range->leftUncharted = leftUncharted;
	range->rightUncharted = rightUncharted;
	
	return leftUncharted + rightUncharted;
}


static inline int getDistance(const int referenceX, const int referenceY, const int testX, const int testY)
{
	
	return labs(testX - referenceX) + labs(testY - referenceY);
}


static inline int getNearest(const int referenceX, const int referenceY, const int testX, const int testY, int& secondX, int& secondY)
{
	const int testDistance = getDistance(referenceX, referenceY, testX, testY);
	const int distance     = getDistance(referenceX, referenceY, secondX, secondY);
	if (testDistance < distance)
	{
		secondX = testX;
		secondY = testY;
		return testDistance;
	}
	return distance;
}


uint32_t extendScanLine(const int startX, const int endX, const int y, const int mapx, const int mapy, scanlinerange_chain_t **tail, int *nearestUnchartedX, int *nearestUnchartedY, int *distanceToNearestUncharted)
{
	scanlinerange_t range;
	uint32_t foundUncharted = 0;

	for (int x = startX; x <= endX; ++x)
	{
		
		if (!shouldBeVisited(x, y)) continue;
		uint32_t unchartedCount = buildScanLine(x, y, &range);
		assert(range.y == y);

		
		if (unchartedCount != 0)
		{
			foundUncharted += unchartedCount;

			if (range.leftUncharted)
			{
				*distanceToNearestUncharted = getNearest(mapx, mapy, range.startx, range.y, *nearestUnchartedX, *nearestUnchartedY);
			}
			if (range.rightUncharted)
			{
				*distanceToNearestUncharted = getNearest(mapx, mapy, range.endx, range.y, *nearestUnchartedX, *nearestUnchartedY);
			}
		}

		
		if (range.startx == range.endx) continue;

		
		scanlinerange_chain_t *item = (scanlinerange_chain_t*)malloc(sizeof(scanlinerange_chain_t));
		memcpy(&item->scanline, &range, sizeof(scanlinerange_t));
		item->next = (*tail)->next;
		(*tail)->next = item;
		(*tail) = item;
	}

	return foundUncharted;
}


int checkForOpenSpaces(const double startX, const double startY, double *outNearestX, double* outNearestY)
{
	

	const int mapy = MAP_OFFS_Y-(int)(MAP_SCALE*startY);
	const int mapx = MAP_OFFS_X+(int)(MAP_SCALE*startX);

	int foundUncharted = 0;
	int nearestUnchartedX = INT_MAX/4;
	int nearestUnchartedY = INT_MAX/4;
	int distanceToNearestUncharted = 0;

	
	cvZero(maptest);

	
	scanlinerange_t range;
	int unchartedCount = buildScanLine(mapx, mapy, &range);
	

	if (unchartedCount != 0)
	{
		foundUncharted += unchartedCount;

		if (range.leftUncharted)
		{
			distanceToNearestUncharted = getNearest(mapx, mapy, range.startx, range.y, nearestUnchartedX, nearestUnchartedY);
		}
		if (range.rightUncharted)
		{
			distanceToNearestUncharted = getNearest(mapx, mapy, range.endx, range.y, nearestUnchartedX, nearestUnchartedY);
		}
	}

	
	scanlinerange_chain_t *head = (scanlinerange_chain_t*)malloc(sizeof(scanlinerange_chain_t));
	memcpy(&head->scanline, &range, sizeof(scanlinerange_t));
	head->next = (scanlinerange_chain_t*)0x0;
	scanlinerange_chain_t *tail = head;

	
	while (head != (scanlinerange_chain_t*)0x0)
	{
			
		const int startX = head->scanline.startx;
		const int endX   = head->scanline.endx;
		const int y      = head->scanline.y;

		
		foundUncharted += extendScanLine(startX, endX, y-1, mapx, mapy, &tail, 
										 &nearestUnchartedX, &nearestUnchartedY, &distanceToNearestUncharted);
		
		
		foundUncharted += extendScanLine(startX, endX, y+1, mapx, mapy, &tail, 
										 &nearestUnchartedX, &nearestUnchartedY, &distanceToNearestUncharted);

		
		scanlinerange_chain_t* next = head->next;
		memset(head, 0, sizeof(scanlinerange_chain_t));
		free(head);
		head = next;
	}

	
	assert (head == (scanlinerange_chain_t*)0x0);

	
	if (foundUncharted)
	{
		if (outNearestX != (double*)0 && outNearestY != (double*)0)
		{
			*outNearestX = (nearestUnchartedX-MAP_OFFS_X)/MAP_SCALE;
			*outNearestY = (MAP_OFFS_Y-nearestUnchartedY)/MAP_SCALE;
		}
	}

#if VERBOSE
	if (foundUncharted)
	{
		printf("%d unkartierte. Nähester: x=%d, y=%d (Distanz: %d)\n", 
			foundUncharted, nearestUnchartedX, nearestUnchartedY, distanceToNearestUncharted);

		if (outNearestX != (double*)0 && outNearestY != (double*)0)
		{
			*outNearestX = (nearestUnchartedX-MAP_OFFS_X)/MAP_SCALE;
			*outNearestY = (MAP_OFFS_Y-nearestUnchartedY)/MAP_SCALE;
		}
	}
	else
	{
		printf("Keine unkartierten Punkte gefunden.\n");
	}
#endif

	return foundUncharted;
}

