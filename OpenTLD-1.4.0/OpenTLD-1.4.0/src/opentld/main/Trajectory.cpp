/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#include "Trajectory.h"

#include <cstdio>

/**
 * @author Clemens Korner
 */

using namespace std;

namespace tld
{

Trajectory::Trajectory()
{
}

Trajectory::~Trajectory()
{
}

void Trajectory::init(std::size_t length)
{
	m_length = length;
	m_track_positions = vector<CvPoint>();
	m_track_colors = vector<CvScalar>();
}

void Trajectory::addPoint(CvPoint point, CvScalar color)
{
	size_t length = m_track_positions.size();

	// vectors aren't full
	if(length < m_length)
	{
		m_track_positions.push_back(point);
		m_track_colors.push_back(color);
	}
	else
	{
		// push element to the end
		m_track_positions.push_back(point);
		m_track_colors.push_back(color);

		// drop first element
		m_track_positions.erase(m_track_positions.begin());
		m_track_colors.erase(m_track_colors.begin());
	}
}

void Trajectory::drawTrajectory(IplImage * image)
{
	CvPoint tempPoint;
	bool needSecondPoint = false;

	for(size_t i = 0; i < m_track_positions.size(); i++)
	{
		// try to find 1. point of the line
		if((!needSecondPoint)&&(m_track_positions[i].x != -1)) {
			tempPoint = m_track_positions[i];
			needSecondPoint = true;
		// try to find 2. point of the line
		}
		else if(needSecondPoint&&(m_track_positions[i].x != -1))
		{
			cvLine(image, tempPoint, m_track_positions[i], m_track_colors[i], 2);
			tempPoint = m_track_positions[i];
		}
	}
}

void Trajectory::faceCoordinates(TLD * tld, char * coordinatesFile)
{
	CvPoint center,rcenter;
		
	CvPoint center = cvPoint(tld->currBB->x+tld->currBB->width/2, tld->currBB->y+tld->currBB->height/2); // Determine the center of the bounding box
	CvPoint rcenter = cvPoint(tld->detectorCascade->imgWidth/2-center->x,tld->detectorCascade->imgHeight/2-center->y); //relative coordinates of the bounding box center to the center of the view
	fprintf(coordinatesFile, " x - %d : y - %d\n", rcenter->x, rcenter->y); // Write it to a file
	
}

}
