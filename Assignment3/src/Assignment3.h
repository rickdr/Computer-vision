/*
 * Assignment3.h
 *
 *  Created on: Nov 13, 2013
 *      Author: coert
 */

#ifndef VOXELRECONSTRUCTION_H_
#define VOXELRECONSTRUCTION_H_

#include <string>
#include <vector>

#include "controllers/Camera.h"

namespace nl_uu_science_gmt
{

class Assignment3
{
	const std::string m_data_path;
	const int m_cam_views_amount;

	std::vector<Camera*> m_cam_views;

public:
	Assignment3(const std::string &, const int);
	virtual ~Assignment3();

	static void showKeys();

	void run(int, char**);
};

} /* namespace nl_uu_science_gmt */

#endif /* VOXELRECONSTRUCTION_H_ */
