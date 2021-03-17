/*
 * Glut.h
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#ifndef GLUT_H_
#define GLUT_H_

#include "Reconstructor.h"

#ifdef _WIN32
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/core/core.hpp>
#include <stddef.h>
#include <vector>

#include "Camera.h"

#endif
#ifdef __linux__
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#define MOUSE_WHEEL_UP   3
#define MOUSE_WHEEL_DOWN 4

namespace nl_uu_science_gmt
{

class Scene3DRenderer;

class Glut
{
private:
	Scene3DRenderer &m_scene3d;

	static Glut* m_Glut;

	static void drawGrdGrid();
	static void drawCamCoord();
	static void drawVolume();
	static void drawArcball();
	static void drawVoxels();
	static void drawWCoord();
	static void drawInfo();

	static inline void perspectiveGL(
			GLdouble, GLdouble, GLdouble, GLdouble);

#ifdef _WIN32
	static void SetupPixelFormat(HDC hDC);
	static LRESULT CALLBACK WndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
#endif

	std::vector<std::vector<std::vector<Reconstructor::Voxel*>>> g_clusters;
	std::vector<std::vector<cv::Mat>> g_colors;
	std::vector<std::vector<cv::Mat>> g_histograms;

	std::vector<std::vector<std::vector<Reconstructor::Voxel*>>> g_clustered_voxels;
	std::vector<std::vector<std::vector<Reconstructor::Voxel*>>> g_path;
	bool tracking;

public:
	Glut(
			Scene3DRenderer &);
	virtual ~Glut();
	float point_distance(cv::Point2f point1, cv::Point2f point2);
	void cluster_voxels(bool init_models);

	void track_histograms();

#ifdef __linux__
	void initializeLinux(
			const char*, int, char**);
	static void mouse(
			int, int, int, int);
#endif
#ifdef _WIN32
	int initializeWindows(const char*);
	void mainLoopWindows();
#endif

	static void keyboard(
			unsigned char, int, int);
	static void motion(
			int, int);
	static void reshape(
			int, int);
	static void reset();
	static void idle();
	static void display();
	static void update(
			int);
	static void quit();

	Scene3DRenderer& getScene3d() const
	{
		return m_scene3d;
	}
};

} /* namespace nl_uu_science_gmt */

#endif /* GLUT_H_ */
