/*
 * Scene3DRenderer.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include <opencv2/opencv.hpp>

#include "Scene3DRenderer.h"

#include <stddef.h>
#include <cassert>
#include <string>

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{
/**
 * Scene properties class (mostly called by Glut)
 */

static Scene3DRenderer* gInstance = NULL;

void onMouse(int aevent, int x, int y, int flags, void* userdata)
{
	if (gInstance != NULL)
	{
		gInstance->OnMouse(aevent, x, y, flags, userdata);
	}
}

void onModeChange(int newmode, void* userdata)
{
	if (gInstance != NULL)
	{
		gInstance->OnModeChange(newmode, userdata);
	}
}

void onFrameChange(int newframe, void* userdata)
{
	if (gInstance != NULL)
	{
		gInstance->OnFrameChange(newframe, userdata);
	}
}

void Scene3DRenderer::OnMouse(int aevent, int x, int y, int flags, void* userdata)
{
	if  ( aevent == EVENT_LBUTTONDOWN )
	{
		//cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if  ( aevent == EVENT_RBUTTONDOWN )
	{
		//cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if  ( aevent == EVENT_MBUTTONDOWN )
	{
		//cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if ( aevent == EVENT_MOUSEMOVE )
	{
		//cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
	}

	
	if (_mode == 1)
	{
		if (x > _width) // if the foreground screen was pressed
		{
			if  ( aevent == EVENT_LBUTTONDOWN )
			{
				ColorDesiredForegroundValue(_current_frame, x - 640, y, 0);
			}
			else if  ( aevent == EVENT_RBUTTONDOWN )
			{
				ColorDesiredForegroundValue(_current_frame, x - 640, y, 255);
			}
		}
	}
}

void Scene3DRenderer::OnModeChange(int newmode, void* userdata)
{
	if (newmode == 0)
	{
		_adjusted_frames.clear();
		_pixelsWereModified = false;
	}
	if (newmode == 1)
	{
		AddAdjustedFrame(_current_frame);
	}
	if (newmode == 2)
	{
		if (_pixelsWereModified)
		{
			_findNewOptimalHSVValues = true;
		}
	}
}

void Scene3DRenderer::OnFrameChange(int newframe, void* userdata)
{
	AddAdjustedFrame(newframe);
}

Scene3DRenderer::Scene3DRenderer(Reconstructor &r, const vector<Camera*> &cs) :
		_reconstructor(r), _cameras(cs), _num(4), _sphere_radius(1850)
{
	gInstance = this;
	_width = 640;
	_height = 480;
	_quit = false;
	_paused = false;
	_rotate = false;
	_camera_view = true;
	_show_volume = true;
	_show_grd_flr = true;
	_show_cam = true;
	_show_org = true;
	_show_arcball = false;
	_show_info = true;
	_fullscreen = false;
	_pixelsWereModified = false;
	_findNewOptimalHSVValues = false;

	// Read the checkerboard properties (XML)
	FileStorage fs;
	fs.open(_cameras.front()->getDataPath() + ".." + string(PATH_SEP) + General::CBConfigFile, FileStorage::READ);
	if (fs.isOpened())
	{
		fs["CheckerBoardWidth"] >> _board_size.width;
		fs["CheckerBoardHeight"] >> _board_size.height;
		fs["CheckerBoardSquareSize"] >> _square_side_len;
	}
	fs.release();

	_current_camera = 0;
	_previous_camera = 0;

	_number_of_frames = _cameras.front()->getFramesAmount();
	_current_frame = 0;
	_previous_frame = -1;

	const int H = 0;
	const int S = 0;
	const int V = 0;
	_h_threshold = H;
	_ph_threshold = H;
	_s_threshold = S;
	_ps_threshold = S;
	_v_threshold = V;
	_pv_threshold = V;
	_before_erosion_threshold = 0;
	_erosion_threshold = 0;
	_mode = 0;
	_brush_size = 5;

	createTrackbar("Frame", VIDEO_WINDOW, &_current_frame, _number_of_frames - 2, onFrameChange);
	createTrackbar("bE", VIDEO_WINDOW, &_before_erosion_threshold, 8);
	createTrackbar("H", VIDEO_WINDOW, &_h_threshold, 255);
	createTrackbar("S", VIDEO_WINDOW, &_s_threshold, 255);
	createTrackbar("V", VIDEO_WINDOW, &_v_threshold, 255);
	createTrackbar("aE", VIDEO_WINDOW, &_erosion_threshold, 8);
	
	// this _mode variable is going to be used to fake menu buttons, go to a different 'mode' to apply something or show a different type of screen
	// _mode 0 is standard mode, ie. showing the foreground
	// _mode 1 allows the user to invert pixels to 'desirable' values, if this happens it is assumed that all the pixels that were not modified are also desirable
	// _mode 2 this mode sets the HSV values to best matching values depending on the 'desirable' values.
	createTrackbar("Mode", VIDEO_WINDOW, &_mode, 2, onModeChange); 
	createTrackbar("Brush", VIDEO_WINDOW, &_brush_size, 10);
	
	setMouseCallback(VIDEO_WINDOW, onMouse);

	createFloorGrid();
	setTopView();
}

/**
 * Free the memory of the floor_grid pointer vector
 */
Scene3DRenderer::~Scene3DRenderer()
{
	for (size_t f = 0; f < _floor_grid.size(); ++f)
		for (size_t g = 0; g < _floor_grid[f].size(); ++g)
			delete _floor_grid[f][g];
}

/**
 * Process the current frame on each camera
 */
bool Scene3DRenderer::processFrame()
{
	if (_mode == 0)
	{
		for (size_t c = 0; c < _cameras.size(); ++c)
		{
			if (_current_frame == _previous_frame + 1)
			{
				_cameras[c]->advanceVideoFrame();
			}
			else if (_current_frame != _previous_frame)
			{
				_cameras[c]->getVideoFrame(_current_frame);
			}
			assert(_cameras[c] != nullptr);
			processForeground(_cameras[c]);
		}
	}
	return true;
}

bool Scene3DRenderer::update()
{
	if (_mode == 1)
	{
		// mode for clicking on extraneous pixels
		if (GetAdjustedFrame(_current_frame) != NULL)
		{
			_cameras[0]->setForegroundImage(GetAdjustedFrame(_current_frame)->_Mat);
		}
	}
	else if (_mode == 2)
	{
		// mode setting new HSV values
	}

	if (_findNewOptimalHSVValues)
	{
		FindNewOptimalHSVValues();
	}

	return true;
}

/**
 * Separate the background from the foreground
 * ie.: Create an 8 bit image where only the foreground of the scene is white
 */
void Scene3DRenderer::processForeground(Camera* camera)
{
	assert(!camera->getFrame().empty());

	Mat hsv_image;
	cvtColor(camera->getFrame(), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

	vector<Mat> channels;
	split(hsv_image, channels);  // Split the HSV-channels for further analysis

	Mat foreground;

	// Before HSV background subtraction erosion
	if (_before_erosion_threshold > 0)
	{
		// Background subtraction
		CalcHSVFilteredForeground(camera, channels, foreground, 0, 0, 0);

		for (int x = 1; x < foreground.cols-1; x++)
		{
			for (int y = 1; y < foreground.rows-1; y++)
			{
				if (foreground.at<byte>(y, x) > 0)
				{
					byte neighboursOccupied = 0;
					if (foreground.at<byte>(y-1, x-1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y, x-1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y+1, x-1) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y-1, x) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y+1, x) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y-1, x+1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y, x+1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y+1, x+1) > 0) neighboursOccupied++;

					if (neighboursOccupied <= _before_erosion_threshold)
					{
						// put the channels hsv values for this pixel equal to the background hsv values, so that no change is detected later on
						channels[0].at<byte>(y, x) = camera->getBgHsvChannels().at(0).at<byte>(y, x);
						channels[1].at<byte>(y, x) = camera->getBgHsvChannels().at(1).at<byte>(y, x);
						channels[2].at<byte>(y, x) = camera->getBgHsvChannels().at(2).at<byte>(y, x);
					}
				}
			}
		}
	}

	// Background subtraction
	CalcHSVFilteredForeground(camera, channels, foreground, _h_threshold, _s_threshold, _v_threshold);

	// Apply erosion
	if (_erosion_threshold > 0)
	{
		// Erosion can only be done in conjunction with a copy of the original image
		Mat foregroundCopy(foreground.rows, foreground.cols, foreground.type());
		foreground.copyTo(foregroundCopy);

		for (int x = 1; x < foreground.cols-1; x++)
		{
			for (int y = 1; y < foreground.rows-1; y++)
			{
				if (foreground.at<byte>(y, x) > 0)
				{
					byte neighboursOccupied = 0;
					if (foreground.at<byte>(y-1, x-1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y, x-1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y+1, x-1) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y-1, x) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y+1, x) > 0) neighboursOccupied++;

					if (foreground.at<byte>(y-1, x+1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y, x+1) > 0) neighboursOccupied++;
					if (foreground.at<byte>(y+1, x+1) > 0) neighboursOccupied++;

					if (neighboursOccupied <= _erosion_threshold)
					{
						foregroundCopy.at<byte>(y, x) = 0;
					}
					else
					{
						foregroundCopy.at<byte>(y, x) = 255;
					}
				}
			}
		}
		camera->setForegroundImage(foregroundCopy);
	}
	else
	{
		camera->setForegroundImage(foreground);
	}
}

/**
 * Set currently visible camera to the given camera id
 */
void Scene3DRenderer::setCamera(int camera)
{
	_camera_view = true;

	if (_current_camera != camera)
	{
		_previous_camera = _current_camera;
		_current_camera = camera;
		_arcball_eye.x = _cameras[camera]->getCameraPlane()[0].x;
		_arcball_eye.y = _cameras[camera]->getCameraPlane()[0].y;
		_arcball_eye.z = _cameras[camera]->getCameraPlane()[0].z;
		_arcball_up.x = 0.0f;
		_arcball_up.y = 0.0f;
		_arcball_up.z = 1.0f;
	}
}

/**
 * Set the 3D scene to bird's eye view
 */
void Scene3DRenderer::setTopView()
{
	_camera_view = false;
	if (_current_camera != -1) _previous_camera = _current_camera;
	_current_camera = -1;

	_arcball_eye = vec(0.0f, 0.0f, 10000.0f);
	_arcball_centre = vec(0.0f, 0.0f, 0.0f);
	_arcball_up = vec(0.0f, 1.0f, 0.0f);
}

/**
 * Create a LUT for the floor grid
 */
void Scene3DRenderer::createFloorGrid()
{
	const int size = _reconstructor.getSize();
	const int z_offset = 3;

	// edge 1
	vector<Point3i*> edge1;
	for (int y = -size * _num; y <= size * _num; y += size)
		edge1.push_back(new Point3i(-size * _num, y, z_offset));

	// edge 2
	vector<Point3i*> edge2;
	for (int x = -size * _num; x <= size * _num; x += size)
		edge2.push_back(new Point3i(x, size * _num, z_offset));

	// edge 3
	vector<Point3i*> edge3;
	for (int y = -size * _num; y <= size * _num; y += size)
		edge3.push_back(new Point3i(size * _num, y, z_offset));

	// edge 4
	vector<Point3i*> edge4;
	for (int x = -size * _num; x <= size * _num; x += size)
		edge4.push_back(new Point3i(x, -size * _num, z_offset));

	_floor_grid.push_back(edge1);
	_floor_grid.push_back(edge2);
	_floor_grid.push_back(edge3);
	_floor_grid.push_back(edge4);
}

void Scene3DRenderer::FindNewOptimalHSVValues()
{
	// Search for the optimal HSV values based on a calculated image and the desired image.

	int similarityGrid[21][21][21];
	for (int h1 = 0; h1 < 21; h1++) { for(int j = 0; j < 21; j++) { for(int k = 0; k < 21; k++) { similarityGrid[h1][j][k] = 0; } } } // initialize to 0

	int counter = 0;
	int similarityscore = 0;

	for (unsigned int i = 0; i < _adjusted_frames.size(); i++)
	{
		if (_adjusted_frames[i]->_Changed == true) // only use the frame if it was adjusted
		{
			counter++;

			cv::Mat* adjustedFrame = & _adjusted_frames[i]->_Mat;

			Mat hsv_image;
			cvtColor(_cameras[0]->getVideoFrame(_adjusted_frames[i]->_Index), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

			vector<Mat> channels;
			split(hsv_image, channels);  // Split the HSV-channels for further analysis

			int hs = _h_threshold - 0;
			int he = _h_threshold + 1;

			int ss = _s_threshold - 0;
			int se = _s_threshold + 1;

			int vs = _v_threshold - 0;
			int ve = _v_threshold + 2;

			for (int H = hs, hi = 0; H <= he; H++, hi++)
			{
				for (int S = ss, si = 0; S <= se; S++, si++)
				{
					for (int V = vs, vi = 0; V <= ve; V++, vi++)
					{
						cv::Mat calculatedFrame;

						// background subtraction
						CalcHSVFilteredForeground(_cameras[0], channels, calculatedFrame, H, S, V);

						for (int x1 = 0; x1 < adjustedFrame->cols; x1++)
						{
							for (int y1 = 0; y1 < adjustedFrame->rows; y1++)
							{
								if (adjustedFrame->at<byte>(y1, x1) == calculatedFrame.at<byte>(y1, x1))
								{
									similarityGrid[hi][si][vi]++;
								}
							}
						}
					}
				}
			}
		}
	}

	int highestHSV_h = -1;
	int highestHSV_s = -1;
	int highestHSV_v = -1;
	int highestHSV_similarity = -1;
	// find highest HSV
	for (int i = 0; i < 21; i++)
	{ 
		for(int j = 0; j < 21; j++) 
		{ 
			for(int k = 0; k < 21; k++) 
			{ 
				if (similarityGrid[i][j][k] > highestHSV_similarity)
				{
					highestHSV_similarity = similarityGrid[i][j][k];
					highestHSV_h = i;
					highestHSV_s = j;
					highestHSV_v = k;
				}
			} 
		} 
	}

	// we now have the most optimal HSV value for the desired result

	int new_h_threshold = _h_threshold + highestHSV_h;

	int new_s_threshold = _s_threshold + highestHSV_s;

	int new_v_threshold = _v_threshold + highestHSV_v;

	_h_threshold = new_h_threshold;
	_s_threshold = new_s_threshold;
	_v_threshold = new_v_threshold;

	_findNewOptimalHSVValues = false;
}

void Scene3DRenderer::AddAdjustedFrame(int aframe)
{
	if (GetAdjustedFrame(aframe) == NULL)
	{
		// if it's not there, we need to create it.
		assert(!_cameras[0]->getVideoFrame(aframe).empty());

		Mat hsv_image;
		cvtColor(_cameras[0]->getVideoFrame(aframe), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

		vector<Mat> channels;
		split(hsv_image, channels);  // Split the HSV-channels for further analysis

		Mat tmp, foreground, background;

		// Init H
		absdiff(channels[0], _cameras[0]->getBgHsvChannels().at(0), tmp);
		threshold(tmp, foreground, _h_threshold, 255, CV_THRESH_BINARY);

		// Init S
		absdiff(channels[1], _cameras[0]->getBgHsvChannels().at(1), tmp);
		threshold(tmp, background, _s_threshold, 255, CV_THRESH_BINARY);
		bitwise_and(foreground, background, foreground);

		// Init V
		absdiff(channels[2], _cameras[0]->getBgHsvChannels().at(2), tmp);
		threshold(tmp, background, _v_threshold, 255, CV_THRESH_BINARY);
		bitwise_or(foreground, background, foreground);
		
		AdjustedFrame* matWithIndex = new AdjustedFrame(foreground, aframe); 
		_adjusted_frames.push_back(matWithIndex);
	}
}

void Scene3DRenderer::ColorDesiredForegroundValue(int aframe, int x, int y, byte color)
{
	AddAdjustedFrame(aframe);

	// find the frame we want to change and invert the pixel
	ColorNeighbouringPixels(GetAdjustedFrame(aframe)->_Mat, x, y, color);

	_pixelsWereModified = true;
	GetAdjustedFrame(aframe)->_Changed = true;
}

void Scene3DRenderer::ColorNeighbouringPixels(cv::Mat& _mat, int x, int y, byte color)
{
	if (_brush_size == 0) _brush_size = 1;
	int s = _brush_size - 1;
	int sx = (x + s) > 0 ? x - s : 0;
	int sy = (y - s) > 0 ? y - s : 0;
	int ex = (x + s) < 640 ? x + s : 639;
	int ey = (y + s) < 480 ? y + s : 479;

	for (int i = sx; i <= ex; i++)
	{
		for (int j = sy; j <= ey; j++)
		{
			_mat.at<byte>(j, i) = color;
		}
	}
}

AdjustedFrame* Scene3DRenderer::GetAdjustedFrame(int frame)
{
	for(unsigned int i = 0; i < _adjusted_frames.size(); i++)
	{
		if (_adjusted_frames[i]->_Index == frame)
		{
			return _adjusted_frames[i];
		}
	}

	return NULL;
}

void Scene3DRenderer::CalcHSVFilteredForeground(Camera* camera, vector<Mat>& channels, cv::Mat& aDst, char H, char S, char V)
{
	Mat tmp, background;

	absdiff(channels[0], camera->getBgHsvChannels().at(0), tmp);
	threshold(tmp, aDst, H, 255, CV_THRESH_BINARY);

	absdiff(channels[1], camera->getBgHsvChannels().at(1), tmp);
	threshold(tmp, background, S, 255, CV_THRESH_BINARY);
	bitwise_and(aDst, background, aDst);

	absdiff(channels[2], camera->getBgHsvChannels().at(2), tmp);
	threshold(tmp, background, V, 255, CV_THRESH_BINARY);
	bitwise_or(aDst, background, aDst);
}

} /* namespace nl_uu_science_gmt */
