#pragma once

#include "include_header.h"

class Sandbox
{
public:

	double g; // gravitational acceleration
	double l; // length of string
	double m; // mass of pendulums

	//double drag_coeff;
	//double air_density;
	//double ref_area;
	//double radius_sphere;

	//double acc_grav;
	//double acc_drag;

	int N; // number of pendulums

	// 2D arrays of size N by run_number to hold values for every pendulum

	std::vector<std::vector<double>> theta;
	std::vector<std::vector<double>> diff1_theta;
	std::vector<std::vector<double>> diff2_theta;

	double temp_grav_term;
	double below_counter_terms;
	double above_counter_terms;

	std::vector<double> t = {};
	

	int run_number;

	double dt;
	bool running;
	bool logger;
	bool show_render;
	int max_frame;

	void run();
	cv::Mat image;
	cv::Mat blank;
	int W_WIDTH;
	int W_HEIGHT;

	double render_ball_pos[2];
	double render_ball_pos_prev[2];
private:
	void initialize();
	void update();
	void render();
	void store();
};

