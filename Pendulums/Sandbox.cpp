#include "Sandbox.h"

void Sandbox::run()
{

	running = true;
	show_render = true;
	logger = false;
	initialize();
	std::cout << "Enter max frame: ";
	std::cin >> max_frame;

	while (running) {
		
		update();
		
		if (show_render) {
			render();
			if (cv::waitKey(100 / 60) == 27) {
				running = false;
			}
		}

		if (run_number >= max_frame) {
			running = false;
		}
		
		
		
		
	}

	cv::destroyAllWindows();

	store();

	return;
			
}

void Sandbox::initialize()
{
	g = 9.8;
	l = 1;
	m = 0.010;

	N = 1;

	for (int i = 0; i < N; i++)
	{

		std::cout << "Initialising Pendulum " << i + 1 << std::endl;
		std::vector<double> temp_theta;
		std::vector<double> temp_diff1_theta;
		std::vector<double> temp_diff2_theta;

		double theta_val = floor((rand() % 100) * 0.005 * 1000) / 1000;

		

		temp_theta.push_back(theta_val);
		temp_diff1_theta.push_back(0.0);
		temp_diff2_theta.push_back(0.0);


		theta.push_back(temp_theta);
		diff1_theta.push_back(temp_diff1_theta);
		diff2_theta.push_back(temp_diff2_theta);
		
		std::cout << "Initial Angle: " << theta[i].back() << std::endl;
		std::cout << "Angular Velocity: " << diff1_theta[i].back() << std::endl;
		std::cout << "Angular Acceleration: " << diff2_theta[i].back() << std::endl;
		std::cout << "__________________________________________________" << std::endl;

	}

	t.push_back(0.0);
	dt = 0.01;



	W_HEIGHT = 1000; //HEIGHT
	W_WIDTH = 1000; //WIDTH

	image = cv::Mat::zeros(W_HEIGHT, W_WIDTH, CV_8UC3);
	blank = cv::Mat::zeros(W_HEIGHT, W_WIDTH, CV_8UC3);

	run_number = 0;

	

}

void Sandbox::update()
{
	std::cout << "Run Number: " << run_number << std::endl;
	

	for (int i = 0; i < N; i++)
	{

		temp_grav_term = 0;
		below_counter_terms = 0;
		above_counter_terms = 0;

		

		temp_grav_term = -(g / l) * sin(theta[i][run_number]);

		if (i > 0) {
			for (int j = 0; j < i; j++)
			{
				below_counter_terms += pow(diff1_theta[j][run_number], 2) * sin(theta[i][run_number] - theta[j][run_number]);
				below_counter_terms += diff2_theta[j][run_number] * cos(theta[i][run_number] - theta[j][run_number]);
			}
		}

		if (N > 1) {
			for (int j = i + 1; j < N; j++)
			{
				double coeff = (N - j) / (N - i);
				above_counter_terms += coeff * (pow(diff1_theta[j][run_number], 2) * sin(theta[i][run_number] - theta[j][run_number]));
				above_counter_terms += coeff * (diff2_theta[j][run_number] * cos(theta[i][run_number] - theta[j][run_number]));
			}
		}
		
		diff2_theta[i].push_back(temp_grav_term - below_counter_terms - above_counter_terms);
		diff1_theta[i].push_back(diff1_theta[i][run_number] + diff2_theta[i].back() * dt);
		theta[i].push_back(theta[i][run_number] + diff1_theta[i].back() * dt);
		if (logger) {
			std::cout << "Pendulum " << i + 1 << std::endl;
			std::cout << "Angle: " << theta[i].back() << std::endl;
			std::cout << "Angular Velocity: " << diff1_theta[i].back() << std::endl;
			std::cout << "Angular Acceleration: " << diff2_theta[i].back() << std::endl;
		}
		
	}

	t.push_back(t.back() + dt);


	run_number += 1;
}

void Sandbox::render()
{
	/*
	(0,0) at (0, half width)
	(0, -half width) at top left

	scaling: 

	Point(X, Y)
	X -> l sin(theta)
	Y -> l cos(theta)

	height <=> N * l * 1.10  
	length of l => height / (N * 1.10)

	*/
	

	double line_render_length = W_HEIGHT / (N * 1.10);

	render_ball_pos[0] = W_WIDTH / 2;
	render_ball_pos[1] = 0.0;

	for (int i = 0; i < N; i++)
	{
		render_ball_pos_prev[0] = render_ball_pos[0];
		render_ball_pos_prev[1] = render_ball_pos[1];

		render_ball_pos[0] += line_render_length * sin(theta[i].back());
		render_ball_pos[1] += line_render_length * cos(theta[i].back());
		cv::line(image, cv::Point(render_ball_pos_prev[0], render_ball_pos_prev[1]), cv::Point(render_ball_pos[0], render_ball_pos[1]), (200, 200, 200), 2);
		cv::circle(image, cv::Point(render_ball_pos[0], render_ball_pos[1]), 0.2 / 0.1, cv::Scalar(100, 255, 128), -100);
	}




	
	cv::imshow("Display Window", image);
	
	image = blank.clone();
	
	


}

void Sandbox::store()
{
	std::ofstream outfile("Pendulum.csv");

	outfile << "N,frame,theta,angular velocity,angular acc,time" << std::endl;

	for (int j = 0; j < run_number; j++) {
		for (int i = 0; i < N; i++)
		{
			outfile << i << "," << j << "," << theta[i][j] << "," << diff1_theta[i][j] << "," << diff2_theta[i][j] << "," << t[j] << std::endl;
		}
	}
	outfile.close();
}
