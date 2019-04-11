// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <chrono>
#include <ctime>

#include <thread>
#include <mutex>
#include <cstring>
#include <cmath>

#include <unistd.h>

#include "cArduino.h"


#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstring>
#include <mutex>

double PI = 3.141592653589793238462643383;

struct short3
{
    uint16_t x, y, z;
};


struct float3 { 
    float x, y, z; 
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};



 float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98;
    bool first = true;
    // Keeps the arrival time of previous gyro frame
double last_ts_gyro = 0;

 void process_gyro(rs2_vector gyro_data, double ts)
    {
        if (first) // On the first iteration, use only data from accelerometer to set the camera's initial position
        {
            last_ts_gyro = ts;
            return;
        }
        // Holds the change in angle, as calculated from gyro
        float3 gyro_angle;

        // Initialize gyro_angle with data from gyro
        gyro_angle.x = gyro_data.x; // Pitch
        gyro_angle.y = gyro_data.y; // Yaw
        gyro_angle.z = gyro_data.z; // Roll

        // Compute the difference between arrival times of previous and current gyro frames
        double dt_gyro = (ts - last_ts_gyro) / 1000.0;
        last_ts_gyro = ts;

        // Change in angle equals gyro measures * time passed since last measurement
        gyro_angle = gyro_angle * dt_gyro;

        // Apply the calculated change of angle to the current angle (theta)
        std::lock_guard<std::mutex> lock(theta_mtx);
        theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
    }

    void process_accel(rs2_vector accel_data)
    {
        // Holds the angle as calculated from accelerometer data
        float3 accel_angle;

        // Calculate rotation angle from accelerometer data
        accel_angle.z = atan2(accel_data.y, accel_data.z);
        accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

        // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
        std::lock_guard<std::mutex> lock(theta_mtx);
        if (first)
        {
            first = false;
            theta = accel_angle;
            // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
            theta.y = PI;
        }
        else
        {
            /* 
            Apply Complementary Filter:
                - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                  that are steady over time, is used to cancel out drift.
                - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
            */
            theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
            theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
        }
    }
    
    // Returns the current rotation angle
    float3 get_theta()
    {
        std::lock_guard<std::mutex> lock(theta_mtx);
        return theta;
    }





std::string fToString(float f){

    std::ostringstream strs;
	strs << std::setprecision(3) << std::fixed << f;
	return strs.str();

}

int main(int argc, char * argv[]) try
{

//    const double DIST_PULSE = 0.05; 

    //std::string sensPort = argv[1];
    
    //wheelSensor ws(sensPort);
    
    cArduino arduino(ArduinoBaundRate::B9600bps);
    
    rs2::pipeline pipe;
    rs2::config cfg;
    
    
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);  
  
    
    auto pf = pipe.start(cfg);
      
    rs2::device dev = pf.get_device();
    auto wheel_odom_snr = dev.first<rs2::wheel_odometer>();
    
    //std::ifstream calibrationFile("../calibration_odometry.json");
    std::ifstream calibrationFile("realsenset265/calibration_odometry.json");
    const std::string json_str((std::istreambuf_iterator<char>(calibrationFile)),
                      std::istreambuf_iterator<char>());
              
    const std::vector<uint8_t> wo_calib(json_str.begin(), json_str.end());
 
    wheel_odom_snr.load_wheel_odometery_config(wo_calib);
    

 
    std::string temp;
    float speed1 = 0;
    float speed2 = 0;
    std::string::size_type sz;

    int fd;
    std::string myfifo = "/tmp/myfifo";
    mkfifo(myfifo.c_str(), 0666);
    
    std::string writePipe = "0.000 0.000 0.000";
    
    while (true)
    {
        fd = open(myfifo.c_str(), O_WRONLY/* | O_NONBLOCK*/ );
        auto frames = pipe.wait_for_frames();
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
       
       
       auto g = frames.first_or_default(RS2_STREAM_GYRO);
       auto a = frames.first_or_default(RS2_STREAM_ACCEL);
       // Cast the frame that arrived to motion frame
        auto gyro_f = g.as<rs2::motion_frame>();
        auto accel_f = a.as<rs2::motion_frame>();
            // Get the timestamp of the current frame
        double ts = gyro_f.get_timestamp();
            // Get gyro measures
        rs2_vector gyro_data = gyro_f.get_motion_data();
            // Call function that computes the angle of motion based on the retrieved measures
        process_gyro(gyro_data, ts);
        
            // Get accelerometer measures
        rs2_vector accel_data = accel_f.get_motion_data();
            // Call function that computes the angle of motion based on the retrieved measures
        process_accel(accel_data);
        
        //std::cout<<" X -> "<<get_theta().x*180/PI <<" Y -> " << get_theta().y*180/PI << " Z -> " << get_theta().z*180/PI<<std::endl;
       // std::cout<<" X -> "<<pose_data.translation.x <<" Y -> " << pose_data.translation.y << " Z -> " << pose_data.translation.z<<std::endl;
        
	    temp = arduino.read();
//	    std::cout << temp << std::endl;
	  	std::this_thread::sleep_for(std::chrono::milliseconds(250));	
	    if(temp.size() > 8){
		
		try{	
	//	std::cout << temp << std::endl;
		speed1 = std::stof (temp,&sz);
		speed2 = std::stof (temp.substr(sz));
		} catch(const std::exception& e){
			std::cout <<"exception:("<< e.what() <<std::endl;
			speed1 = 0;
			speed2 = 0;
		}
		}
		
        bool b1 = wheel_odom_snr.send_wheel_odometry(0, f.get_frame_number(), {speed1,0,0});
        bool b2 = wheel_odom_snr.send_wheel_odometry(1, f.get_frame_number(), {speed2,0,0});

//	double dist = sqrt((pose_data.translation.x)*(pose_data.translation.x) + 
//							(pose_data.translation.y)*(pose_data.translation.y) + 
//							(pose_data.translation.z)*(pose_data.translation.z));
	
	
	writePipe = (fToString(pose_data.translation.x) + " " + fToString(pose_data.translation.z) + " " + fToString(get_theta().y));
	
	//writePipe = fToString(pose_data.translation.x) + " " + fToString(pose_data.translation.z);
	//cout << writePipe << 
	write(fd, writePipe.c_str(), 2+sizeof(fToString(pose_data.translation.x))+sizeof(fToString(pose_data.translation.x))+sizeof(fToString(get_theta().y)));
	close(fd);	   
	
	        
     //   std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
     //       pose_data.translation.y << " " << pose_data.translation.z << " (meters) "<<" "<< b1<<" " << b2 << " " << speed1 << " " << speed2<<std::endl;
    }
    
    unlink(myfifo.c_str());
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
  
