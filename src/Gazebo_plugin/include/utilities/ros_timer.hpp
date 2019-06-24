/**
 * @file timer.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 * Modified by: Edgar Granados, 2019, ITAM.
 * 
 */
 
#ifndef SPARSE_ROS_TIMER_HPP
#define SPARSE_ROS_TIMER_HPP

// #include <sys/time.h>
#include <ros/ros.h>
/**
 * A class that can express current time as a single number and which can
 * also provide methods that measure elapsed time between consecutive calls.
 * 
 * @brief <b>A clock for measuring time using system time</b>
 * @authors Kostas Bekris
 * 
 */
class sys_ros_timer_t
{

  protected:
    /** @brief When the timer was started */
    ros::Time start;
    /** @brief When the timer finished */
    ros::Time finish;
    /** @brief How much time has elapsed since start */
    ros::Duration elapsed;

    double duration_to_double(ros::Duration duration);

  public:
    sys_ros_timer_t();
    virtual ~sys_ros_timer_t();

    /**
     * Gets the time in seconds
     * 
     * @brief Gets the time in seconds
     * @return The time in seconds
     */
    double get_time_in_secs();

    /**
     * Resets the timer
     * 
     * @brief Resets the timer
     */
    void reset();

    /**
     * Measures the timer and returns the value in seconds
     * 
     * @brief Measures the timer
     * @return The elapsed time in seconds
     */
    double measure();

    /**
     * Performs measure and reset
     * 
     * @brief Calls measure and reset
     * @return The elapsed time in seconds
     */
    double measure_reset();

    /**
     * Adds a delay to the clock
     * 
     * @brief Adds a delay to the clock
     * @param delay Determines how much delay is added
     */
    void add_delay_user_clock(double delay);
};

#endif