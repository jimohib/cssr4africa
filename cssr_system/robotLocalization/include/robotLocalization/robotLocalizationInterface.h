#ifndef ROBOT_LOCALIZATION_INTERFACE_H
#define ROBOT_LOCALIZATION_INTERFACE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctype.h>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

#define ROS_PACKAGE_NAME "cssr_system"
#define MAX_FILENAME_LENGTH 200

int circle_circle_intersection(double x0, double y0, double r0,
                              double x1, double y1, double r1,
                              double *xi, double *yi,
                              double *xi_prime, double *yi_prime);

int circle_centre(double x1, double y1,
                  double x2, double y2,
                  double alpha,
                  double *xc1, double *yc1,
                  double *xc2, double *yc2,
                  double *r);

void display_error_and_exit(const char error_message[]);
void prompt_and_exit(int status);
void prompt_and_continue();
void print_message_to_file(FILE *fp, const char message[]);

#endif