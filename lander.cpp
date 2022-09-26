// Mars lander simulator
// Version 1.10
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2017

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "lander.h"

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    double error, p_out, weight, delta;
    static const double Kh = 0.03, Kp = 1, target_speed = 0.5;
    //fuel consumed is 26.3L in case 1
    //e = -(0.5 + Kh * h + v.er)
    error = -(target_speed + Kh * (position.abs() - MARS_RADIUS) + velocity * position.norm());
    //p_out = Kp * e
    p_out = Kp * error;
    weight = (GRAVITY * MARS_MASS/ position.abs2()) * fuel * FUEL_CAPACITY * FUEL_DENSITY;
    delta = weight / MAX_THRUST;
    if (p_out < -delta)
    {
      throttle = 0.0;
    }
    else if (p_out < (1 - delta))
    {
      throttle = delta + p_out;
    }
    else
    {
       throttle = 1.0;
    }
}


void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  vector3d a, new_position, acceleration_gravity, thrust, drag;
  double mass;
  static vector3d previous_position;
  if (simulation_time == 0.0)
  {
    //resultant acceleration
    acceleration_gravity = -GRAVITY * MARS_MASS * position.norm() / position.abs2();
    thrust = thrust_wrt_world();
    drag = -atmospheric_density(position) * DRAG_COEF_LANDER * 3.1415 * pow(LANDER_SIZE, 2) * velocity.abs2() * velocity.norm();
    mass = fuel * FUEL_CAPACITY * FUEL_DENSITY;
    a = acceleration_gravity + (thrust + drag) / mass;
    //first point using Euler integration
    previous_position = position;
    position = previous_position + velocity * delta_t + 0.5 * pow(delta_t, 2) * a;
    velocity = velocity + delta_t * a;
  }
  else
  {
  //resultant acceleration
  acceleration_gravity = -GRAVITY * MARS_MASS * position.norm() / position.abs2();
  thrust = thrust_wrt_world();
  drag = -atmospheric_density(position) * DRAG_COEF_LANDER * 3.1415 * pow(LANDER_SIZE, 2) * velocity.abs2() * velocity.norm();
  mass = fuel * FUEL_CAPACITY * FUEL_DENSITY;
  a = acceleration_gravity + (thrust + drag) / mass;
  //rest using verlet integration
  new_position = 2 * position - previous_position + a * delta_t * delta_t;
  velocity = (new_position - previous_position) / (2 * delta_t);
  previous_position = position;
  position = new_position;
  }

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
	// a circular equatorial orbit
	position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
	velocity = vector3d(0.0, -3247.087385863725, 0.0);
	orientation = vector3d(0.0, 90.0, 0.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = false;
	autopilot_enabled = false;
	break;

  case 1:
	// a descent from rest at 10km altitude
	position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
	velocity = vector3d(0.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = true;
	break;

  case 2:
	// an elliptical polar orbit
	position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
	velocity = vector3d(3500.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = false;
	autopilot_enabled = false;
	break;

  case 3:
	// polar surface launch at escape velocity (but drag prevents escape)
	position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
	velocity = vector3d(0.0, 0.0, 5027.0);
	orientation = vector3d(0.0, 0.0, 0.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = false;
	autopilot_enabled = false;
	break;

  case 4:
	// an elliptical orbit that clips the atmosphere each time round, losing energy
	position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
	velocity = vector3d(4000.0, 0.0, 0.0);
	orientation = vector3d(0.0, 90.0, 0.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = false;
	autopilot_enabled = false;
	break;

  case 5:
	// a descent from rest at the edge of the exosphere
	position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
	velocity = vector3d(0.0, 0.0, 0.0);
	orientation = vector3d(0.0, 0.0, 90.0);
	delta_t = 0.1;
	parachute_status = NOT_DEPLOYED;
	stabilized_attitude = true;
	autopilot_enabled = false;
	break;

  case 6:
  position = vector3d(0.0, -(MARS_RADIUS + 500.0), 0.0);
  velocity = vector3d(0.0, 0.0, 0.0);
  orientation = vector3d(0.0, 0.0, 90.0);
  delta_t = 0.01;
  parachute_status = NOT_DEPLOYED;
  stabilized_attitude = true;
  autopilot_enabled = true;
	break;

  case 7:
  position = vector3d(0.0, -(MARS_RADIUS + 510.0), 0.0);
  velocity = vector3d(0.0, 0.0, 0.0);
  orientation = vector3d(0.0, 0.0, 90.0);
  delta_t = 0.01;
  parachute_status = NOT_DEPLOYED;
  stabilized_attitude = true;
  autopilot_enabled = true;
	break;

  case 8:
  position = vector3d(0.0, -(MARS_RADIUS + 700.0), 0.0);
  velocity = vector3d(0.0, 0.0, 0.0);
  orientation = vector3d(0.0, 0.0, 90.0);
  delta_t = 0.01;
  parachute_status = NOT_DEPLOYED;
  stabilized_attitude = true;
  autopilot_enabled = true;
	break;

  case 9:
	break;

  }

}


