// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <cmath>

void autopilot_land(void)
// INSERT YOUR CODE
// Autopilot to adjust the engine throttle, parachute and attitude control
{
    double lander_mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel;
    vector3d g_mars = (-GRAVITY * MARS_MASS / position.abs2()) * position.norm();

    double altitude = position.abs() - MARS_RADIUS;

    stabilized_attitude = true;

    if (parachute_status == NOT_DEPLOYED && safe_to_deploy_parachute() && altitude < 40000) {
        parachute_status = DEPLOYED;
    }

    double kh = 0.02;
    double error = -(0.5 + kh * altitude + velocity * (position.norm()));

    double kp = 2;
    double p_out = kp * error;
    double delta = (GRAVITY * MARS_MASS * lander_mass / position.abs2()) / MAX_THRUST;

    if (p_out <= -delta) throttle = 0.0;
    else if (p_out >= 1 - delta) throttle = 1.0;
    else throttle = delta + p_out;
}

void autopilot_hover(void) {
    //INSERT YOUR CODE
    stabilized_attitude = true;
    float kd = 0.25;
    float kp = 0.01;

    // autopilot for hovering at the given altitude
    double lander_mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel;
    double altitude = position.abs() - MARS_RADIUS;
    double delta = (GRAVITY * MARS_MASS * lander_mass / position.abs2()) / MAX_THRUST;

    //target altitude
    double target_altitude = 600.0;
    double speed = -velocity * position.norm(); // downward +
    // PD controller
    throttle = delta + kp * (target_altitude - altitude) + kd * speed;
    if (throttle > 1.0) throttle = 1.0; else if (throttle < 0.0) throttle = 0.0;

}

//INSERT YOUR CODE
vector3d drag(void) {
    double area_lander, area_chute;
    //drag
    area_lander = 3.14 * pow(LANDER_SIZE, 2);
    area_chute = 3.14 * pow(2 * LANDER_SIZE, 2);
    if (parachute_status == DEPLOYED) {
        return -0.5 * atmospheric_density(position) * velocity.abs2() * (DRAG_COEF_CHUTE * area_chute + DRAG_COEF_LANDER * area_lander) * velocity.norm();
    }
    else {
        return -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * area_lander * velocity.abs2() * velocity.norm();
    }

}

//INSERT YOUR CODE
void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
    vector3d accel, new_position;
    static vector3d previous_position;

    // do not make this as a global variable
    double lander_mass = UNLOADED_LANDER_MASS + FUEL_CAPACITY * FUEL_DENSITY * fuel;
    vector3d g_mars = (-GRAVITY * MARS_MASS / position.abs2()) * position.norm();

    vector3d fg = lander_mass * g_mars;
    vector3d thr = thrust_wrt_world();
    vector3d drag_force = drag();

    // acceleration
    accel = (fg + thr + drag_force) / lander_mass;
    if (simulation_time == 0.0) {
        new_position = position + velocity * delta_t;
        velocity = accel * delta_t;
    }
    else {
        //position and velocity (Verlet)
        new_position = 2 * position - previous_position + pow(delta_t, 2) * accel;
        velocity = (1 / delta_t) * (new_position - position);
    }


    // update previous and current positions
    previous_position = position;
    position = new_position;


    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) autopilot_hover();

    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation(void)
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
        position = vector3d(1.2 * MARS_RADIUS, 0.0, 0.0);
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
        delta_t = 0.01; // hovering: 0.01, landing 0.1
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = true;
        break;

    case 2:
        // an elliptical polar orbit
        position = vector3d(0.0, 0.0, 1.2 * MARS_RADIUS);
        velocity = vector3d(3500.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 3:
        // polar surface launch at escape velocity (but drag prevents escape)
        position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
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
        autopilot_enabled = true;
        break;

    case 6:
        break;

    case 7:
        break;

    case 8:
        break;

    case 9:
        break;

    }
};

