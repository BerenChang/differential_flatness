#pragma once

#include "math3d.h"

#define PP_DEGREE (7)
#define PP_SIZE (PP_DEGREE + 1)
#define GRAVITY_CONSTANT (9.81f)

struct full_state {
	struct vec pos; // position
	struct vec vel; // velocity
	struct vec acc; // acceleration
	struct vec att; // attitude as in Euler angle (x = roll, y = pitch, z = yaw)
	struct vec omega; // attitude rate as in Euler angle
	float yaw;
};

struct poly4d {
	float p[4][PP_SIZE];
};