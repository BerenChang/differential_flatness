#pragma once

#include "polynomial.h"
#include "data_types.h"
#include "math3d.h"

float poly_eval(struct poly4d const* p, float t);
void poly_deriv(float p[PP_SIZE]);
vec polyval_pos(struct poly4d const* p, float t);
vec polyval_yaw(struct poly4d const* p, float t);
void polyder4d(struct poly4d* p);