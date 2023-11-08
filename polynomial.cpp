#include "polynomial.h"

static struct poly4d poly4d_tmp;

// evaluate a polynomial at time t using horner's rule.
float poly_eval(struct poly4d const* p, float t) {
	float x = 0.0;
	for (int i = PP_DEGREE; i >= 0; --i) {
		x = x * t + p[i];
	}
	return x;
}


// compute derivative of a 1d polynomial in place
void poly_deriv(float p[PP_SIZE]) {
	for (int i = 1; i <= PP_DEGREE; ++i) {
		p[i - 1] = i * p[i];
	}
	p[PP_DEGREE] = 0;
}


// evaluate a position vector at time t
vec polyval_pos(struct poly4d const* p, float t) {
	return makvec(poly_eval(p->p[0], t), poly_eval(p->p[1], t), poly_eval(p->p[2], t));
}


// evaluate yaw angle at time t
vec polyval_yaw(struct poly4d const* p, float t) {
	return poly_eval(p->p[3], t);
}


// evaluate the derivative of 4d polynomial
void polyder4d(struct poly4d* p) {
	for (int i = 0; i < 4; ++i) {
		polyder(p->p[i]);
	}
}


