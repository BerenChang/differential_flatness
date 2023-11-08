// Author: Beren
// For the purpose of generating full state trajectories from flat outputs
// Input: Sequence of position x, y, z and angle yaw
// Ouput: Trajectory


#include "diff_flat.h"

// return full state struct at time t
struct full_state flat_converter(float t) {

	// flat variables
	struct full_state full_state_output;
	full_state_output.pos = polyval_pos(p, t);
	full_state_output.att.z = polyval_yaw(p, t);

	// 1st derivative
	struct poly4d* deriv = &poly4d_tmp;
	*deriv = *p;
	polyder4d(deriv);
	out.vel = polyval_xyz(deriv, t);
	float dyaw = polyval_yaw(deriv, t);

	// 2nd derivative
	polyder4d(deriv);
	out.acc = polyval_pos(deriv, t);

	// 3rd derivative
	polyder4d(deriv);
	struct vec jerk = polyval_pos(deriv, t);

	struct vec thrust = vadd(full_state_output.acc, mkvec(0, 0, GRAVITY_CONSTANT));
	// float thrust_mag = mass * vmag(thrust);

	struct vec z_b = vnormalize(thrust);
	struct vec x_c = mkvec(cosf(full_state_output.yaw), sinf(full_state_output.yaw), 0);
	struct vec y_b = vnormalize(vcross(z_b, x_c));
	struct vec x_b = vcross(y_b, z_b);

	struct vec jerk_orth_zbody = vorthunit(jerk, z_body);
	struct vec h_w = vscl(1.0f / vmag(thrust), jerk_orth_zbody);

	full_state_output.omega.x = -vdot(h_w, y_body);
	full_state_output.omega.y = vdot(h_w, x_body);
	full_state_output.omega.z = z_body.z * dyaw;

	return full_state_output;
}