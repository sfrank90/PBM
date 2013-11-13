#include "Spring.h"

Force3D Spring::getForce() const {
	/* Please compute the stiffness and damping force exerted on the first particle.
	 * The rest length is stored in the variable "length", the stiffness and damping
	 * are stored in "stiffness" and "damping", respectively.
	 * The two particles are "p1" and "p2".
	 */
	Length3D diff_pos = p2->position - p1->position;
	Velocity3D diff_vel = p2->velocity - p1->velocity;

	Force3D f_hooke = stiffness * (norm(diff_pos) - length) * (diff_pos / norm(diff_pos));
	Force3D f_damp = ((damping * diff_vel) * (diff_pos / norm(diff_pos))) *(diff_pos / norm(diff_pos));

	return f_hooke + f_damp;
}

