#include "DoublePendulum.h"

DoublePendulum::DoublePendulum(std::vector<particle_type> &particles, const Length &l1, const Length &l2):
	System(particles), l1(l1), l2(l2) {}

void DoublePendulum::computeAccelerations() {
	/* Read the angle ("position") and angular velocity ("velocity") of both particles from the "particles" vector.
	 * Then compute the angular accelerations and store them in the particles' "acceleration" variable.
	 */

	AngularVelocity p1_v, p2_v;
	Length3D p1_p, p2_p; //angular position?!
	for (std::vector<particle_type>::iterator p = particles.begin(), int i = 1; p != particles.end(); ++p, i++)
	{
		// read and store position and velocity
		
		// compute angular acceleration

		// store acceleration for each particle
	}
}

