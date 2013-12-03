#include "DoublePendulum.h"

DoublePendulum::DoublePendulum(std::vector<particle_type> &particles, const Length &l1, const Length &l2):
	System(particles), l1(l1), l2(l2) {}

void DoublePendulum::computeAccelerations() {
	/* Read the angle ("position") and angular velocity ("velocity") of both particles from the "particles" vector.
	 * Then compute the angular accelerations and store them in the particles' "acceleration" variable.
	 */

    AngularVelocity p1_v, p2_v;
    Angle p1_p, p2_p; //angular position?!

    p1_v = particles[0].velocity;
    p2_v = particles[1].velocity;

    p1_p = particles[0].position;
    p2_p = particles[1].position;

    Mass m1, m2;
    m1 = particles[0].mass;
    m2 = particles[1].mass;

    Angle diff_ang = (p2_p - p1_p);

    particles[0].velocity = p1_v;
    particles[1].velocity = p2_v;

	Acceleration _g = 9.81*m/s/s;

	particles[0].acceleration = (m2 * l1 * p1_v * p1_v * sin(diff_ang) * cos(diff_ang) + m2 * l2 * p2_v *p2_v *sin(diff_ang) - (m1+m2) * _g * sin(p1_p))/ ((m1+m2)*l1 - m2*l1*cos(diff_ang)*cos(diff_ang));
	particles[1].acceleration = (-m2 * l2 * p2_v * p2_v * sin(diff_ang) * cos(diff_ang) + (m1+m2)*(_g*sin(p1_p) * cos(diff_ang) - l1 * p1_v * p1_v *sin(diff_ang) -_g * sin(p2_p)))/ ((m1+m2)*l2 - m2*l2*cos(diff_ang)*cos(diff_ang));

}

