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

    AngularVelocity d = (p2_v - p1_v);
    double diff_vel = ((p2_v - p1_v)*s/m).value_type;

    /*particles[0].velocity = p1_v;
    particles[1].velocity = p2_v;

    particles[0].acceleration = (m2 * l1 * p1_v * p1_v * sin(diff_vel) * cos(diff_vel) + m2 * l2 * p2_v *p2_v *sin(diff_vel) - (m1+m2) * g * sin(p1_p))/ ((m1+m2)*l1 - m2*l1*cos(diff_vel)*cos(diff_vel));
    particles[1].acceleration = (-m2 * l2 * p2_v * p2_v * sin(diff_vel) * cos(diff_vel) + (m1+m2) +(g*sin(p1_p) * cos(diff_vel) - l1 * p1_v * p1_v *sin(diff_vel) -g * sin(p2_p)))/ ((m1+m2)*l2 - m2*l2*cos(diff_vel)*cos(diff_vel));

*/

}

