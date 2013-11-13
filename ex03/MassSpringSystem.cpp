#include "MassSpringSystem.h"

MassSpringSystem::MassSpringSystem(std::vector<Particle> &particles, std::vector<Spring> &springs, ParticleDamping damping, Force wind_force, Time wind_period):
	System(particles), _springs(springs), _damping(damping), _wind_force(wind_force), _wind_period(wind_period) {}

void MassSpringSystem::computeAccelerations() {
	/* 1) Please apply the forces of all springs in "_springs" to the particles attached to the spring.
	 * Remember that the getForce() function returns a force, not an acceleration, because the
	 * acceleration depends on the mass of the particle under consideration. Also, you only need to
	 * call getForce() once for each spring because the force for both attached particles only
	 * differs by the sign. Do not forget to set all accelerations to zero at the very beginning.
	 *
	 * 2) Now take the damping of the particle motion, stored in "_damping", into account.
	 *
	 * 3) Then add gravity.
	 *
	 * 4) Set the accelerations of "fixed" particles to zero.
	 *
	 * 5) Finally, apply a crudely approximated wind acceleration. The magnitude of the acceleration is
	 * stored in "_wind_force", while the period of the oscillating wind direction is stored in
	 * "_wind_period".
	 */

	//set the acceleration of all particles to zero
	for (std::vector<Particle>::iterator p = particles.begin(); p != particles.end(); ++p) {
		p->acceleration[0] = p->acceleration[1] = p->acceleration[2] = 0 * m / s / s;
	}

	for (std::vector<Spring>::const_iterator s = _springs.begin(); s != _springs.end(); ++s) {
		//get forces for each spring
		Force3D f_spring = s->getForce();

		//general forces
		Force3D f_gravity = (s->p1->mass * s->p2->mass) / ((s->p2->position - s->p1->position) * (s->p2->position - s->p1->position)) * G;
		Force f_wind = _wind_force;
		
		//forces for p1: gravity, damping, wind
		Force3D f_damp = _damping * s->p1->velocity;
		//calculate acceleration for particles one (p1)
		s->p1->acceleration = f_spring / s->p1->mass;

		//forces for p2: damping
		f_damp = _damping * s->p2->velocity;
		//calculate acceleration for particles two (p2)
		s->p2->acceleration = (-1.0)* f_spring / s->p2->mass;
	}
}

