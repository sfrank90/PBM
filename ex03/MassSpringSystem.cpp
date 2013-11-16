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
	Acceleration3D g; g[0] = 0.0 * m / s / s; g[1] = - 9.81 * m / s / s; g[2] = 0.0 * m / s / s;
	Acceleration3D fixed_p; fixed_p[0] = 0.0 * m / s / s; fixed_p[1] = 0.0 * m / s / s; fixed_p[2] = 0.0 * m / s / s;

	//wind force
	Force3D wind_force3D;
	wind_force3D[0] = (1.5 + cos(time / _wind_period)) * _wind_force;
	wind_force3D[1] = 0 * _wind_force;
	wind_force3D[2] = (sin(time / _wind_period)) * _wind_force;

	//set the acceleration of all particles to zero
	for (std::vector<Particle>::iterator p = particles.begin(); p != particles.end(); ++p) {
		p->acceleration[0] = p->acceleration[1] = p->acceleration[2] = 0 * m / s / s;
	}

	for (std::vector<Spring>::const_iterator s = _springs.begin(); s != _springs.end(); ++s) {
		//get forces for each spring
		Force3D f_int = s->getForce();
		
		//damping forces for each particles
		Force3D fp1_damp = - _damping * s->p1->velocity;
		Force3D fp2_damp = - _damping * s->p2->velocity;

		//gravity forces
		Force3D fp1_g = g * s->p1->mass;
		Force3D fp2_g = g * s->p2->mass;

		//calculate acceleration for the two particles of the spring
		if(s->p1->fixed)
			s->p1->acceleration = fixed_p;
		else 
			s->p1->acceleration = (f_int + fp1_damp + wind_force3D + fp1_g)/ s->p1->mass;

		if(s->p2->fixed)
			s->p2->acceleration = fixed_p;
		else
			s->p2->acceleration = ((-1.0)* f_int + fp2_damp + wind_force3D + fp2_g) / s->p2->mass;
	}
}

