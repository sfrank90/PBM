#include "EulerSolver.h"

void EulerSolver::step(const Time stepsize) {
	/* Please fill in your code for computing an explicit Euler time step here.
	 * The system on which the solver operates is available in the variable "_system".
	 * You have to let the system compute the accelerations exerted on all the
	 * particles before you can set the new position and velocity of each particle.
	 * Do not forget to increment the current time of the system when you are done.
	 */
    _system->computeAccelerations();

    for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
    {
        //Compute new velocity after stepsize
        Velocity3D v_diff = p->acceleration * stepsize;
        p->velocity += v_diff;

        //Compute new position after stepsize
        Length3D p_diff = p->velocity * stepsize;
        p->position += p_diff;
    }
	_system->time += stepsize;
}

