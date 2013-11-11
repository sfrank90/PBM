#include "RungeKuttaSolver.h"

void RungeKuttaSolver::step(const Time stepsize)  {
	/* Please fill in your code for computing a 4th order Runge-Kutta time step here.
	 * The system on which the solver operates is available in the variable "system".
	 *
	 * The system of differential equations that you will solve is of the form
	 *
	 *   y'(t) = f(y(t)),
	 *
	 * where
	 *
	 *   y(t) = (x(t), v(t)),
	 *
	 * with x(t) and v(t) containing the positions and velocities of all the
	 * particles, respectively. You can split this system into the position and
	 * velocity components, such that
	 *
	 *   x'(t) = v(t)  and
	 *   v'(t) = a(x(t)).
	 *
	 * You can evaluate a(x(t)) by setting all the particle positions in the system to
	 * the corresponding values of x(t), calling _system->computeAccelerations() and
	 * reading the accelerations back from the particles. If you want to be generic
	 * and allow for systems where the accelerations may also depend on the velocity
	 * (for example, when there is friction) or on the time (for example, when an
	 * external force changes over time), you should also set the particle velocities
	 * and the system time before calling computeAccelerations().
	 *
	 * When your computation is done, store the new x and v values in the particles.
	 * Do not forget to increment the current time of the system when you are done.
	 */
	Length3D p0, p1, p2, p3, p4;
	Velocity3D v0, v1, v2, v3, v4;
	Acceleration3D a1, a2, a3, a4;

	_system->computeAccelerations();
    for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
    {
		//get actual values for the particle
		p0 = p->position;
		v0 = p->velocity;

		//runge kutta 4th order
		p1 = p0;
        v1 = v0;
		a1 = p->acceleration;
		
		p2 = p0 + v1 * stepsize * 0.5;
		p->position = p2;
		_system->computeAccelerations();
		a2 = p->acceleration;
        v2 = v0 + a1 *stepsize * 0.5;	
		
		p3 = p0 + v2 * stepsize * 0.5;
		p->velocity = v2; p->position = p3;
		_system->computeAccelerations();
		a3 = p->acceleration;
        v3 = v0 + a2 * stepsize * 0.5;

		p4 = p0 + v3 * stepsize;
		p->velocity = v3; p->position = p4;
		_system->computeAccelerations();
		a4 = p->acceleration;
        v4 = v0 + a3 * stepsize;

		//set final values for new position and velocity
		p->position = p0 + (stepsize * (v1 + 2 * v2 + 2 * v3 + v4))/6;
		p->velocity = v0 + (stepsize * (a1 + 2 * a2 + 2 * a3 + a4))/6;

    }
	// Increment the current system time.
	_system->time += stepsize;

}




