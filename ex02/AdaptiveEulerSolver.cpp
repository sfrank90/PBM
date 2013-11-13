#include "AdaptiveEulerSolver.h"

void AdaptiveEulerSolver::step(const Time stepsize) {
	/* Please fill in your code for computing an adaptive explicit Euler time step here.
	 * The system on which the solver operates is available in the variable "_system".
	 * The average error threshold is available in the variable "_threshold".
	 * You have to let the system compute the accelerations exerted on all the
	 * particles before you can set the new position and velocity of each particle.
	 * Do not forget to increment the current time of the system when you are done.
	 */

	Time cum_stepsize = 0 * s;
    int i = 0;

	//repeat: find stepsize, compute explicit euler till cumulated stepsize is equal stepsize
    while (cum_stepsize != stepsize)
	{
        Length err_max = 0 * m;
        Time stepsize_opt = stepsize - cum_stepsize;
		_system->computeAccelerations();
		//Calculate adaptive stepsize (stepsize_new)
		for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
		{
            Length3D p_temp1 = p->position + stepsize_opt * p->velocity;

            Length3D p_temp2 = p->position + 0.5 * stepsize_opt * p->velocity;
            Velocity3D v_temp = p->velocity + 0.5 * stepsize_opt * p->acceleration;

            p_temp2 += stepsize_opt/2 * v_temp;

            err_max = maximum(err_max, norm(p_temp1-p_temp2));
		}
	
		//compare err_max of the particle system with threshold distance
		if(err_max > _threshold)
		{
            stepsize_opt= stepsize * quantity::sqrt(_threshold/err_max);
        }

		//correct stepsize if nessecary
        if(cum_stepsize + stepsize_opt > stepsize)
            stepsize_opt = stepsize - cum_stepsize;

		//compute explicit euler
		for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
		{
			//Compute new position and velocity with adaptive stepsize_new
            p->position += stepsize_opt * p->velocity;
            p->velocity += stepsize_opt * p->acceleration;
		}

		//add actual stepsize to cumulated stepsize
        cum_stepsize += stepsize_opt;
        ++i;
	}	
    if(i > 1)
        std::cout << i << " steps in adaptive Euler solver" << std::endl;
  
	// Increment the current system time.
    _system->time += stepsize;
}

