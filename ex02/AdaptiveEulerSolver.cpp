#include "AdaptiveEulerSolver.h"

void AdaptiveEulerSolver::step(const Time stepsize) {
	/* Please fill in your code for computing an adaptive explicit Euler time step here.
	 * The system on which the solver operates is available in the variable "_system".
	 * The average error threshold is available in the variable "_threshold".
	 * You have to let the system compute the accelerations exerted on all the
	 * particles before you can set the new position and velocity of each particle.
	 * Do not forget to increment the current time of the system when you are done.
	 */

	_system->computeAccelerations();
    for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
    {
		Time stepsize_new, stepsize_temp = stepsize;

		//repeat explicit euler till cumulated stepsize (cum_stepsize) is equal to function stepsize
		Time cum_stepsize = 0 * s;
		while (cum_stepsize != stepsize)
		{
			//Calculate adaptive stepsize (stepsize_new)
			Length err_max = 0 * m;

			Length3D p_temp1 = p->position + stepsize_temp * p->velocity;
			Length3D p_temp2 = p->position;
			Velocity3D v_temp = p->velocity;
			for(int i = 0; i < 2; i++)
			{
				p_temp2 += stepsize_temp/2 * v_temp;
				v_temp += stepsize_temp/2 * p->acceleration;
			}
			
			if(norm(p_temp1 - p_temp2) <= _threshold)
			{
				stepsize_temp = stepsize_temp / 2;
				//set adaptive stepsize to actual stepsize
				stepsize_new = stepsize_temp;
		
				//Compute new position and velocity with adaptive stepsize_new
				p->position += stepsize_new * p->velocity;
				p->velocity += stepsize_new * p->acceleration;

				//add actual stepsize to cumulated stepsize
				cum_stepsize += stepsize_new;
			}	
			else
			{
				std::cout << "Condition not fullfilled. Reducing stepsize by 1 second" << std::endl;
				stepsize_temp -= 1 * s;
			}		
		}
    }
	// Increment the current system time.
	_system->time += stepsize;
}

