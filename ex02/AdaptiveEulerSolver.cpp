#include "AdaptiveEulerSolver.h"

void AdaptiveEulerSolver::step(const Time stepsize) {
	/* Please fill in your code for computing an adaptive explicit Euler time step here.
	 * The system on which the solver operates is available in the variable "_system".
	 * The average error threshold is available in the variable "_threshold".
	 * You have to let the system compute the accelerations exerted on all the
	 * particles before you can set the new position and velocity of each particle.
	 * Do not forget to increment the current time of the system when you are done.
	 */

	Time stepsize_new; 
	Time cum_stepsize = 0 * s;
	Length err_max = 0 * m;
	
	_system->computeAccelerations();
	//Calculate adaptive stepsize (stepsize_new)
	for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
	{
		Length3D p_temp1 = p->position + stepsize * p->velocity;
		
		Length3D p_temp2 = p->position + stepsize/2 * p->velocity;
		Velocity3D v_temp = stepsize/2 * p->acceleration;
		p_temp2 += stepsize/2 * v_temp;

		if(norm(p_temp1-p_temp2) > err_max)
			err_max = norm(p_temp1-p_temp2);
	}
	
	//compare errors
	if(err_max > _threshold)
	{
		std::cout << "Condition not fullfilled. Reducing stepsize...";
		stepsize_new = stepsize * pow(0.5,(_threshold/err_max));
		std::cout << "finished" << std::endl;
	}	
	while (cum_stepsize < stepsize)
	{
		for (std::vector<Particle>::iterator p = _system->particles.begin(); p != _system->particles.end(); ++p)
		{
			//Compute new position and velocity with adaptive stepsize_new
			p->position += stepsize_new * p->velocity;
			p->velocity += stepsize_new * p->acceleration;
		}

		//add actual stepsize to cumulated stepsize
		cum_stepsize += stepsize_new;
	}	
  
	// Increment the current system time.
	_system->time += stepsize;
}

