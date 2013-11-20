#ifndef _RUNGE_KUTTA_SOLVER_H
#define _RUNGE_KUTTA_SOLVER_H

#include "Solver.h"

/**
 * Runge-Kutta method for solving the equations of motion of a system.
 */
class RungeKuttaSolver: public Solver {
	public:
		/**
		 * Construct a new solver for a given system.
		 */
		RungeKuttaSolver(System *system): Solver(system) {}

		/**
		 * Iterate the system by a fixed timestep.
		 */
		void step(const Time stepsize);
};

#endif

