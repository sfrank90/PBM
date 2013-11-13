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

    // stroe original values for p0 and v0
    std::vector<Length3D> p0;
    std::vector<Velocity3D> v0;
    for (size_t i = 0; i < _system->particles.size(); ++i) {
        p0.push_back(_system->particles[i].position);
        v0.push_back(_system->particles[i].velocity);
    }

    // Compute K1
    _system->computeAccelerations();
    std::vector<Velocity3D> k_v1;
    std::vector<Acceleration3D> k_a1;
    for (size_t i = 0; i < _system->particles.size(); ++i) {
        //store velocity v0 and computed a(p0) for each particle
        k_v1.push_back(_system->particles[i].velocity);
        k_a1.push_back(_system->particles[i].acceleration);

        //compute new dp1 and dv1
        _system->particles[i].position = p0[i] + 0.5 * stepsize * k_v1[i];
        _system->particles[i].velocity = v0[i] + 0.5 * stepsize * k_a1[i];
    }

    // Compute K2
    _system->computeAccelerations();
    std::vector<Velocity3D> k_v2;
    std::vector<Acceleration3D> k_a2;
    for (size_t i = 0; i < _system->particles.size(); ++i) {
        //store values for K2
        k_v2.push_back(_system->particles[i].velocity);
        k_a2.push_back(_system->particles[i].acceleration);

        //compute new dp2 and dv2
        _system->particles[i].position = p0[i] + 0.5 * stepsize * k_v2[i];
        _system->particles[i].velocity = v0[i] + 0.5 * stepsize * k_a2[i];
    }

    // Compute K3
    _system->computeAccelerations();
    std::vector<Velocity3D> k_v3;
    std::vector<Acceleration3D> k_a3;
    for (size_t i = 0; i < _system->particles.size(); ++i) {
        //store values for K3
        k_v3.push_back(_system->particles[i].velocity);
        k_a3.push_back(_system->particles[i].acceleration);

        //compute new dp3 and dv3
        _system->particles[i].position = p0[i] + stepsize * k_v3[i];
        _system->particles[i].velocity = v0[i] + stepsize * k_a3[i];
    }

    // Compute K4 and final result for p and v
    _system->computeAccelerations();
    std::vector<Velocity3D> k_v4;
    std::vector<Acceleration3D> k_a4;
    for (size_t i = 0; i < _system->particles.size(); ++i) {
        //store values for K4
        k_v4.push_back(_system->particles[i].velocity);
        k_a4.push_back(_system->particles[i].acceleration);

        //final computation for p and v
        _system->particles[i].position = p0[i] + stepsize / 6.0 * (k_v1[i] + 2.0 * k_v2[i] + 2.0 * k_v3[i] + k_v4[i]);
        _system->particles[i].velocity = v0[i] + stepsize / 6.0 * (k_a1[i] + 2.0 * k_a2[i] + 2.0 * k_a3[i] + k_a4[i]);
    }

    // increment system time
    _system->time += stepsize;

}




