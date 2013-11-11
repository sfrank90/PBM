#include "GravitationalSystem.h"

GravitationalSystem::GravitationalSystem(std::vector<Particle> &particles): System(particles) {}

void GravitationalSystem::computeAccelerations() {
	/* Please fill in your code for computing the gravitational acceleration here.
	 * The list of particles is available in the variable "particles". Each
	 * particle has a member "acceleration" to which you will write the
	 * acceleration which is exerted on that particle by the gravitational
	 * influence of all the other particles. The acceleration a for a particle is
	 * given as
	 *
	 *   a = F / m,
	 *
	 * where F is the sum of all the gravitational forces from other particles,
	 * as defined on the exercise sheet, and m is the mass of the particle that is
	 * accelerated.
	 *
	 * You may use the functions "dot", "squared_norm", "norm" and "pow" from
	 * Quantity.h to compute dot products, norms and powers of physical quantities.
	 */

    // Print the initial configuration.
    /*for (std::vector<Particle>::const_iterator p = particles.begin(); p != particles.end(); ++p)
        std::cout << *p << std::endl;
    std::cout << std::endl;*/

    int i = 0;
    for (std::vector<Particle>::iterator p = particles.begin(); p != particles.end(); ++p)
    {
        i++;
        //create overall force for this particle
        Force3D f;

        //get mass and position for first particle
        Mass m1 = p->mass;
        Length3D p1 = p->position;

        for (std::vector<Particle>::const_iterator p_next = particles.begin(); p_next != particles.end(); ++p_next)
        {
            // if positions are equal dont calculate the force
            Length3D p2 = p_next->position;
            if(p1 == p2){}
            else
            {
                //get mass and position for second particle
                Mass m2 = p_next->mass;
                f += G * ((m1 * m2)/ squared_norm(p2 - p1)) * ((p2 - p1)/norm(p2 - p1));
            }
        }

        //calculate the acceleration for this specific particles according to all others
        const Acceleration3D a = f / m1;
        std::cout << "Acceleration for " << p->name << " : " << a << std::endl;

        //TODO: tore acceleration to specific particles
        p->acceleration = a;
    }
    std::cout << std::endl;

    // Print the end configuration after acceleration modification.
    for (std::vector<Particle>::const_iterator p = particles.begin(); p != particles.end(); ++p)
        std::cout << *p << std::endl;
    std::cout << std::endl;

}

