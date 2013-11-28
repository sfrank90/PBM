#ifndef _PLANE_H
#define _PLANE_H

#include "Obstacle.h"

class Plane: public Obstacle {
	public:
		/**
		 * Create a plane with given vectorial distance from the origin.
		 */
		Plane(const Length3D &v, const ReflectionCoefficient &r=ReflectionCoefficient(), const Number &f=Number());

		/**
		 * Create a plane with given normal vector and scalar distance from the origin.
		 */
		Plane(const Number3D &n, const Length &d, const ReflectionCoefficient &r=ReflectionCoefficient(), const Number &f=Number());

		/**
		* 
		*/
		void setHalfExtend(float x, float y) {
			_extendX = x;
			_extendY = y;
			init();
		}
		/**
		 * Destroy the plane.
		 */
		~Plane() {}

		/**
		 * Get a read-only reference to the normal vector.
		 */
		const Number3D &normal() const { return _n; }

		/**
		 * Get a read-only reference to the scalar distance from the origin.
		 */
		const Length &offset() const { return _d; }

		/**
		 * Compute the reflection force on a particle.
		 */
		Force3D computeReflectionForce(const Particle &p) const;

		/**
		 * Compute the friction force on a particle.
		 */
		Force3D computeFrictionForce(const Particle &p) const;

		/**
		 * Draw the obstacle on the screen.
		 */
		virtual void draw(const Length &scale) const;
	private:
		Number3D _n; //< The normal vector.
		Length _d; //< The scalar distance from the origin.
		float _extendX, _extendY;
		Number3D _p1,_p2,_p3,_p4;
		void init();
		bool isParticleOnPlane(const Particle &p) const;

};

#endif

