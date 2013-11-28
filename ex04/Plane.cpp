#include "Plane.h"

#include <cstdlib>

#if defined(__APPLE_CC__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

Plane::Plane(const Length3D &v, const ReflectionCoefficient &r, const Number &f):
	Obstacle(r, f), _n(v / norm(v)), _d(norm(v)), _extendX(0), _extendY(0) {
    init();
}

Plane::Plane(const Number3D &n, const Length &d, const ReflectionCoefficient &r, const Number &f):
	Obstacle(r, f), _n(n), _d(d), _extendX(0), _extendY(0) {
    init();
}

void Plane::init() {
	Number3D rnd; // Random "up" vector for drawing obstacles.
	for (size_t i = 0; i < 3; ++i)
		rnd[i] = rand() / double(RAND_MAX);
	Number3D u = cross(_n, rnd); // u is orthogonal to _n
	u /= norm(u);
	Number3D v = cross(_n, u); // v is orthogonal to _n and u
	v /= norm(v);

	double c = 20.0;
	if(_extendX > 0 && _extendY > 0) {
		_p1 =  _extendX * u + _extendY * v + _d * _n / m;
		_p2 = -_extendX * u + _extendY * v + _d * _n / m;
		_p3 = -_extendX * u - _extendY * v + _d * _n / m;
		_p4 =  _extendX * u - _extendY * v + _d * _n / m;
	}
	else{
		_p1 = c * (u + v) + _d * _n / m;
		_p2 = c * (-u + v) + _d * _n / m;
		_p3 = c * (-u - v) + _d * _n / m;
		_p4 = c * (u - v) + _d * _n / m;
	}
}

void Plane::draw(const Length &scale) const {
	float offset = 0.02;
        float sc = 1; 
        Number3D center;
	Number3D     p1 = (_p1 - _n*offset)*m/scale,
		     p2 = (_p2 - _n*offset)*m/scale,
		     p3 = (_p3- _n*offset)*m/scale,
		     p4 = (_p4- _n*offset)*m/scale;
        center = _d*_n/m;//(_p1 -_d*_n+_p2+_p3+_p4)/4.0;
        p1= center + sc*(p1-center); 
p2= center + sc*(p2-center); 
p3= center + sc*(p3-center); 
p4= center + sc*(p4-center); 
	glBegin(GL_QUADS);
	glVertex3d(p1[0], p1[1], p1[2]);
	glVertex3d(p2[0], p2[1], p2[2]);
	glVertex3d(p3[0], p3[1], p3[2]);
	glVertex3d(p4[0], p4[1], p4[2]);
	glEnd();
}

bool Plane::isParticleOnPlane(const Particle &p) const {
    Number3D  v1, v2;
	if(_extendX > 0 && _extendY > 0) {
                Length dist = dot(_n, p.position);
                if(!(_d - dist < Length(0.01))) return false;
               // if(!(dist - _d < Length(0.01))) return false;
		v1 = _p2-_p1; v2 = _p1-_p4;
		v1 /= norm(v1);
		v2 /= norm(v2);

		//Length t = _d / dot(p.position/norm(p.position),_n);
		//Length3D hitpoint = t * p.position/norm(p.position);

		Number3D normHit = p.position / m;

		if( dot((normHit-_p1)/norm(normHit-_p1), v1) <= Number(0) ||
			dot((normHit-_p2)/norm(normHit-_p2), -v1) <= Number(0) ||
			dot((normHit-_p4)/norm(normHit-_p4), v2) <= Number(0) ||
			dot((normHit-_p1)/norm(normHit-_p1), -v2) <= Number(0))
			return false;
		//std::cout << dot((normHit-_p2)/norm(normHit-_p2), -v1) << std::endl;
                
		return true;
	}
	return true;

	/*Length dist = dot(_n, p.position);
	bool ison = true;
	
	if (_extendX > 0 && _extendY > 0) {
		// verhindern, dass viel weiter unter der ebene alles hinaufbeschleunigt wird
		//ison &= (_d - dist < Length(0.01));
		
		// _p1 und _p3 liegen diagonal zueinander, sowie _p2 und _p4
		// man kann über _p1 -> _p2 -> _p3 -> _p4 -> _p1 um die ebene laufen
                const float offset = 2;
		const Number3D points[] = {_p1 + _n*offset, _p2 + _n*offset, _p3 + _n*offset, _p4 + _n*offset, _p1 + _n*offset};
		for (size_t i = 0; i < 4 && ison; i++) {
			// 1. ebene mit points[i] und points[i+1] als normale erstellen
			Number3D nl = (points[i+1] - points[i]);
			Number3D n = nl / norm(nl);
			Number d = dot(points[i], n);
			
			// 2. gucken ob p einen größeren abstand hat
			ison &= dot(p.position/m, n) > d;
			
			// irgenwie muss der rand härter werden
		}
                
	}

        return ison;*/
}

Force3D Plane::computeReflectionForce(const Particle &p) const {
	/* Please fill in your code for computing the reflection force here.
	 *
	 * Check if the particle is on the correct side of the plane, i.e., in direction of
	 * the normal vector "_n". The distance of the plane to the origin is available as
	 * "_d". If the particle is on the wrong side of the plane, apply the reflection
	 * force. The reflection force tries to push the particle back to the correct side
	 * of the plane. It should be proportional to the distance of the particle to the
	 * plane (the proportionality constant is stored in "_r"), and should act in the
	 * direction of the normal of the plane.
	 */

	Force3D reflection;
	Length h = dot(p.position, _n);

	if(h < _d && isParticleOnPlane(p)) {
		reflection = (_d - h) * _r * _n;
	}
	return reflection;
}

Force3D Plane::computeFrictionForce(const Particle &p) const {
	/* Please fill in your code for computing the friction force here.
	 *
	 * Check if the particle is on the correct side of the plane, i.e., in direction of
	 * the normal vector "_n". The distance of the plane to the origin is available as
	 * "_d". If the particle is on the wrong side of the plane, apply the friction force.
	 * The friction force slows down the particle with respect to the plane. It should be
	 * proportional to the normal component of the force that is currently acting on the
	 * particle (the proportionality constant is stored in "_f"), and should act in the
	 * direction of the velocity of the particle.
	 */
	Force3D friction;
	if(dot(p.position, _n) < _d && isParticleOnPlane(p)) {
		Force3D currForce = p.mass * p.acceleration;
		friction =  dot(currForce, -_n) * _f * (-p.velocity) / norm(p.velocity);
	}
	return friction;
}

