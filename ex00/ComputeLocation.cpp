#include "Quantity.h"
using namespace quantity;

Length3D computePointLocation(Time t)
{
    Acceleration3D g; g[0] = 0.0 * m / s / s; g[1] = - 9.81 * m / s / s; g[2] = 0.0 * m / s / s;
    Velocity3D v0; v0[0] = 10.0 * m / s; v0[1] = 0.0 * m / s; v0[2] = 0.0 * m / s;
    Length3D p0; p0[0] = 0.0 * m; p0[1] = 50.0 * m; p0[2] = 0.0 * m;

    std::cout << "Acceleration g " << g << std::endl;
    std::cout << "Velocity v0 " << v0 << std::endl;
    std::cout << "Point p0 " << p0 << std::endl;

    return p0 + v0 * t + 0.5 * g * t * t;
    //return p0;
}

int main()
{
	std::cout << "Test sequence for program..." << std::endl;


    Time t = 3.0 * seconds;
    std::cout << "Time to compute " << t << std::endl;

    Length3D newPoint = computePointLocation(t);
    std::cout << "Computed Location of Point p " << newPoint << std::endl;

	//Let the window remains open till user input
	std::cin.get();
	return 0;
}
