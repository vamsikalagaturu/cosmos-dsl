#include <iostream>
#include <cmath>

using namespace std;

struct Point3D {
    double x, y, z;
};

double distance3D(Point3D p1, Point3D p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

int main() {
    Point3D p1 = { 1.0, 2.0, 3.0 };
    Point3D p2 = { 4.0, 5.0, 6.0 };
    double dist = distance3D(p1, p2);
    cout << "Distance between (" << p1.x << ", " << p1.y << ", " << p1.z << ") and (" << p2.x << ", " << p2.y << ", " << p2.z << ") is " << dist << endl;
    return 0;
}