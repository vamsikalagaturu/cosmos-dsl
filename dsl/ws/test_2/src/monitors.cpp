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
    auto comp_op = "<";
    
    auto threshold = 0.001;

    auto from_pos_coord_f = "http://example.com/rob#f1";
    auto from_pos_coord_u = "http://qudt.org/vocab/unit/M";
    auto to_pos_coord_f = "http://example.com/rob#f1";
    auto to_pos_coord_u = "http://qudt.org/vocab/unit/M";
            

    if (from_pos_coord_u != to_pos_coord_u)
    {
        // TODO: handle different units
        cerr << "Units are not the same" << endl;
        cerr << "Handler not implemented" << endl;
        return 1;
    }

    if (from_pos_coord_f != to_pos_coord_f)
    {
        // TODO: transform into same frame
        cerr << "Frames are not the same" << endl;
        cerr << "Handler not implemented" << endl;
        return 1;
    }
            
            
    Point3D from_pos_coord;
    Point3D to_pos_coord;
            
            
    from_pos_coord.x = 0.1;
    from_pos_coord.y = 0.2;
    from_pos_coord.z = 0.1;
            

            
    to_pos_coord.x = 0.1;
    to_pos_coord.y = 0.3;
    to_pos_coord.z = 0.1;
    double distance = distance3D(from_pos_coord, to_pos_coord);

    cout << "Distance: " << distance << endl;

            
    if (comp_op == "<")
    {
        if (distance < threshold)
        {
            cout << "Distance is less than threshold" << endl;
        }
        else
        {
            cout << "Distance is greater than threshold" << endl;
        }
    } else if (comp_op == ">")
    {
        if (distance > threshold)
        {
            cout << "Distance is greater than threshold" << endl;
        }
        else
        {
            cout << "Distance is less than threshold" << endl;
        }
    } else if (comp_op == "==")
    {
        if (fabs(distance - threshold) < 0.0001)
        {
            cout << "Distance is equal to threshold" << endl;
        }
        else
        {
            cout << "Distance is not equal to threshold" << endl;
        }
    } else
    {
        cerr << "Unknown comparison operator" << endl;
        cerr << "Handler not implemented" << endl;
        return 1;
    }

    return 0;
}