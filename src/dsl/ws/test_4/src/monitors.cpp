#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

double distancePointToPoint(std::vector<double> xyz1, std::vector<double> xyz2) {
    double x1 = xyz1[0];
    double y1 = xyz1[1];
    double z1 = xyz1[2];

    double x2 = xyz2[0];
    double y2 = xyz2[1];
    double z2 = xyz2[2];

    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));

    return distance;
}

int main() {
    auto comp_op = "lt";
    
    auto threshold = 0.001;

    double distance = distancePointToPoint(
        {0.1, 0.2, 0.1},
        {0.1, 0.3, 0.1}
    );

    cout << "Distance: " << distance << endl;

    
    if (comp_op == "lt")
    {
        if (distance < threshold)
        {
            cout << "Distance is less than threshold" << endl;
        }
        else
        {
            cout << "Distance is greater than threshold" << endl;
        }
    } else if (comp_op == "gt")
    {
        if (distance > threshold)
        {
            cout << "Distance is greater than threshold" << endl;
        }
        else
        {
            cout << "Distance is less than threshold" << endl;
        }
    } else if (comp_op == "eq")
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