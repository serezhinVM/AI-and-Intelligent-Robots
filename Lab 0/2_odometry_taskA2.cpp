#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

int main() {
    double Vl1 = 10, Vr1 = 8;
    double Vl2 = 5, Vr2 = 7;
    double W = 30;
    double dt = 1;
    double x, y, theta;

    ofstream file("/home/vladimir/Documents/programs/trajectory_both");

    x = 30; y = 30; theta = M_PI / 4;
    for (int k = 0; k <= 100; k++) {
        file << x << "," << y << endl;
        x = x + (Vr1 + Vl1) / 2 * cos(theta) * dt;
        y = y + (Vr1 + Vl1) / 2 * sin(theta) * dt;
        theta = theta + (Vr1 - Vl1) / W * dt;
    }

    file << endl;

    x = 30; y = 30; theta = M_PI / 4;
    for (int k = 0; k <= 100; k++) {
        file << x << "," << y << endl;
        x = x + (Vr2 + Vl2) / 2 * cos(theta) * dt;
        y = y + (Vr2 + Vl2) / 2 * sin(theta) * dt;
        theta = theta + (Vr2 - Vl2) / W * dt;
    }

    file.close();
    cout << "Two trajectories saved to trajectory_both" << endl;
    return 0;
}