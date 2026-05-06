#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

int main() {
    double Vl = 10;
    double Vr = 8;
    double W = 30;
    double dt = 1;

    double x = 30, y = 30, theta = M_PI / 4;

    ofstream file("/home/vladimir/Documents/programs/trajectory1");
    if (!file.is_open()) {
        cout << "Error opening file!" << endl;
        return 1;
    }

    for (int k = 0; k <= 100; k++) {
        file << x << "," << y << endl;
        x = x + (Vr + Vl) / 2 * cos(theta) * dt;
        y = y + (Vr + Vl) / 2 * sin(theta) * dt;
        theta = theta + (Vr - Vl) / W * dt;
    }

    file.close();
    cout << "Trajectory saved to trajectory1" << endl;
    return 0;
}