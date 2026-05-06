#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

// Объединяем Task A и Task B: траектории + карта окружения

int main() {
    ofstream file;
    file.open("/home/vladimir/Documents/programs/Trajectories-Map");
    
    // === ТРАЕКТОРИИ (как в Task A-2) ===
    double Vl1 = 10, Vr1 = 8;
    double Vl2 = 5, Vr2 = 7;
    double W = 30;
    double dt = 1;
    double x, y, theta;
    
    // Траектория 1: Vl=10, Vr=8
    x = 30, y = 30, theta = M_PI / 4;
    file << "# Trajectory 1: Vl=10, Vr=8" << endl;
    for (int k = 0; k <= 100; k++) {
        file << x << "," << y << endl;
        x = x + (Vr1 + Vl1) / 2 * cos(theta) * dt;
        y = y + (Vr1 + Vl1) / 2 * sin(theta) * dt;
        theta = theta + (Vr1 - Vl1) / W * dt;
    }
    
    file << endl;
    
    // Траектория 2: Vl=5, Vr=7
    x = 30, y = 30, theta = M_PI / 4;
    file << "# Trajectory 2: Vl=5, Vr=7" << endl;
    for (int k = 0; k <= 100; k++) {
        file << x << "," << y << endl;
        x = x + (Vr2 + Vl2) / 2 * cos(theta) * dt;
        y = y + (Vr2 + Vl2) / 2 * sin(theta) * dt;
        theta = theta + (Vr2 - Vl2) / W * dt;
    }
    
    file << endl;
    
    // === КАРТА ОКРУЖЕНИЯ (как в Task B) ===
    double x0[5] = {0.8, 0.8, 1.7, 2.2, 2.25};
    double y0[5] = {1.5, 2.25, 1.0, 1.3, 0.25};
    double r = 0.125, wallLength = 2.5;
    double PI = 3.14159265;
    double RD = PI / 180;
    
    // Стены (в метрах -> см: умножить на 100)
    file << "# Walls" << endl;
    wallLength = wallLength * 100;  // 2.5m -> 250cm
    for (double j = 0; j <= wallLength; j += 1) {  // 1cm шаг
        file << j << "," << 0 << endl;
    }
    for (double j = 0; j <= wallLength; j += 1) {
        file << j << "," << wallLength << endl;
    }
    for (double j = 0; j <= wallLength; j += 1) {
        file << 0 << "," << j << endl;
    }
    for (double j = 0; j <= wallLength; j += 1) {
        file << wallLength << "," << j << endl;
    }
    
    // Колонны
    file << "# Pillars" << endl;
    r = r * 100;  // 0.125m -> 12.5cm
    x0[0] *= 100; x0[1] *= 100; x0[2] *= 100; x0[3] *= 100; x0[4] *= 100;
    y0[0] *= 100; y0[1] *= 100; y0[2] *= 100; y0[3] *= 100; y0[4] *= 100;
    for (int p = 0; p < 4; p++) {
        for (int i = 0; i < 360; i += 20) {
            file << x0[p] + r * cos(i * RD) << "," << y0[p] + r * sin(i * RD) << endl;
        }
    }
    
    // Зарядка
    file << "# Charger" << endl;
    for (int i = 0; i < 360; i += 20) {
        file << x0[4] + r * cos(i * RD) << "," << y0[4] + r * sin(i * RD) << endl;
    }
    
    // Робот (квадрат 25см x 25см в центре 30, 30)
    file << "# Robot" << endl;
    double rx = 30, ry = 30, re = 25;
    double h = re / 2;
    for (double i = rx - h; i <= rx + h; i += 1) {
        file << i << "," << ry - h << endl;
    }
    for (double i = rx - h; i <= rx + h; i += 1) {
        file << i << "," << ry + h << endl;
    }
    for (double j = ry - h; j <= ry + h; j += 1) {
        file << rx - h << "," << j << endl;
    }
    for (double j = ry - h; j <= ry + h; j += 1) {
        file << rx + h << "," << j << endl;
    }
    
    file.close();
    cout << "Trajectories-Map saved" << endl;
    return 0;
}