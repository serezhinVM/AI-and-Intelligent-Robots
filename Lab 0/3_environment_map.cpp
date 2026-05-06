#include <iostream>
#include <fstream>
#include <cmath>
using namespace std;

// Основная программа для создания карты окружения (Environment Map)
// Карта содержит: 4 колонны, 1 зарядная станция, стены, робот

int main() {
    // Открываем файл для записи
    ofstream map;
    map.open("/home/vladimir/Documents/programs/EnvironmentMap.txt");
    
    // Координаты центров объектов (5 точек: 4 колонны + 1 зарядка)
    double x0[5] = {0.8, 0.8, 1.7, 2.2, 2.25};  // x координаты
    double y0[5] = {1.5, 2.25, 1.0, 1.3, 0.25}; // y координаты
    
    double r = 0.125;        // радиус колонн и зарядки
    double wallLength = 2.5; // длина стены (размер комнаты)
    double PI = 3.14159265;
    double RD = PI / 180;    // радианы в градусы
    
    // === 1. СТЕНЫ (Walls) ===
    // Комната размером 2.5м x 2.5м
    map << "# Walls" << endl;
    
    // Нижняя стена (y = 0)
    for (double j = 0; j <= wallLength; j += 0.01) {
        map << j << ' ' << 0 << endl;
    }
    
    // Верхняя стена (y = wallLength)
    for (double j = 0; j <= wallLength; j += 0.01) {
        map << j << ' ' << wallLength << endl;
    }
    
    // Левая стена (x = 0)
    for (double j = 0; j <= wallLength; j += 0.01) {
        map << 0 << ' ' << j << endl;
    }
    
    // Правая стена (x = wallLength)
    for (double j = 0; j <= wallLength; j += 0.01) {
        map << wallLength << ' ' << j << endl;
    }
    
    // === 2. КОЛОННЫ (Pillars) ===
    // 4 колонны - круги радиусом r
    map << "# Pillars" << endl;
    
    // Для каждой из 4 колонн рисуем окружность (точки через каждые 20 градусов)
    for (int p = 0; p < 4; p++) {
        for (int i = 0; i < 360; i += 20) {
            // x = x_center + r * cos(angle)
            // y = y_center + r * sin(angle)
            map << x0[p] + r * cos(i * RD) << ' ' << y0[p] + r * sin(i * RD) << endl;
        }
    }
    
    // === 3. ЗАРЯДНАЯ СТАНЦИЯ (Charger) ===
    // 5-я точка - зарядка
    map << "# Charger" << endl;
    
    for (int i = 0; i < 360; i += 20) {
        map << x0[4] + r * cos(i * RD) << ' ' << y0[4] + r * sin(i * RD) << endl;
    }
    
    // === 4. РОБОТ ===
    // Квадрат 0.25м x 0.25м в центре (0.3, 0.3)
    map << "# Robot (square 0.25m edge at center 0.3, 0.3)" << endl;
    
    double robot_center_x = 0.3; // центр робота по x
    double robot_center_y = 0.3; // центр робота по y
    double robot_edge = 0.25;   // размер робота
    double half = robot_edge / 2;
    
    // Нижняя сторона
    for (double i = robot_center_x - half; i <= robot_center_x + half; i += 0.01) {
        map << i << ' ' << robot_center_y - half << endl;
    }
    
    // Верхняя сторона
    for (double i = robot_center_x - half; i <= robot_center_x + half; i += 0.01) {
        map << i << ' ' << robot_center_y + half << endl;
    }
    
    // Левая сторона
    for (double j = robot_center_y - half; j <= robot_center_y + half; j += 0.01) {
        map << robot_center_x - half << ' ' << j << endl;
    }
    
    // Правая сторона
    for (double j = robot_center_y - half; j <= robot_center_y + half; j += 0.01) {
        map << robot_center_x + half << ' ' << j << endl;
    }
    
    // Закрываем файл
    map.close();
    cout << "Environment map saved" << endl;
    return 0;
}