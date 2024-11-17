#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;
class Solution {
public:
    vector<int> dist;
    double hour;

    int minSpeedOnTime(vector<int>& dist, double hour);
    bool satisfy(int speed);

};