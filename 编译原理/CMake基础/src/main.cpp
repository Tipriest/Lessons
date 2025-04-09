#include "../include/Solution.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;
int main(int argc, char **argv) {
  Solution solution;
  vector<int> dist = {1, 1, 100000};
  double hour = 2.01;
  cout << solution.minSpeedOnTime(dist, hour) << endl;
  return 1;

  
}