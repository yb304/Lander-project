#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

int main() {

  // declare variables
  double m, k, x, v, t_max, dt, t, a, previous_position, position, velocity;
  vector<double> t_list, x_list, v_list, a_list;

  // mass, spring constant, initial position and velocity
  m = 1;
  k = 1;
  x = 0;
  v = 1;
  t = 0;
  // simulation time and timestep
  t_max = 100;
  dt = 0.1;
  x_list.push_back(0);
  v_list.push_back(1);
  t_list.push_back(0);
  // First two points use Euler integration
    for (t = 0; t < dt; t = t + dt )
    {
    a = -k * x / m;
    previous_position = x;
    position = previous_position + v * dt + 0.5;
    velocity = v + a * dt;
    t = t + dt;
    t_list.push_back(t);
    x_list.push_back(position);
    v_list.push_back(velocity);
    }
  // Verlet integration

  for (t = dt; t < t_max; t = t + dt)
  { a = -k * (x_list[x_list.size() - 1]) / m;
    position = 2 * x_list[x_list.size() - 1] + (x_list[x_list.size() - 2])+ dt * dt * a;
    velocity = (x_list[x_list.size() - 1] - x_list[x_list.size() - 2]) / dt;
    t = t + dt;
    x_list.push_back(position);
    v_list.push_back(velocity);
    t_list.push_back(t);
  }


  // print x_list and v_list
  for (auto v : x_list)
      cout << v << "\n";
  for (auto n : v_list)
      cout << n << "\n";
  // Write the trajectories to file
  ofstream fout;
  fout.open("/Users/Yeming/trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i = i + 1) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << endl;
    }
  } else { // file did not open successfully
    cout << "Could not open trajectory file for writing" << endl;
  }
  }
