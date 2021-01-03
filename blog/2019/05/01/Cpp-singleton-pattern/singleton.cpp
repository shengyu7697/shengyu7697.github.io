// g++ singleton.cpp -std=c++11
#include <iostream>
#include <stdio.h>
using namespace std;

class Setting {
public:
  static Setting &getInstance() {
    static Setting sInstance;
    return sInstance;
  }

  Setting() {}
};

int main() {
  Setting &setting = Setting::getInstance();
  Setting &setting2 = Setting::getInstance();
  cout << "setting addr: " << &setting << endl;
  cout << "setting2 addr: " << &setting2 << endl;

  return 0;
}
