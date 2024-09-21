#include <iostream>

using namespace std;

enum class days:short {SUN, MON, TUE, WED, THU, FRI, SAT};

inline days operator++(days& d) {
  return days(static_cast<int>(d) + 1 % 7);
}

ostream& operator<<(ostream& out, const days& d){
  switch(d) {
  case days::SUN: out << "SUN"; break;
  case days::MON: out << "MON"; break;
  case days::TUE: out << "TUE"; break;
  case days::WED: out << "WED"; break;
  case days::THU: out << "THU"; break;
  case days::FRI: out << "FRI"; break;
  case days::SAT: out << "SAT"; break;
  }
  return out;
}

int main() {
  days d{days::SUN};
  cout << d << endl;

  return 0;
}
