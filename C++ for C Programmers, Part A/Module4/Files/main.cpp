#include <iostream>
#include <iterator>
#include <fstream>
#include <filesystem>
#include <vector>

using namespace std;

int main()
{
  std::filesystem::path currentPath = std::filesystem::current_path();
  std::cout << "Current directory: " << currentPath << std::endl;

  ifstream data_file("./data.txt", ifstream::in);

  istream_iterator<int> start(data_file), end;
  int sum{0};
  size_t count{0};
  vector<int> data(start, end);
  for(auto n: data)
  // for (auto it = data.begin(); it != data.end(); ++it)
  // for (auto it = start; it != end; ++it)
  {
    // int n = *it;
    sum += n;
    cout << "n = " << n << endl;
    count++;
  }
  cout << "sum = " << sum << endl;
  cout << "average is " << 1.0 * sum / count << endl;

  data_file.close();
  return 0;
}