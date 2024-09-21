// Convert this program to C++
// change to C++ io
// change to one line comments
// change defines of constants to const
// change array to vector<>
// inline any short function

#include <iostream>
#include <vector>

using namespace std;

template <class summable>
summable sum(const vector<summable>& d)
{
    summable sum{0};
    for (auto i: d)
        sum += i;
    return sum;
}

int main()
{
    // generate test data
    vector<int> data;
    const int DATA_SIZE = 40;
    for (int i{0}; i < DATA_SIZE; ++i)
        data.push_back(i);

    cout << "sum is " << sum(data) << endl;

    return 0;
}
