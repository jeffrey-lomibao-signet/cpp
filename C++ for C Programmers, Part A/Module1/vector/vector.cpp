#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main()
{
    vector<string> msg{"Hello", "C++", "World", "from", "VS Code", "in the Lab Sandbox!"};

    for (const string &word : msg)
    {
        cout << word << " ";
    }
    cout << endl;
}
