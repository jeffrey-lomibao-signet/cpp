#include <iostream>

using namespace std;

class point {
public:
    point() {
        x = 0;
        y = 0;
    }

    point(double x, double y) {
        setx(x);
        sety(y);
    }

    double getx() const { return x; }    // accessor, automatic inline
    void setx(double v) { x = v; } // mutator
    double gety() const { return y; }
    void sety(double v) { y = v; }

    point operator+(const point& p) const {
        return point(this->x + p.x, this->y + p.y );
    }

    friend ostream& operator<<(ostream& out, const point& p) {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

private: 
    double x, y;       // hide representation
};

int main() {
    point a{3.5, 2.5}, b{2.5, 4.5}, c;
    cout << "a = " << a << "; b = " << b << endl;
    cout << "sum = " << a + b << endl;

    return 0;
}
