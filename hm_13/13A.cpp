#include <bits/stdc++.h>

using namespace std;

long double EPS = 1e-12;

long double PI = acos(-1.0);

struct Vector {
    long double x = 0, y = 0;

    Vector() = default;

    Vector(long double x_, long double y_) : x(x_), y(y_) {}
};

struct Point {
    long double x = 0, y = 0;

    Point() = default;

    Point(long double x_, long double y_) : x(x_), y(y_) {}
};

struct Ray {
    Point A;
    Vector v;
};

long double dot(Vector &a, Vector &b) {
    return a.x * b.x + a.y * b.y;
}

long double cross(Vector &a, Vector &b) {
    return a.x * b.y - a.y * b.x;
}
bool on_ray(Point &p, Ray &r) {
    Vector AB = {r.v.x - r.A.x, r.v.y - r.A.y};
    Vector AP = {p.x - r.A.x, p.y - r.A.y};
    if (fabsl(cross(AB, AP)) <= EPS && dot(AB, AP) >= -EPS) {
        return true;
    }
    return false;
}

bool is_betveen(Point &p, Point &a, Point &b) {
    if (min(a.x, b.x) - EPS <= p.x && p.x <= max(a.x, b.x) + EPS && min(a.y, b.y) - EPS <= p.y &&
        p.y <= EPS + max(a.y, b.y)) {
        return true;
    } else {
        return false;
    }
}

vector<long double> A_B_C(long double x_1, long double y_1, long double x_2, long double y_2) {
    return {y_2 - y_1, x_1 - x_2, -x_1 * y_2 + y_1 * x_2};
}

bool is_point(long double x_1, long double y_1, long double x_2, long double y_2) {
    if (x_1 == x_2 && y_1 == y_2) {
        return true;
    }
    return false;
}

bool is_goriz(vector<long double> &h) {
    if (h[0] == 0 && h[1] != 0) {
        return true;
    }
    return false;
}

bool is_vert(vector<long double> &h) {
    if (h[1] == 0 && h[0] != 0) {
        return true;
    }
    return false;
}

long double mod_vector(Vector v) {
    return (sqrt(v.x * v.x + v.y * v.y));
}

long double squer(Vector a, Vector b) {
    return cross(a, b) / 2;
}

Vector point_to_vector(Point &a, Point &b) {
    return {b.x - a.x, b.y - a.y};
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    string base32 = "0123456789bcdefghjkmnpqrstuvwxyz";
    int t;
    cin >> t;
    for (int i = 0; i < t; ++i) {
        int d;
        long double lon, lat;
        cin >> d >> lon >> lat;
        long double l_lon = -180.0, r_lon = 180.0;
        long double l_lat = -90.0, r_lat = 90.0;
        for (int sym = 0; sym < d; ++sym) {
            int idx = 0;
            for (int bit = 0; bit < 5; ++bit) {
                idx <<= 1;
                if ((sym * 5 + bit) % 2 == 0) {              
                    long double mid = (l_lon + r_lon) / 2.0;
                    if (lon >= mid) {
                        idx |= 1;
                        l_lon = mid;
                    } else {
                        r_lon = mid;
                    }
                } else {                         
                    long double mid = (l_lat + r_lat) / 2.0;
                    if (lat >= mid) {
                        idx |= 1;
                        l_lat = mid;
                    } else {
                        r_lat = mid;
                    }
                }
            }
            cout << base32[idx];
        }
        cout << endl;        
    }
    return 0;
}
