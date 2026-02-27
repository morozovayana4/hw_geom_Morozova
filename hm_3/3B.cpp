#include <bits/stdc++.h>

using namespace std;

long double EPS = 1e-9;

struct Vector {
    long double x = 0, y = 0;

    Vector() = default;

    Vector(long double x_, long double y_) : x(x_), y(y_) {
    }
};

struct Point {
    long double x = 0, y = 0;

    Point() = default;

    Point(long double x_, long double y_) : x(x_), y(y_) {
    }
};

struct Ray {
    Point A;
    Vector v;
};

long double dot(Vector &a, Vector &b) {
    //скалярное
    return a.x * b.x + a.y * b.y;
}

long double cross(Vector &a, Vector &b) {
    return a.x * b.y - a.y * b.x;
}

bool on_ray(Point &p, Ray &r) {
    Vector AB = {r.v.x - r.A.x, r.v.y - r.A.y};
    Vector AP = {p.x - r.A.x, p.y - r.A.y};
    if (cross(AB, AP) <= EPS && dot(AB, AP) >= -EPS) {
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

vector<long double>
peresech(long double x1, long double y1, long double x2, long double y2, long double x3, long double y3, long double x4,
         long double y4) {
    vector<long double> f1 = A_B_C(x1, y1, x2, y2);
    vector<long double> f2 = A_B_C(x3, y3, x4, y4);
    if (is_point(x1, y1, x2, y2) && is_point(x3, y3, x4, y4)) {
        if (x1 == x3 && y1 == y3) {
            return {1, x1, y1};
        } else {
            return {0};
        }
    }
    if (is_point(x1, y1, x2, y2)) {
        if (f2[0] * x1 + f2[1] * y1 + f2[2] <= EPS) {
            return {1, x1, y1};
        } else {
            return {0};
        }
    }
    if (is_point(x3, y3, x4, y4)) {
        if (f1[0] * x3 + f1[1] * y3 + f1[2] <= EPS) {
            return {1, x3, y3};
        } else {
            return {0};
        }
    }
    long double det = f1[0] * f2[1] - f2[0] * f1[1];
    if (fabs(det) <= EPS) {
        if (fabs(f1[0] * f2[2] - f2[0] * f1[2]) <= EPS &&
            fabs(f1[1] * f2[2] - f2[1] * f1[2]) <= EPS) {
            return {2};
        } else {
            return {0};
        }
    }
    long double denom = (-f2[0] * f1[1] + f2[1] * f1[0]);
    if (fabsl(denom) <= EPS) {
        return {0};
    }
    long double y = (f2[0] * f1[2] - f2[2] * f1[0]) / denom;
    long double x;
    if (is_goriz(f1)) {
        x = -(f2[1] * y + f2[2]) / f2[0];
        return {1, x, y};
    }
    x = -1 * (f1[1] * y + f1[2]) / f1[0];
    return {1, x, y};
}

bool point_in_polynom(vector<Point> &points, int n, Point point_p, int ok) {
    for (int i = 0; i < n - 1; ++i) {
        vector<long double> edge = A_B_C(points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
        vector per = peresech(point_p.x, point_p.y, point_p.x + 1, point_p.y, points[i].x, points[i].y, points[i + 1].x, points[i + 1].y);
        Vector AB = point_to_vector(points[i], points[i + 1]);
        Vector AP = point_to_vector(points[i], point_p);
        if (fabsl(cross(AB, AP)) <= EPS && is_betveen(point_p, points[i], points[i + 1])) {
            return true;
        }
        if (per[0] == 2 && is_betveen(point_p, points[i], points[i + 1])) {
            return true;
        }
        if (per[0] == 1 && point_p.x <= per[1] + EPS) {
            Point a = {per[1], per[2]};
            if (is_betveen(a, points[i], points[i + 1])) {
                if (points[i].y > point_p.y + EPS != points[i + 1].y > point_p.y + EPS) {
                    ok *= -1;
                }
            }
        }
    }
    Vector AB = point_to_vector(points[n - 1], points[0]);
    Vector AP = point_to_vector(points[n - 1], point_p);
    if (fabsl(cross(AB, AP)) <= EPS && is_betveen(point_p, points[n - 1], points[0])) {
        return true;
    }
    vector<long double> edge = A_B_C(points[0].x, points[0].y, points[n - 1].x, points[n - 1].y);
    vector per = peresech(point_p.x, point_p.y, point_p.x + 1, point_p.y, points[0].x, points[0].y, points[n - 1].x, points[n - 1].y);

    if (per[0] == 1 && point_p.x <= per[1] + EPS) {
        Point a = {per[1], per[2]};
        if (is_betveen(a, points[0], points[n - 1])) {
            if (points[n - 1].y > point_p.y + EPS != points[0].y > point_p.y + EPS) {
                ok *= -1;
            }
        }
    }
    if (ok == 1) {
        return true;
    }
    return false;
}

int main() {
    int n;
    cin >> n;
    Point point_p;
    cin >> point_p.x >> point_p.y;
    vector<Point> points(n);
    int ok = -1;
    for (int i = 0; i < n; ++i) {
        long double x, y;
        cin >> x >> y;
        points[i] = Point(x, y);
    }

    if (point_in_polynom(points,n,point_p,ok)) {
        cout << "YES" << endl;
    }else {
        cout << "NO" << endl;
    }
    return 0;
}
