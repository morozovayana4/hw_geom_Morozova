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
Point PIVOT;

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

double engl_near_vectors(Vector &a, Vector &b) {
    long double cr = cross(a, b);
    long double dt = dot(a, b);
    long double ang = atan2(cr, dt);
    if (ang < 0) ang += 2.0 * PI;
    return (double) ang;
}

long double len(Point &a, Point &b) {
    return sqrtl((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

long double dist2_noauto(Point &p) {
    return (p.x - PIVOT.x) * (p.x - PIVOT.x) + (p.y - PIVOT.y) * (p.y - PIVOT.y);
}

long double orient(Point& a, Point& b, Point& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool inside_halfplane(Point p, long double A, long double B, long double C) {
    return A * p.x + B * p.y + C <= EPS;
}

vector<Point> cut_polygon(vector<Point>& poly, long double A, long double B, long double C) {
    vector<Point> res;
    int m = poly.size();
    if (m == 0) {
        return res;
    }

    for (int i = 0; i < m; ++i) {
        Point cur = poly[i];
        Point nxt = poly[(i + 1) % m];
        long double v1 = A * cur.x + B * cur.y + C;
        long double v2 = A * nxt.x + B * nxt.y + C;
        if (v2 <= EPS && v1 <= EPS) {
            res.push_back(nxt);
        } else if (v1 <= EPS && !(v2 <= EPS)) {
            long double t = v1 / (v1 - v2);
            res.push_back({cur.x + (nxt.x - cur.x) * t,cur.y + (nxt.y - cur.y) * t});
        } else if (!(v1 <= EPS) && v2 <= EPS) {
            long double t = v1 / (v1 - v2);
            res.push_back({cur.x + (nxt.x - cur.x) * t,cur.y + (nxt.y - cur.y) * t});
            res.push_back(nxt);
        }
    }
    vector<Point> ans;
    for (int i = 0; i < res.size(); ++i) {
        if (i == 0 || fabsl(res[i].x - res[i - 1].x) > EPS || fabsl(res[i].y - res[i - 1].y) > EPS) {
            ans.push_back(res[i]);
        }
    }
    if (ans.size() >= 2 && fabsl(ans.front().x - ans.back().x) <= EPS && fabsl(ans.front().y - ans.back().y) <= EPS) {
        ans.pop_back();
    }
    return ans;
}
    

long double polygon_area(vector<Point>& poly) {
    long double s = 0.0;
    int m = poly.size();
    for (int i = 0; i < m; ++i) {
        s += poly[i].x * poly[(i + 1) % m].y - poly[i].y * poly[(i + 1) % m].x;
    }
    if (m < 3) {
        return 0.0;
    }
    return fabsl(s) / 2.0;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    int n;
    long double w, h;
    cin >> n >> w >> h;
    vector<Point> points(n);
    for (int i = 0; i < n; ++i) {
        cin >> points[i].x >> points[i].y;
    }
    cout << fixed << setprecision(6);
    for (int i = 0; i < n; ++i) {
        vector<Point> poly = {{0.0, 0.0}, {w, 0.0}, {w, h}, {0.0, h}};
        for (int j = 0; j < n; ++j) {
            if (j != i) {
                long double A = 2.0*(points[j].x - points[i].x);
                long double B = 2.0*(points[j].y - points[i].y);
                long double C = points[i].x * points[i].x + points[i].y * points[i].y  - points[j].x * points[j].x - points[j].y * points[j].y;
                poly = cut_polygon(poly, A, B, C);
                if (poly.empty()) {
                    break;
                }
            }
        }
        cout << polygon_area(poly) << endl;
    }
    return 0;
}
