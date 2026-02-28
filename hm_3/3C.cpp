#include <bits/stdc++.h>

using namespace std;

long double EPS = 1e-9;

const long double PI = acosl(-1.0L);

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

vector<long double> peresech(long double x1, long double y1, long double x2, long double y2, long double x3,
                             long double y3, long double x4,
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
    if (ang < 0) ang += 2.0L * PI;
    return (double) ang;
}

double len(Point &a, Point &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}


Point PIVOT;

long double dist2_noauto(const Point &p) {
    return (p.x - PIVOT.x) * (p.x - PIVOT.x) + (p.y - PIVOT.y) * (p.y - PIVOT.y);
}

bool cmp_noauto(const Point &A, const Point &B) {
    long double a1 = atan2l(A.y - PIVOT.y, A.x - PIVOT.x);
    long double a2 = atan2l(B.y - PIVOT.y, B.x - PIVOT.x);
    if (fabsl(a1 - a2) > EPS) {
        return a1 < a2;
    }
    return dist2_noauto(A) < dist2_noauto(B);
}

long double orient(Point &a, Point &b, Point &c) {
    return (b.x - a.x) * (c.y - a.y) -
           (b.y - a.y) * (c.x - a.x);
}

vector<Point> find_walls(vector<Point> points, int n) {
    sort(points.begin(), points.end(), [](Point &A, Point &B) {
        if (A.x != B.x) {
            return A.x < B.x;
        }
        return A.y < B.y;
    });

    vector<Point> temp;

    if (!points.empty()) {
        temp.push_back(points[0]);
        for (int i = 1; i < points.size(); ++i) {
            if (!(points[i].x == points[i - 1].x && points[i].y == points[i - 1].y)) {
                temp.push_back(points[i]);
            }
        }
    }

    points = temp;
    n = points.size();

    if (n <= 1) {
        return points;
    }
    vector<Point> plot;
    for (int i = 0; i < n; ++i) {
        while (plot.size() >= 2) {
            int sz = plot.size();
            if (orient(plot[sz - 2], plot[sz - 1], points[i]) <= 0) {
                plot.pop_back();
            } else {
                break;
            }
        }
        plot.push_back(points[i]);
    }

    int lower_size = plot.size();

    for (int i = n - 2; i >= 0; --i) {
        while (plot.size() > lower_size) {
            int sz = plot.size();
            if (orient(plot[sz - 2], plot[sz - 1], points[i]) <= 0) {
                plot.pop_back();
            } else {
                break;
            }
        }
        plot.push_back(points[i]);
    }
    if (!plot.empty()) {
        plot.pop_back();
    }
    return plot;
}

long double diameter2(vector<Point> &plot) {
    int m = plot.size();
    if (m < 2) {
        return 0.0;
    }
    if (m == 2) {
        return (plot[0].x - plot[1].x) * (plot[0].x - plot[1].x) + (plot[0].y - plot[1].y) * (plot[0].y - plot[1].y);
    }

    long double best = 0.0;
    int k = 1;

    for (int i = 0; i < m; ++i) {
        int i2 = i + 1;
        if (i2 == m) {
            i2 = 0;
        }
        int k2 = k + 1;
        if (k2 == m) {
            k2 = 0;
        }

        while (fabsl(orient(plot[i], plot[i2], plot[k2])) > fabsl(orient(plot[i], plot[i2], plot[k]))) {
            k = k2;
            ++k2;
            if (k2 == m) {
                k2 = 0;
            }
        }

        long double d = (plot[i].x - plot[k].x) * (plot[i].x - plot[k].x) + (plot[i].y - plot[k].y) * (
                            plot[i].y - plot[k].y);
        if (d > best) {
            best = d;
        }

        d = (plot[i2].x - plot[k].x) * (plot[i2].x - plot[k].x) + (plot[i2].y - plot[k].y) * (plot[i2].y - plot[k].y);
        if (d > best) {
            best = d;
        }
    }

    return best;
}

int main() {
    int n;
    cin >> n;
    vector<Point> points(n);
    for (int i = 0; i < n; ++i) {
        long double x, y;
        cin >> x >> y;
        points[i] = Point(x, y);
    }
    vector<Point> plot = find_walls(points, n);
    reverse(plot.begin(), plot.end());
    cout << diameter2(plot) << endl;
}
