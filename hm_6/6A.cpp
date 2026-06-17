
#include <bits/stdc++.h>

using namespace std;

long double EPS = 1e-9;

const long double PI = acosl(-1.0L);

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


vector<Point> find_walls(vector<Point> points, int n){
    int id = 0;
    for (int i = 1; i < n; ++i) {
        if (points[i].y < points[id].y - EPS ||
            (fabsl(points[i].y - points[id].y) <= EPS && points[i].x < points[id].x - EPS))
            id = i;
    }
    swap(points[0], points[id]);
    PIVOT = points[0];
    sort(points.begin() + 1, points.end(), cmp_noauto);
    stack<Point> gr;
    gr.push(points[0]);
    gr.push(points[1]);
    for (int i = 2; i < n; ++i) {
        while (gr.size() >= 2) {
            Point top = gr.top();
            gr.pop();
            Point prev = gr.top();
            Vector v1 = point_to_vector(prev, top);
            Vector v2 = point_to_vector(top, points[i]);
            if (cross(v1, v2) > EPS) {
                gr.push(top);
                break;
            }
        }
        gr.push(points[i]);
    }
    vector<Point> plot;
    while (!gr.empty()) {
        plot.push_back(gr.top());
        gr.pop();
    }
    return plot;
}

void make_canonical(vector<Point> &poly) {
    if (poly.size() <= 1) {
        return;
    }
    long double area2 = 0;
    for (int i = 0; i < poly.size(); ++i) {
        int nxt = (i + 1) % poly.size();
        area2 += poly[i].x * poly[nxt].y - poly[nxt].x * poly[i].y;
    }
    if (area2 < 0) {
        reverse(poly.begin(), poly.end());
    }
    int start = 0;
    for (int i = 1; i < poly.size(); ++i) {
        if (poly[i].y < poly[start].y - EPS ||
            (fabsl(poly[i].y - poly[start].y) <= EPS && poly[i].x < poly[start].x - EPS)) {
            start = i;
        }
    }
    vector<Point> reordered;
    reordered.reserve(poly.size());
    for (int i = 0; i < poly.size(); ++i) {
        reordered.push_back(poly[(start + i) % poly.size()]);
    }
    poly.swap(reordered);
}

vector<Point> minkowski_sum(vector<Point> left_poly, vector<Point> right_poly) {
    make_canonical(left_poly);
    make_canonical(right_poly);
    int n = left_poly.size();
    int m = right_poly.size();
    int i = 0;
    int j = 0;
    int left = 0;
    int right = 0;
    vector<Point> merged;
    merged.reserve(n + m + 1);
    while (left < n || right < m) {
        Point cur = {left_poly[i].x + right_poly[j].x, left_poly[i].y + right_poly[j].y};
        if (merged.empty() || fabsl(merged.back().x - cur.x) > EPS || fabsl(merged.back().y - cur.y) > EPS) {
            merged.push_back(cur);
        }
        if (left == n) {
            j = (j + 1) % m;
            ++right;
        } else if (right == m) {
            i = (i + 1) % n;
            ++left;
        } else {
            Vector edge_left = { left_poly[(i + 1) % n].x - left_poly[i].x, left_poly[(i + 1) % n].y - left_poly[i].y };
            Vector edge_right = {right_poly[(j + 1) % m].x - right_poly[j].x, right_poly[(j + 1) % m].y - right_poly[j].y };
            long double turn = cross(edge_left, edge_right);

            if (turn > EPS) {
                i = (i + 1) % n;
                ++left;
            } else if (turn < -EPS) {
                j = (j + 1) % m;
                ++right;
            } else {
                i = (i + 1) % n;
                j = (j + 1) % m;
                ++left;
                ++right;
            }
        }
    }
    if (merged.size() > 1 && fabsl(merged.front().x - merged.back().x) <= EPS &&  fabsl(merged.front().y - merged.back().y) <= EPS) {
        merged.pop_back();
    }
    return merged;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int a;
    cin >> a;
    vector<Point> poly_a(a);
    for (int i = 0; i < a; ++i) {
        cin >> poly_a[i].x >> poly_a[i].y;
    }
    int b;
    cin >> b;
    vector<Point> poly_b(b);
    for (int i = 0; i < b; ++i) {
        cin >> poly_b[i].x >> poly_b[i].y;
    }

    vector<Point> reflected_b = poly_b;
    for (auto& p : reflected_b) {
        p.x = -p.x;
        p.y = -p.y;
    }

    vector<Point> sum = minkowski_sum(poly_a, reflected_b);
    Point origin{0, 0};
    bool ok = false;

    int n = sum.size();
    if (n == 0) {
        ok = false;
    } else if (n == 1) {
        ok = fabsl(sum[0].x - origin.x) <= EPS && fabsl(sum[0].y - origin.y) <= EPS;
    } else if (n == 2) {
        Point &a = sum[0];
        Point &b = sum[1];
        Vector ab = {b.x - a.x, b.y - a.y};
        Vector ap = {origin.x - a.x, origin.y - a.y};
        if (fabsl(cross(ab, ap)) <= EPS) {
            long double proj = dot(ab, ap);
            long double len2 = dot(ab, ab);
            ok = proj >= -EPS && proj <= len2 + EPS;
        } else {
            ok = false;
        }
    } else {
        long double area = 0;
        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            area += sum[i].x * sum[j].y - sum[j].x * sum[i].y;
        }

        bool ccw = area > 0;
        ok = true;
        for (int i = 0; i < n; ++i) {
            Point &a = sum[i];
            Point &b = sum[(i + 1) % n];
            Vector edge = {b.x - a.x, b.y - a.y};
            Vector to_p = {origin.x - a.x, origin.y - a.y};
            long double cr = cross(edge, to_p);
            if ((ccw && cr < -EPS) || (!ccw && cr > EPS)) {
                ok = false;
                break;
            }
        }
    }

        if (ok) {
            cout << "YES" << endl;
            return 0;
        }
        cout << "NO" << endl;    
        return 0;
    }
