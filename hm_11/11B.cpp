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


Point PIVOT;

long double dist2_noauto(Point &p) {
    return (p.x - PIVOT.x) * (p.x - PIVOT.x) + (p.y - PIVOT.y) * (p.y - PIVOT.y);
}

bool cmp_noauto(Point &A, Point &B) {
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


struct Segment {
    Point a, b;
    int id = 0;
};

struct Event {
    long double x, y;
    int type, id;
};

vector<Segment> segments;
long double sweep_x = 0;

long double orient(Point& a, Point& b, Point& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool check_intersect(int i, int j) {
    Point& p1 = segments[i].a;
    Point& p2 = segments[i].b;
    Point& p3 = segments[j].a;
    Point& p4 = segments[j].b;
    long double dx1 = p2.x - p1.x, dy1 = p2.y - p1.y;
    long double dx2 = p4.x - p3.x, dy2 = p4.y - p3.y;
    long double det = dx1 * dy2 - dx2 * dy1;
    if (fabsl(det) > EPS) {
        long double t = ((p3.x - p1.x) * dy2 - (p3.y - p1.y) * dx2) / det;
        long double u = ((p3.x - p1.x) * dy1 - (p3.y - p1.y) * dx1) / det;
        if (t >= -EPS && t <= 1+EPS && u >= -EPS && u <= 1+EPS) {
            return true;
        }
        return false;
    } else {
        if (fabsl(orient(p1, p2, p3)) > EPS) return false;
        long double min1x = min(p1.x, p2.x), max1x = max(p1.x, p2.x);
        long double min2x = min(p3.x, p4.x), max2x = max(p3.x, p4.x);
        long double min1y = min(p1.y, p2.y), max1y = max(p1.y, p2.y);
        long double min2y = min(p3.y, p4.y), max2y = max(p3.y, p4.y);
        if (max1x + EPS < min2x || max2x + EPS < min1x) {
            return false;
        }
        if (max1y + EPS < min2y || max2y + EPS < min1y) {
            return false;
        }
        return true;
    }
}

struct SegCmp {
    bool operator()(int i, int j) {
        if (i == j) {
            return false;
        }
        Point& seg1a = segments[i].a;
        Point& seg1b = segments[i].b;
        Point& seg2a = segments[j].a;
        Point& seg2b = segments[j].b;
        long double y1;
        if (fabsl(seg1a.x - seg1b.x) <= EPS) {
            y1 = min(seg1a.y, seg1b.y);
        } else {
            y1 = seg1a.y + (seg1b.y - seg1a.y) * (sweep_x - seg1a.x) / (seg1b.x - seg1a.x);
        }
        long double y2;
        if (fabsl(seg2a.x - seg2b.x) <= EPS) {
            y2 = min(seg2a.y, seg2b.y);
        } else {
            y2 = seg2a.y + (seg2b.y - seg2a.y) * (sweep_x - seg2a.x) / (seg2b.x - seg2a.x);
        }

        if (fabsl(y1 - y2) > EPS) return y1 < y2;
         return i < j;
    }
};

bool in_circle(Point a, Point b, Point c, Point p) {
    long double ax = a.x - p.x;
    long double ay = a.y - p.y;
    long double bx = b.x - p.x;
    long double by = b.y - p.y;
    long double cx = c.x - p.x;
    long double cy = c.y - p.y;
    long double det =(ax * ax + ay * ay) * (bx * cy - by * cx) -(bx * bx + by * by) * (ax * cy - ay * cx) +(cx * cx + cy * cy) * (ax * by - ay * bx);
    long double s = orient(a, b, c);
    if (fabsl(s) <= EPS) {
        return false;
    }
    if (s > 0) {
        return det > EPS;
    } else {
        return det < -EPS;
    }
}

long double circumradius(Point &a, Point &b, Point &c) {
    long double s = fabsl(orient(a, b, c));
    if (s <= EPS) {
        return 0.0;
    }
    return len(a, b) * len(b, c) * len(c, a) / (2.0 * s);
}

struct TriangleData {
    int a, b, c;
    bool alive = true;
};

vector<TriangleData> triangles;
unordered_map<unsigned long long, int> edge_to_triangle;

unsigned long long edge_key(int u, int v) {
    return (static_cast<unsigned long long>(static_cast<unsigned int>(u)) << 32) |
           static_cast<unsigned int>(v);
}

int third_vertex_of_triangle(int tid, int u, int v) {
    if (tid < 0 || tid >= (int)triangles.size() || !triangles[tid].alive) {
        return -1;
    }

    int a = triangles[tid].a;
    int b = triangles[tid].b;
    int c = triangles[tid].c;

    if (a == u && b == v) return c;
    if (b == u && c == v) return a;
    if (c == u && a == v) return b;

    return -1;
}

void put_edge(int u, int v, int tid) {
    edge_to_triangle[edge_key(u, v)] = tid;
}

void pop_edge(int u, int v) {
    auto it = edge_to_triangle.find(edge_key(u, v));
    if (it != edge_to_triangle.end()) {
        edge_to_triangle.erase(it);
    }
}


void pop_triangle(vector<Point>& points, int a, int b, int c) {
    if (orient(points[a], points[b], points[c]) < 0) {
        swap(b, c);
    }
    auto it = edge_to_triangle.find(edge_key(a, b));
    if (it != edge_to_triangle.end()) {
        int tid = it->second;
        if (tid >= 0 && tid < triangles.size()) {
            triangles[tid].alive = false;
        }
    }
    pop_edge(a, b);
    pop_edge(b, c);
    pop_edge(c, a);
}

int opposite(int a, int b) {
    auto it = edge_to_triangle.find(edge_key(a, b));
    if (it == edge_to_triangle.end()) {
        return -1;
    }
    return third_vertex_of_triangle(it->second, a, b);
}


vector<int> hull;
int SUPER_A, SUPER_B, SUPER_C;

void make_ccw(vector<Point>& points, int& a, int& b, int& c) {
    if (orient(points[a], points[b], points[c]) < 0) {
        swap(b, c);
    }
}

bool point_in_triangle(Point &p, Point &a, Point &b, Point &c) {
    if (orient(a, b, p) < -EPS) {
        return false;
    }
    if (orient(b, c, p) < -EPS) {
        return false;
    }
    if (orient(c, a, p) < -EPS) {
        return false;
    } 
    return true;
}

struct KNode {
    Point p;
    int left;
    int right;
};

static inline int new_knode(vector<KNode>& nodes, Point p) {
    KNode node;
    node.p = p;
    node.left = -1;
    node.right = -1;
    int id = nodes.size();
    nodes.push_back(node);
    return id;
}
bool cmp_kd_x(Point A, Point B) {
    if (fabsl(A.x - B.x) > EPS) {
        return A.x < B.x;
    }
    return A.y < B.y;
}

bool cmp_kd_y(Point A, Point B) {
    if (fabsl(A.y - B.y) > EPS) {
        return A.y < B.y;
    }
    return A.x < B.x;
}

int build_tree(vector<Point>& arr, int L, int R, int d, vector<KNode>& nodes) {
    if (L >= R) {
        return -1;
    }
    int axis = d % 2;
    int mid = (L + R - 1) / 2;
    if (axis == 0) {
        sort(arr.begin() + L, arr.begin() + R, cmp_kd_x);
    } else {
        sort(arr.begin() + L, arr.begin() + R, cmp_kd_y);
    }
    int id = new_knode(nodes, arr[mid]);
    int left_id = build_tree(arr, L, mid, d + 1, nodes);
    int right_id = build_tree(arr, mid + 1, R, d + 1, nodes);
    nodes[id].left = left_id;
    nodes[id].right = right_id;
    return id;
}

void search_tree(vector<KNode>& nodes, Point qry, int cur_id, int depth,
                 Point& best1, Point& best2, long double& d1, long double& d2) {
    if (cur_id == -1) return;

    long double dx = nodes[cur_id].p.x - qry.x;
    long double dy = nodes[cur_id].p.y - qry.y;
    long double cur_d = dx * dx + dy * dy;

    if (cur_d < d1) {
        swap(best1, best2);
        swap(d1, d2);
        best1 = nodes[cur_id].p;
        d1 = cur_d;
    } else if (cur_d < d2) {
        best2 = nodes[cur_id].p;
        d2 = cur_d;
    }
    int axis = depth % 2;
    int fr, sec;
    long double dif;
    if (axis == 0) {
        dif = qry.x - nodes[cur_id].p.x;
        if (dif < 0) {
            fr = nodes[cur_id].left;
            sec = nodes[cur_id].right;
        } else {
            fr = nodes[cur_id].right;
            sec = nodes[cur_id].left;
        }
    } else {
        dif = qry.y - nodes[cur_id].p.y;
        if (dif < 0) {
            fr = nodes[cur_id].left;
            sec = nodes[cur_id].right;
        } else {
            fr = nodes[cur_id].right;
            sec = nodes[cur_id].left;
        }
    }
    search_tree(nodes, qry, fr, depth + 1, best1, best2, d1, d2);
    if (dif * dif <= d2 + EPS) {
        search_tree(nodes, qry, sec, depth + 1, best1, best2, d1, d2);
    }
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    int n;
    cin >> n;
    vector<Point> points(n);
    for (int i = 0; i < n; ++i) {
        cin >> points[i].x >> points[i].y;
    }
    vector<KNode> nodes;
    nodes.reserve(n);
    int root_id = build_tree(points, 0, n, 0, nodes);
    int q;
    cin >> q;
    while (q--) {
        Point point;
        cin >> point.x >> point.y;
        Point best1, best2;
        long double d1 = 1e19;
        long double d2 = 1e19;
        search_tree(nodes, point, root_id, 0, best1, best2, d1, d2);
        cout << best1.x << ' ' <<best1.y << ' ' << best2.x << ' ' << best2.y << endl;
    }
    return 0;
}
