// 3D
#include <bits/stdc++.h>

using namespace std;

long double EPS = 1e-6;

long double PI = acosl(-1.0L);

struct Vector {
    long double x = 0, y = 0, z = 0;

    Vector() = default;

    Vector(long double x_, long double y_, long double z_) : x(x_), y(y_), z(z_) {
    }
};

struct Point {
    long double x = 0, y = 0, z = 0;

    Point() = default;

    Point(long double x_, long double y_, long double z_) : x(x_), y(y_), z(z_) {
    }
};

struct Ray {
    Point A;
    Vector v;
};

long double dot(Vector a, Vector b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector cross(Vector a, Vector b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

long double mod_vector(Vector v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector point_to_vector(Point &a, Point &b) {
    return {b.x - a.x, b.y - a.y, b.z - a.z};
}

bool on_ray(Point &p, Ray &r) {
    Vector AB = {p.x - r.A.x, p.y - r.A.y, p.z - r.A.z};
    Vector AP = {r.v.x, r.v.y, r.v.z};
    Vector cr = cross(AB, AP);
    if (mod_vector(cr) <= EPS && dot(AB, AP) >= -EPS) {
        return true;
    }
    return false;
}

bool ok(long double a, long double b) {
    return a * b >= -EPS;
}

long double orient3d(Point &a, Point &b, Point &c, Point &d) {
    return dot(cross(point_to_vector(a, b), point_to_vector(a, c)), point_to_vector(a, d));
}

struct EdgeDCEL {
    int from = -1;
    int twin = -1;
    int next = -1;
    int prev = -1;
    int face = -1;
};

struct HullDCEL {
    vector<int> verts;
    vector<EdgeDCEL> edges;
    vector<int> faces;
};

vector<Point> pts;
HullDCEL hull;
map<pair<int, int>, int> edge_pos;

int make_triangle(int x, int y, int z, queue<int> &bfs) {
    int face_id = hull.faces.size();
    int id1 = hull.edges.size();

    hull.edges.push_back({});
    hull.edges.push_back({});
    hull.edges.push_back({});

    hull.edges[id1].from = x;
    hull.edges[id1 + 1].from = y;
    hull.edges[id1 + 2].from = z;

    hull.edges[id1].next = id1 + 1;
    hull.edges[id1 + 1].next = id1 + 2;
    hull.edges[id1 + 2].next = id1;

    hull.edges[id1].face = face_id;
    hull.edges[id1 + 1].face = face_id;
    hull.edges[id1 + 2].face = face_id;

    hull.edges[id1].prev = id1 + 2;
    hull.edges[id1 + 1].prev = id1;
    hull.edges[id1 + 2].prev = id1 + 1;

    hull.faces.push_back(id1);

    if (hull.verts[x] == -1) {
        hull.verts[x] = id1;
    }
    if (hull.verts[y] == -1) {
        hull.verts[y] = id1 + 1;
    }
    if (hull.verts[z] == -1) {
        hull.verts[z] = id1 + 2;
    }

    vector<pair<pair<int, int>, int>> all_edges = {
        {{x, y}, id1},
        {{y, z}, id1 + 1},
        {{z, x}, id1 + 2}
    };

    for (auto item : all_edges) {
        pair<int, int> cur = item.first;
        int id = item.second;
        if (edge_pos.count({cur.second, cur.first})) {
            int twin_id = edge_pos[{cur.second, cur.first}];
            hull.edges[id].twin = twin_id;
            hull.edges[twin_id].twin = id;
        }
        edge_pos[cur] = id;
    }
    bfs.push(id1);
    bfs.push(id1 + 1);
    bfs.push(id1 + 2);
    return face_id;
}

int choose_third_vertex(int a, int b) {
    int cand = -1;
    for (int i = 0; i < pts.size(); i++) {
        if (i != a && i != b) {
            cand = i;
            break;
        }
    }
    for (int i = 0; i < pts.size(); i++) {
        if (i != a && i != b && i != cand) {
            long double val = orient3d(pts[a], pts[b], pts[cand], pts[i]);

            if (val > EPS) {
                cand = i;
            } else {
                if (fabsl(val) <= EPS) {
                    Vector ab = point_to_vector(pts[a], pts[b]);
                    Vector ac = point_to_vector(pts[a], pts[cand]);
                    Vector ai = point_to_vector(pts[a], pts[i]);
                    if (mod_vector(cross(ab, ai)) > mod_vector(cross(ab, ac)) + EPS) {
                        cand = i;
                    }
                }
            }
        }
    }
    return cand;
}

vector<vector<int>> build_convex_hull() {
    int n = pts.size();
    hull.verts.assign(n, -1);
    hull.edges.clear();
    hull.faces.clear();
    edge_pos.clear();
    queue<int> bfs;
    int start = 0;
    for (int i = 1; i < n; i++) {
        if (pts[i].z < pts[start].z - EPS) {
            start = i;
        } else {
            if (fabsl(pts[i].z - pts[start].z) <= EPS) {
                if (pts[i].x < pts[start].x - EPS) {
                    start = i;
                } else {
                    if (fabsl(pts[i].x - pts[start].x) <= EPS) {
                        if (pts[i].y < pts[start].y - EPS) {
                            start = i;
                        }
                    }
                }
            }
        }
    }
    int second = -1;
    long double best_angle = 1e100;
    for (int i = 0; i < pts.size(); i++) {
        if (i != start) {
            long double dx = pts[i].x - pts[start].x;
            long double dy = pts[i].y - pts[start].y;
            long double angle = atan2l(pts[i].z - pts[start].z, sqrt(dx * dx + dy * dy));
            if (second == -1 || angle < best_angle - EPS) {
                best_angle = angle;
                second = i;
            }
        }
    }
    make_triangle(start, second, choose_third_vertex(start, second), bfs);
    while (!bfs.empty()) {
        int edge_id = bfs.front();
        bfs.pop();
        if (hull.edges[edge_id].twin == -1) {
            int u = hull.edges[edge_id].from;
            int v = hull.edges[hull.edges[edge_id].next].from;
            int w = choose_third_vertex(v, u);
            make_triangle(v, u, w, bfs);
        }
    }
    vector<vector<int>> result;
    for (int i = 0; i < hull.faces.size(); i++) {
        int e1 = hull.faces[i];
        int e2 = hull.edges[e1].next;
        int e3 = hull.edges[e2].next;
        vector<int> cur = {hull.edges[e1].from, hull.edges[e2].from, hull.edges[e3].from};
        int pos = 0;
        if (cur[1] < cur[pos]) {
            pos = 1;
        }
        if (cur[2] < cur[pos]) {
            pos = 2;
        }
        result.push_back({cur[pos], cur[(pos + 1) % 3], cur[(pos + 2) % 3]});
    }
    sort(result.begin(), result.end());
    return result;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    int m;
    cin >> m;
    for (int i=0; i < m; i++) {
        int n;
        cin >> n;
        pts.resize(n);
        for (int j = 0; j < n; j++) {
            cin >> pts[j].x >> pts[j].y >> pts[j].z;
        }
        vector<vector<int>> faces = build_convex_hull();
        cout << faces.size() << endl;
        for (auto face : faces) {
            cout << 3 << ' ' << face[0] << ' ' << face[1] << ' ' << face[2] << endl;
        }
    }
    return 0;
}
