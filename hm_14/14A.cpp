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

struct Rect {
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0, id = 0;
};

struct Node {
    Rect box;
    vector<int> sons;
    vector<int> rects;
    bool leaf = false;
};

vector<Rect> a;
vector<Node> tree;

void extend_box(Rect &box, Rect &r, bool first) {
    if (first) {
        box = r;
    } else {
        box.x1 = min(box.x1, r.x1);
        box.y1 = min(box.y1, r.y1);
        box.x2 = max(box.x2, r.x2);
        box.y2 = max(box.y2, r.y2);
    }
}

int build_index(int n) {
    vector<int> ids;
    for (int i = 0; i < n; ++i) {
        ids.push_back(i);
    }
    vector<int> cur;
    while (!ids.empty()) {
        Node v;
        v.leaf = true;
        int first_id = ids.back();
        ids.pop_back();
        v.rects.push_back(first_id);
        v.box = a[first_id];
        while (v.rects.size() < 4 && !ids.empty()) {
            int best = 0;
            long long best_add = -1;
            long long best_square = -1;
            long long w_box = v.box.x2 - v.box.x1;
            long long h_box = v.box.y2 - v.box.y1;
            long long s_box = w_box * h_box;
            for (int i = 0; i < ids.size(); ++i) {
                Rect now;
                now.x1 = min(v.box.x1, a[ids[i]].x1);
                now.y1 = min(v.box.y1, a[ids[i]].y1);
                now.x2 = max(v.box.x2, a[ids[i]].x2);
                now.y2 = max(v.box.y2, a[ids[i]].y2);
                long long w_now = now.x2 - now.x1;
                long long h_now = now.y2 - now.y1;
                long long s_now = w_now * h_now;
                long long w_rect = a[ids[i]].x2 - a[ids[i]].x1;
                long long h_rect = a[ids[i]].y2 - a[ids[i]].y1;
                long long s_rect = w_rect * h_rect;
                long long add = s_now - s_box;
                if (best_add == -1 || add < best_add || add == best_add && s_rect < best_square) {
                    best = i;
                    best_add = add;
                    best_square = s_rect;
                }
            }
            int id = ids[best];
            v.rects.push_back(id);
            extend_box(v.box, a[id], false);
            ids[best] = ids.back();
            ids.pop_back();
        }
        tree.push_back(v);
        cur.push_back(tree.size() - 1);
    }
    while (cur.size() > 1) {
        vector<int> ids;
        for (int i = 0; i < cur.size(); ++i) {
            ids.push_back(cur[i]);
        }
        vector<int> nxt;
        while (!ids.empty()) {
            Node v;
            v.leaf = false;
            int first_id = ids.back();
            ids.pop_back();
            v.sons.push_back(first_id);
            v.box = tree[first_id].box;
            while (v.sons.size() < 4 && !ids.empty()) {
                int best = 0;
                long long best_add = -1;
                long long best_square = -1;
                long long w_box = v.box.x2 - v.box.x1;
                long long h_box = v.box.y2 - v.box.y1;
                long long s_box = w_box * h_box;
                for (int i = 0; i < ids.size(); ++i) {
                    Rect now;
                    now.x1 = min(v.box.x1, tree[ids[i]].box.x1);
                    now.y1 = min(v.box.y1, tree[ids[i]].box.y1);
                    now.x2 = max(v.box.x2, tree[ids[i]].box.x2);
                    now.y2 = max(v.box.y2, tree[ids[i]].box.y2);
                    long long w_now = now.x2 - now.x1;
                    long long h_now = now.y2 - now.y1;
                    long long s_now = w_now * h_now;
                    long long w_rect = tree[ids[i]].box.x2 - tree[ids[i]].box.x1;
                    long long h_rect = tree[ids[i]].box.y2 - tree[ids[i]].box.y1;
                    long long s_rect = w_rect * h_rect;
                    long long add = s_now - s_box;
                    if (best_add == -1 || add < best_add || add == best_add && s_rect < best_square) {
                        best = i;
                        best_add = add;
                        best_square = s_rect;
                    }
                }
                int id = ids[best];
                v.sons.push_back(id);
                extend_box(v.box, tree[id].box, false);
                ids[best] = ids.back();
                ids.pop_back();
            }
            tree.push_back(v);
            nxt.push_back(tree.size() - 1);
        }
        cur = nxt;
    }
    return cur[0];
}

void locate(int v, int x, int y, vector<int> &ans) {
    if (x < tree[v].box.x1 || tree[v].box.x2 < x ||
        y < tree[v].box.y1 || tree[v].box.y2 < y) {
        return;
    }
    if (!tree[v].leaf) {
        for (int son : tree[v].sons) {
            locate(son, x, y, ans);
        }
        return;
    }
    for (int id : tree[v].rects) {
        if (!(x < a[id].x1 || a[id].x2 < x ||
            y < a[id].y1 || a[id].y2 < y)) {
            ans.push_back(a[id].id);
        }
    }
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    int n;
    cin >> n;
    a.resize(n);
    for (int i = 0; i < n; ++i) {
        cin >> a[i].x1 >> a[i].y1 >> a[i].x2 >> a[i].y2;
        a[i].id = i + 1;
    }
    int root = build_index(n);
    int q;
    cin >> q;
    for (int i = 0; i < q; ++i) {
        int x, y;
        cin >> x >> y;
        vector<int> ans;
        locate(root, x, y, ans);
        sort(ans.begin(), ans.end());
        cout << ans.size();
        for (int j = 0; j < ans.size(); ++j) {
            cout << ' ' << ans[j];
        }
        cout << endl;
    }

    return 0;
}
