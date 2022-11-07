#include <algorithm>
#include <climits>
#include <functional>
#include <map>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

template <typename T>
class graph {
public:
    int number_of_edges = 0;
    std::unordered_map<T, std::vector<T>> adj;

    void add_node(T u) {
        adj[u];
    }

    void add_edge(T u, T v) {
        number_of_edges++;
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    int number_of_nodes() {
        return adj.size();
    }

    void remove_node(T u) {
        if (adj.find(u) != adj.end()) {
            number_of_edges -= adj[u].size();
            for (auto i : adj[u]) {
                adj[i].erase(find(adj[i].begin(), adj[i].end(), u));
            }
            adj.erase(u);
        }
    }

    void remove_edge(T u, T v) {
        if (adj.find(u) != adj.end()) {
            auto it = find(adj[u].begin(), adj[u].end(), v);
            if (it != adj[u].end()) {
                adj[u].erase(it);
                adj[v].erase(find(adj[v].begin(), adj[v].end(), u));
                number_of_edges--;
            }
        }
    }

    int degree(T u) {
        return adj[u].size();
    }

    void clear() {
        adj.clear();
    }

    std::vector<T> bfs(T s) {
        std::vector<T> bfs;
        std::unordered_map<T, bool> vis;
        vis[s] = true;
        std::queue<T> q;
        q.push(s);
        while (!q.empty()) {
            T v = q.front();
            q.pop();
            bfs.push_back(v);
            for (auto u : adj[v]) {
                if (!vis[u]) {
                    q.push(u);
                    vis[u] = true;
                }
            }
        }
        return bfs;
    }

    std::vector<T> dfs(T u) {
        std::vector<T> dfs;
        std::unordered_map<T, bool> vis;
        vis[u] = true;
        std::stack<T> s;
        s.push(u);
        while (!s.empty()) {
            T v = s.top();
            s.pop();
            dfs.push_back(v);
            for (auto u : adj[v]) {
                if (!vis[u]) {
                    s.push(u);
                    vis[u] = true;
                }
            }
        }
        return dfs;
    }

    int number_of_connected_components() {
        int n(0);
        std::unordered_map<T, bool> vis;
        std::stack<T> s;
        for (auto it = adj.begin(); it != adj.end(); it++) {
            if (!vis[it->first]) {
                vis[it->first] = true;
                n++;
                s.push(it->first);
                while (!s.empty()) {
                    T v = s.top();
                    s.pop();
                    for (auto u : adj[v]) {
                        if (!vis[u]) {
                            s.push(u);
                            vis[u] = true;
                        }
                    }
                }
            }
        }
        return n;
    }

    bool cyclic() {
        std::unordered_map<T, bool> vis;
        std::unordered_map<T, T> p;

        std::function<bool(T, T)> dfs = [&](T v, T _p) {
            vis[v] = true;
            for (auto i : adj[v]) {
                if (i == _p) {
                    continue;
                }
                if (vis[i]) {
                    return true;
                }
                p[i] = v;
                return dfs(i, v);
            }
            return false;
        };

        for (auto i : adj) {
            if (!vis[i.first] && dfs(i.first, p[i.first])) {
                return true;
            }
        }
        return false;
    }

    bool is_bipartite() {
        bool flag = true;
        std::unordered_map<T, int> side;
        std::queue<T> q;

        for (auto i : adj) {
            if (side[i.first] == 0) {
                q.push(i.first);
                side[i.first] = 1;
                while (!q.empty()) {
                    T v = q.front();
                    q.pop();
                    for (auto u : adj[v]) {
                        if (side[u] == 0) {
                            side[u] = !side[v];
                            q.push(u);
                        } else {
                            flag &= side[u] != side[v];
                            if (!flag) {
                                return false;
                            }
                        }
                    }
                }
            }
        }
        return true;
    }
};

template <typename T>
class digraph {
public:
    int number_of_edges = 0;
    std::unordered_map<T, std::vector<T>> adj;
    std::unordered_map<T, int> in_degree;

    void add_node(T u) {
        adj[u];
    }

    void add_edge(T u, T v) {
        number_of_edges++;
        adj[u].push_back(v);
        adj[v];
        in_degree[v]++;
    }

    int number_of_nodes() {
        return adj.size();
    }

    int out_degree(T u) {
        return adj[u].size();
    }

    void remove_node(T u) {
        if (adj.find(u) != adj.end()) {
            for (auto &i : adj) {
                if (i.first == u) {
                    number_of_edges -= i.second.size();
                    for (auto j : i.second) {
                        in_degree[j]--;
                    }
                    continue;
                }

                auto it = find(i.second.begin(), i.second.end(), u);
                if (it != i.second.end()) {
                    i.second.erase(it);
                    number_of_edges--;
                }
            }
            adj.erase(u);
            in_degree.erase(u);
        }
    }