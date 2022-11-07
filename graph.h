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
    
    void remove_edge(T u, T v) {
        if (adj.find(u) != adj.end()) {
            auto it = find(adj[u].begin(), adj[u].end(), v);
            if (it != adj[u].end()) {
                adj[u].erase(it);
                in_degree[v]--;
                number_of_edges--;
            }
        }
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

    bool cyclic() {
        return topological_sort().size() != adj.size();
    }

    std::vector<T> topological_sort() {
        std::vector<T> ans;
        std::unordered_map<T, int> inDegree = in_degree;
        std::queue<T> q;
        for (auto i : adj) {
            if (inDegree[i.first] == 0) {
                q.push(i.first);
            }
        }

        while (!q.empty()) {
            T v = q.front();
            q.pop();
            ans.push_back(v);
            for (auto i : adj[v]) {
                inDegree[i]--;
                if (inDegree[i] == 0) {
                    q.push(i);
                }
            }
        }

        if (ans.size() == adj.size()) {
            return ans;
        }
        return {};
    }

    // For printing SCCs in a directed graph
    void dfs(T node, std::stack<T> &st, std::unordered_map<T, bool> &vis, std::unordered_map<T, std::vector<T>> &adj) {
        vis[node] = true;
        for (auto it : adj[node]) {
            if (!vis[it]) {
                dfs(it, st, vis, adj);
            }
        }
        st.push(node);
    }

    void revDfs(int node, std::unordered_map<T, bool> &vis, std::unordered_map<T, std::vector<T>> &transpose, std::vector<T> &SCC) {
        SCC.push_back(node);
        vis[node] = true;
        for (auto it : transpose[node]) {
            if (!vis[it]) {
                revDfs(it, vis, transpose, SCC);
            }
        }
    }

    // Driver code for sccs in a digraph
    std::vector<std::vector<T>> SCCs() {
        std::vector<std::vector<T>> SCCsOfGraph;
        std::stack<int> st;
        std::unordered_map<T, bool> vis;
        for (auto it : adj) {
            if (!vis[it.first]) {
                dfs(it.first, st, vis, adj);
            }
        }

        std::unordered_map<T, std::vector<T>> transpose;
        for (auto it : adj) {
            vis[it.first] = false;
            for (auto itr : adj[it.first]) {
                transpose[itr].push_back(it.first);
            }
        }

        while (!st.empty()) {
            int node = st.top();
            st.pop();
            if (!vis[node]) {
                std::vector<T> SCC;
                revDfs(node, vis, transpose, SCC);
                SCCsOfGraph.push_back(SCC);
            }
        }
        return SCCsOfGraph;
    } // Driver code ends

    int number_of_SCCs() {
        std::vector<std::vector<T>> SCCs = this->SCCs();
        return SCCs.size();
    }
};

template <typename T>
class wgraph {
public:
    std::map<T, std::vector<std::pair<T, int>>> adj;
    int number_of_edges = 0;

    void add_edge(T v, T w, int k = 1) {
        adj[v].push_back({w, k});
        adj[w].push_back({v, k});
    }

    void add_node(T u) {
        adj[u];
    }

    void remove_node(T u) {
        number_of_edges -= adj[u].size();
        for (auto i : adj[u]) {
            int p = 0;
            for (auto it : adj[i.first]) {
                if (it.first == u) {
                    adj[i.first].erase(adj[i.first].begin() + p);
                }
                p++;
            }
        }
        adj.erase(u);
    }

    void remove_edge(T u, T v) {
        std::vector<std::pair<T, int>> V1 = adj[u];
        std::vector<std::pair<T, int>> V2 = adj[v];
        int p = 0;
        for (auto it : V1) {
            if (it.first == v) {
                adj[u].erase(adj[u].begin() + p);
            }
            p++;
        }

        p = 0;
        for (auto it : V2) {
            if (it.first == u) {
                adj[v].erase(adj[v].begin() + p);
            }
            p++;
        }
        number_of_edges--;
    }
