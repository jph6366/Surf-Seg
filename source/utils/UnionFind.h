#include <vector>
#include <unordered_map>



class UnionFind {
public:
    UnionFind() {}

    void add(int x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            rank[x] = 0;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        if (rootX != rootY) {
            if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
        }
    }

    // Helper function to get all elements in the same set as a given element
    std::vector<int> getAllInSameSet(int element) {
        int root = find(element);  // Find the representative of the given element
        std::vector<int> sameSet;

        // Iterate over all elements and collect those with the same root
        for (const auto& [key, _] : parent) {
            if (find(key) == root) {
                sameSet.push_back(key);
            }
        }
        return sameSet;
    }

    std::vector<int> getAllNotInSameSet(int element) {
        int root = find(element);  // Find the representative of the given element
        std::vector<int> sameSet;

        // Iterate over all elements and collect those with the same root
        for (const auto& [key, _] : parent) {
            if (find(key) != root) {
                sameSet.push_back(key);
            }
        }
        return sameSet;
    }

private:
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
};

// Function to print all elements in the same set as the given element
void printAllInSameSet(UnionFind &uf, int element) {
    std::vector<int> sameSet = uf.getAllInSameSet(element);
    std::cout << "Elements in the same set as " << element << ": ";
    for (int x : sameSet) {
        std::cout << x << " ";
    }
    std::cout << std::endl;
}