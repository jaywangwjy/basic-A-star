#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm> 
#include <functional>

using namespace std;

constexpr int gridSize = 8;

// Define a coordinate structure for 2D points
struct Coordinate {
    int x;
    int y;

    Coordinate(int x, int y) : x(x), y(y) {}
};

// Define a node structure to represent cells in the grid
struct Node {
    Coordinate coordinates;
    int G; // Cost from start to this node
    int H; // Heuristic (Manhattan distance to the target)
    Node* parent;

    Node(int x, int y) : coordinates(x, y), G(0), H(0), parent(nullptr) {}

    int F() const { return G + H; }
};

// Helper function to calculate the Manhattan distance heuristic
int manhattanDistance(const Coordinate& source, const Coordinate& target) {
    return abs(source.x - target.x) + abs(source.y - target.y);
}

// Helper function to check if a coordinate is within the grid boundaries
bool isValidCoordinate(int x, int y) {
    return x >= 0 && x < gridSize && y >= 0 && y < gridSize;
}

// Helper function to check if a coordinate is traversable (not an obstacle)
bool isTraversable(const vector<vector<int>>& grid, int x, int y) {
    return isValidCoordinate(x, y) && grid[y][x] == 0;
}

// A* algorithm to find the shortest path from (0, 0) to (7, 7) in the grid
vector<Coordinate> findShortestPath(const vector<vector<int>>& grid) {
    const Coordinate target(7, 7);

    // Priority queue for open nodes sorted by their F value (lowest first)
    priority_queue<Node*, vector<Node*>, function<bool(Node*, Node*)>> openSet(
        [](Node* a, Node* b) { return a->F() > b->F(); }
    );

    // 2D array to keep track of visited nodes
    vector<vector<bool>> visited(gridSize, vector<bool>(gridSize, false));

    // Start from (0, 0)
    Node* startNode = new Node(0, 0);
    startNode->H = manhattanDistance(startNode->coordinates, target);
    openSet.push(startNode);

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->coordinates.x == target.x && current->coordinates.y == target.y) {
            // Path found, reconstruct and return the path
            vector<Coordinate> path;
            while (current != nullptr) {
                path.push_back(current->coordinates);
                current = current->parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        visited[current->coordinates.y][current->coordinates.x] = true;

        // Explore neighbors
        int dx[] = { 0, 1, 0, -1 };
        int dy[] = { -1, 0, 1, 0 };
        for (int i = 0; i < 4; ++i) {
            int newX = current->coordinates.x + dx[i];
            int newY = current->coordinates.y + dy[i];

            if (isTraversable(grid, newX, newY) && !visited[newY][newX]) {
                Node* neighbor = new Node(newX, newY);
                neighbor->G = current->G + 1;
                neighbor->H = manhattanDistance(neighbor->coordinates, target);
                neighbor->parent = current;
                openSet.push(neighbor);
            }
        }
    }

    // No path found
    return {};
}

int main() {
    // Manually define the specific grid
    vector<vector<int>> grid = {
        { 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 1, 0 },
        { 0, 0, 0, 1, 1, 1, 0, 0 },
        { 0, 0, 1, 1, 1, 1, 0, 0 },
        { 0, 0, 0, 1, 1, 1, 0, 0 },
        { 0, 0, 0, 0, 1, 1, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0 },
        { 0, 0, 0, 0, 0, 0, 0, 0 }
    };

    // Print the specified grid
    cout << "Generated Grid:" << endl;
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            cout << grid[y][x] << " ";
        }
        cout << endl;
    }

    // Find the shortest path using A* algorithm
    vector<Coordinate> path = findShortestPath(grid);

    // Print the path
    if (!path.empty()) {
        cout << "\nShortest Path:" << endl;
        for (size_t i = 0; i < path.size(); ++i) {
            cout << "(" << path[i].x << ", " << path[i].y << ")";
            if (i != path.size() - 1) {
                cout << " -> ";
            }

            // Mark the path in the grid with value 5
            grid[path[i].y][path[i].x] = 5;
        }
        cout << endl;
    } else {
        cout << "\nNo path found!" << endl;
    }

    // Print the updated grid with the shortest path
    cout << "\nShortest Path Visualized:" << endl;
    for (int y = 0; y < gridSize; ++y) {
        for (int x = 0; x < gridSize; ++x) {
            cout << grid[y][x] << " ";
        }
        cout << endl;
    }

    return 0;
}