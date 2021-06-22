/****************************************************
 * Finding the path between source and destination
 * 
 * @author Sunil
 * @date   22.06.2021
 ****************************************************/ 

#include <algorithm>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using std::sort;
using std::cout;
using std::cin;
using std::abs;
using std::exit;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

// Enum that represent different State on grid
enum class State
{
    kEmpty,
    kObstacle,
    kClosed,
    kPath,
    kStart,
    kFinish
};

// Function prototypes
vector<vector<State>> readGridFile(string path);
vector<State> parseLine(string line);
vector<vector<State>> astarSearch(vector<vector<State>> grid, const int start[2], const int goal[2]);
int manhattanDistance(int x0, int y0, int x1, int y1);
void cellSort(vector<vector<int>> &frontier);
bool compare(const vector<int> &v1, const vector<int> &v2);
void expandNeighbours(vector<int> &curr, vector<vector<int>> &frontier, vector<vector<State>> &grid, const int goal[2]);
bool isValidCell(int x, int y, const vector<vector<State>> &grid);
void addPoint(int x, int y, int g, int h, vector<vector<int>> &frontier, vector<vector<State>> &grid);
void printGridData(const vector<vector<State>> &grid);
string cellString(const State &curr_state);

// Defining constants
const int GRID_CHANGE_MOVE[4][2] = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

// Definition of main()
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "Error: File path and name not provided\n";
        return 1;
    }

    // Reading grid data from file
    vector<vector<State>> grid = readGridFile(argv[1]);

    // Display grid data
    printGridData(grid);

    // Read start and goal coordinate
    int start[2];
    int goal[2];
    cout << "Enter start coordinate: ";
    cin >> start[0] >> start[1];
    cout << "Enter goal coordinate: ";
    cin >> goal[0] >> goal[1];

    // Performing search on grid
    vector<vector<State>> solution = astarSearch(grid, start, goal);

    // Display grid solution data
    printGridData(solution);
    
    
    return 0;
}

// Reading grid data from file
vector<vector<State>> readGridFile(string path)
{
    string line;
    vector<vector<State>> grid;
    ifstream myfile(path);

    // Checking that file is opened correcly or not
    if (myfile)
    {
        while (getline(myfile, line))
        {
            grid.push_back(parseLine(line));
        }
    }
    else 
    {
        cout << "Error: File not opened\n";
    }

    myfile.close();

    return grid;
}

// Parsing string using istringstream
vector<State> parseLine(string line)
{
    int ch;
    vector<State> row;
    istringstream sline(line);

    while (sline >> ch)
    {
        if (ch == 0)
        {
            row.push_back(State::kEmpty);
        }
        else 
        {
            row.push_back(State::kObstacle);
        }
    }

    return row; 
}

// Implementation of A* algorithm
vector<vector<State>> astarSearch(vector<vector<State>> grid, const int start[2], const int goal[2])
{
    int x = start[0];
    int y = start[1];
    int g = 0;
    int h = manhattanDistance(x, y, goal[0], goal[1]);
    vector<vector<int>> frontier;
    vector<int> curr;addPoint(x, y, g, h, frontier, grid);

    while (frontier.size())
    {
        cellSort(frontier);
        curr = frontier[frontier.size() - 1];
        frontier.pop_back();

        x = curr[0];
        y = curr[1];

        // Checking that we reach goal or not
        if (x == goal[0] && y == goal[1])
        {
            grid[start[0]][start[1]] = State::kStart;
            grid[x][y] = State::kFinish;
            return grid;
        }

        // Labeling as point in path
        grid[x][y] = State::kPath;

        expandNeighbours(curr, frontier, grid, goal);
    }

    cout << "No path found\n";

    return vector<vector<State>>{};
}

// Calculating Manhattan distance between two points
int manhattanDistance(int x0, int y0, int x1, int y1)
{
    return abs(x0 - x1) + abs(y0 - y1);
}

// Sorting frontier based on compare
void cellSort(vector<vector<int>> &frontier)
{
    sort(frontier.begin(), frontier.end(), compare);
}

// Comparison funciont for 'sort'
bool compare(const vector<int> &v1, const vector<int> &v2)
{
    return (v1[2] + v1[3]) > (v2[2] + v2[3]);
}

// Expanding all neighbours of current point
void expandNeighbours(vector<int> &curr, vector<vector<int>> &frontier, vector<vector<State>> &grid, const int goal[2])
{
    int x = curr[0];
    int y = curr[1];
    int g = curr[2];
    int h;
    int x1;
    int y1;

    for (int i = 0; i < 4; i++)
    {
        x1 = x + GRID_CHANGE_MOVE[i][0];
        y1 = y + GRID_CHANGE_MOVE[i][1];

        if (isValidCell(x1, y1, grid))
        {
            h = manhattanDistance(x1, y1, goal[0], goal[1]);
            addPoint(x1, y1, g, h, frontier, grid);
        }
    }

    return;
}

// Validating move
bool isValidCell(int x, int y, const vector<vector<State>> &grid)
{
    int n1 = grid.size();
    int n2 = grid[0].size();

    if (x >= 0 && x < n1 && y >= 0 && y < n2 && grid[x][y] == State::kEmpty)
    {
        return true;
    }

    return false;
}

// Adding point to frontier and labing it as kClosed
void addPoint(int x, int y, int g, int h, vector<vector<int>> &frontier, vector<vector<State>> &grid)
{
    frontier.push_back({x, y, g, h});
    grid[x][y] = State::kClosed;
}

// Printing grid data
void printGridData(const vector<vector<State>> &grid)
{
    for (const auto &row: grid)
    {
        for (const auto &curr_state: row)
        {
            cout << cellString(curr_state);
        }
        cout << "\n";
    }
    cout << "\n\n";
}

// Return string based on current state to display
string cellString(const State &curr_state)
{
    switch (curr_state)
    {
        case State::kObstacle:
            return "⛰️   ";
        case State::kStart:
            return "🚦  ";
        case State::kFinish:
            return "🏁  ";
        case State::kPath:
            return "🚗  ";
        default:
            return "0   ";
    }
}