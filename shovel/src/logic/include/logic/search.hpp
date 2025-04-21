#include <bits/stdc++.h>
#define COL 82
#define ROW 50

struct Point{
    int x;
    int y;
    Point(){
        x = 0;
        y = 0;
    }
    Point(int X, int Y){
        x = X;
        y = Y;
    }
};

struct cell{
    Point parent;
    double totalCost, cost, heuristic;
};

typedef std::pair<int, int> Coord;
 
typedef std::pair<double, std::pair<int, int> > Node;
 
class Search{
    public:
    cell cells[ROW][COL];

    bool closedList[ROW][COL];

	std::set<Node> openList;

    int map[ROW][COL];

    int Row, Col;

    int startX, startY, destX, destY;
    double newCost, newH, newTotal;

    float Width = 0;

    std::stack<Coord> points;

    void setRowCol(int row, int col);

    void initializeMap();

    void initializeMap(float width);

    void setMap(int map[][COL]);

    void setObstacle(int x, int y, int type, int radius);

    void setOpen(int x, int y);

    bool isValid(int x, int y);

    bool isOpen(int x, int y, bool includeHoles = false);

    bool isDestination(int x, int y);

    double calculateHeuristic(int x, int y);

    void setStart(Point start);

    void setDest(Point dest);

    void initializeCells();

    bool checkSuccessor(int i, int deltaX, int j, int deltaY, bool includeHoles = false);

    void printPath(std::stack<Coord> Path);

    std::stack<Coord> getPath();

    std::stack<Coord> aStar(bool includeHoles = false);

    std::stack<Coord> aStar(Point src, Point dest, bool includeHoles = false, bool simplify = false);
    
    std::stack<Coord> aStar(int grid[][COL], Point src, Point dest, bool includeHoles = false, bool simplify = false);

    std::stack<Coord> aStar(std::stack<Coord> points, bool includeHoles = false, bool simplify = false);

    bool isCollinear(Coord p1, Coord p2, Coord p3);

    std::stack<Coord> getSimplifiedPath(std::stack<Coord> rpath);

    void printMap();

    void addPointToStack(int x, int y);
};