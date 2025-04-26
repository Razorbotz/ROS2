#include "logic/search.hpp"
#include <cmath>
#include <stdexcept>

void Search::setRowCol(int row, int col){
	if(row > ROW)
		throw std::runtime_error("ROW out of bounds");
	if(col > COL)
		throw std::runtime_error("COL out of bounds");
	Row = row;
	Col = col;
}

void Search::initializeMap(){
    int buffer = std::ceil(Width / 2);
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			if(i >= buffer && i < Row - buffer && j >= buffer && j < Col - buffer){
				this->map[i][j] = 0;
			}
			else{
				this->map[i][j] = 3;
			}
		}
	}
}

void Search::initializeMap(float width){
	int buffer = std::ceil(width / 2);
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			if(i >= buffer && i < Row - buffer && j >= buffer && j < Col - buffer){
				this->map[i][j] = 0;
			}
			else{
				this->map[i][j] = 3;
			}
		}
	}
    Width = width;
}

void Search::setMap(int map[][COL]){
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			this->map[i][j] = map[i][j];
		}
	}
}

void Search::setObstacle(int x, int y, int type, int radius){
    int buffer = std::ceil(Width / 2);

    for(int i = (x - radius / 2) - buffer; i < x + (radius / 2) + buffer; i++){
        for(int j = (y - radius / 2) - buffer; j < y + (radius / 2) + buffer; j++){
            if(isValid(i, j)) {
                this->map[i][j] = type;
            }
        }
    }
}

void Search::setOpen(int x, int y){
    this->map[x][y] = 0;
}

bool Search::isValid(int x, int y){
    return (x >= 0) && (x < Row) && (y >= 0) && (y < Col);
}

bool Search::isOpen(int x, int y, bool includeHoles){
	if(includeHoles){
		if(!(this->map[x][y] == 0 || this->map[x][y] == 1)){
			obstacleFound = true;
		}
		return (this->map[x][y] == 0 || this->map[x][y] == 1);
	}
	else{
		if(this->map[x][y] != 0){
			obstacleFound = true;
		}
	    return this->map[x][y] == 0;
	}
}

bool Search::isDestination(int x, int y){
    return x == this->destX && y == this->destY;
}

double Search::calculateHeuristic(int x, int y){
    return pow(double(pow(int(x - this->destX), 2) + pow((y - this->destY), 2)), 0.5);
}

void Search::setStart(Point start){
    this->startX = start.x;
    this->startY = start.y;
}

void Search::setDest(Point dest){
    this->destX = dest.x;
    this->destY = dest.y;
}

void Search::printPath(std::stack<Coord> Path){
	printf("\nThe Path is ");
	while (!Path.empty()) {
		std::pair<int, int> p = Path.top();
		Path.pop();
		printf("-> (%d,%d) ", p.first, p.second);
	}
}

std::stack<Coord> Search::getPath(){
	int row = this->destX;
	int col = this->destY;

	std::stack<Coord> Path;

	while (!(this->cells[row][col].parent.x == row && this->cells[row][col].parent.y == col)) {
		Path.push(std::make_pair(row, col));
		int temp_row = this->cells[row][col].parent.x;
		int temp_col = this->cells[row][col].parent.y;
		row = temp_row;
		col = temp_col;
	}

	Path.push(std::make_pair(row, col));
	return Path;
}

void Search::initializeCells(){
	for (int i = 0; i < Row; i++) {
		for (int j = 0; j < Col; j++) {
			this->cells[i][j].totalCost = FLT_MAX;
			this->cells[i][j].cost = FLT_MAX;
			this->cells[i][j].heuristic = FLT_MAX;
			this->cells[i][j].parent.x = -1;
			this->cells[i][j].parent.y = -1;
		}
	}
}

bool Search::checkSuccessor(int i, int deltaX, int j, int deltaY, bool includeHoles){
	if(isValid(i+deltaX, j+deltaY)){
		if(isDestination(i+deltaX, j+deltaY)){
			this->cells[i+deltaX][j+deltaY].parent.x = i;
			this->cells[i+deltaX][j+deltaY].parent.y = j;
			return true;
		}
		else if(!this->closedList[i+deltaX][j+deltaY] && isOpen(i+deltaX, j+deltaY, includeHoles)){
			float cost = pow((pow(deltaX, 2) + pow(deltaY, 2)), 0.5);
			Search::newCost = this->cells[i][j].cost + cost;
			Search::newH = calculateHeuristic(i+deltaX, j+deltaY);
			Search::newTotal = Search::newCost + Search::newH;

			if (this->cells[i+deltaX][j+deltaY].totalCost == FLT_MAX || this->cells[i+deltaX][j+deltaY].totalCost > Search::newTotal) {
				this->openList.insert(std::make_pair(Search::newTotal, std::make_pair(i+deltaX, j+deltaY)));

				this->cells[i+deltaX][j+deltaY].totalCost = Search::newTotal;
				this->cells[i+deltaX][j+deltaY].cost = Search::newCost;
				this->cells[i+deltaX][j+deltaY].heuristic = Search::newH;
				this->cells[i+deltaX][j+deltaY].parent.x = i;
				this->cells[i+deltaX][j+deltaY].parent.y = j;
			}
		}
	}
	return false;
}

std::stack<Coord> Search::aStar(bool includeHoles){
	std::stack<Coord> Path;
	if (!isValid(this->startX, this->startY) || (!isValid(this->destX, this->destY))) {
		std::cout << "Invalid point." << std::endl;
		Path.push(std::make_pair(-2, -2));
		return Path;
	}

	if (!isOpen(this->startX, this->startY) || !isOpen(this->destX, this->destY)) {
		std::cout << "Closed point." << std::endl;
		Path.push(std::make_pair(-3, -3));
		return Path;
	}
	if (isDestination(this->startX, this->startY)) {
		Path.push(std::make_pair(this->startX, this->startY));
		return Path;
	}
	memset(this->closedList, false, sizeof(this->closedList));

	int i, j;

	initializeCells();

	i = this->startX, j = this->startY;
	this->cells[i][j].totalCost = 0.0;
	this->cells[i][j].cost = 0.0;
	this->cells[i][j].heuristic = 0.0;
	this->cells[i][j].parent.x = i;
	this->cells[i][j].parent.y = j;

	this->openList.insert(std::make_pair(0.0, std::make_pair(i, j)));

	while (!this->openList.empty()) {
		Node p = *this->openList.begin();

		this->openList.erase(this->openList.begin());

		i = p.second.first;
		j = p.second.second;
		this->closedList[i][j] = true;

		if(checkSuccessor(i, -1, j, 0, includeHoles))return getPath();
		if(checkSuccessor(i, 1, j, 0, includeHoles))return getPath();
		if(checkSuccessor(i, 0, j, -1, includeHoles))return getPath();
		if(checkSuccessor(i, 0, j, 1, includeHoles))return getPath();

		if(checkSuccessor(i, -1, j, 1, includeHoles))return getPath();
		if(checkSuccessor(i, -1, j, -1, includeHoles))return getPath();
		if(checkSuccessor(i, 1, j, 1, includeHoles))return getPath();
		if(checkSuccessor(i, 1, j, -1, includeHoles))return getPath();
	}

	Path.push(std::make_pair(-1, -1));
	return Path;
}

std::stack<Coord> Search::aStar(int grid[][COL], Point src, Point dest, bool includeHoles, bool simplify){
	setMap(grid);
	if(simplify)
		return getSimplifiedPath(aStar(src, dest));
	return aStar(src, dest);
}

std::stack<Coord> Search::aStar(Point src, Point dest, bool includeHoles, bool simplify){
	setStart(src);
	setDest(dest);
	if(simplify)
		return getSimplifiedPath(aStar(includeHoles));
	return aStar(includeHoles);
}

std::stack<Coord> Search::aStar(std::stack<Coord> points, bool includeHoles, bool simplify){
	
	auto origPoints = points; 
	std::stack<Coord> Path;
	auto origPoints = points;
	Coord top = points.top();
	Point start = Point(top.first, top.second);
	points.pop();
	bool first = true;
	while(!points.empty()){
		obstacleFound = false;
		top = points.top();
		Point dest = Point(top.first, top.second);
		points.pop();
		std::stack<Coord> newPath = aStar(start, dest, includeHoles);
		if(first)
			first = false;
		else
			newPath.pop();
		while(!newPath.empty()){
			Path.push(newPath.top());
			newPath.pop();
		}
		start = dest;
	}
	if(simplify)
		return getSimplifiedPath(Path, origPoints);
	return Path;
}

std::unordered_set<Coord> Search::stackToSet(const std::stack<Coord>& stk) {
    std::unordered_set<Coord> result;
    std::stack<Coord> temp = stk;

    while (!temp.empty()) {
        result.insert(temp.top());
        temp.pop();
    }

    return result;
}

bool Search::isCollinear(Coord p1, Coord p2, Coord p3) {
    return (long long)(p2.second - p1.second) * (p3.first - p2.first) ==
           (long long)(p3.second - p2.second) * (p2.first - p1.first);
}

bool Search::isObstacleBetweenPoints(Coord p1, Coord p2) {
    int x1 = p1.first, y1 = p1.second;
    int x2 = p2.first, y2 = p2.second;

    int dx = abs(x2 - x1), dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (!isValid(x1, y1) || !isOpen(x1, y1)) return true;

        if (x1 == x2 && y1 == y2) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x1 += sx; }
        if (e2 < dx)  { err += dx; y1 += sy; }
    }

    return false;
}

std::stack<Coord> Search::getSimplifiedPath(std::stack<Coord> rpath) {
    std::vector<Coord> pathVec;
    while (!rpath.empty()) {
        pathVec.push_back(rpath.top());
        rpath.pop();
    }
    std::reverse(pathVec.begin(), pathVec.end());

    if (pathVec.empty()) {
        return std::stack<Coord>();
    }

    std::vector<Coord> simplifiedPathVec;
    simplifiedPathVec.push_back(pathVec[0]);

    size_t i = 1;
    while (i < pathVec.size()) {
        Coord lastSimplified = simplifiedPathVec.back();
        size_t furthest = i;

        for (size_t j = i + 1; j < pathVec.size(); ++j) {
            if (!isObstacleBetweenPoints(lastSimplified, pathVec[j])) {
                furthest = j;
            } else {
                break;
            }
        }

        simplifiedPathVec.push_back(pathVec[furthest]);
        i = furthest + 1;
    }

    std::vector<Coord> finalPathVec;
    finalPathVec.push_back(simplifiedPathVec[0]);

    for (size_t i = 1; i < simplifiedPathVec.size(); ++i) {
        Coord current = simplifiedPathVec[i];
        Coord last = finalPathVec.back();

        if (finalPathVec.size() >= 2) {
            Coord secondLast = finalPathVec[finalPathVec.size() - 2];
            if (isCollinear(secondLast, last, current)) {
                finalPathVec.pop_back();
            }
        }
        finalPathVec.push_back(current);
    }

    std::stack<Coord> result;
    for (const auto& pt : finalPathVec) {
        result.push(pt);
    }

    return result;
}

std::stack<Coord> Search::getSimplifiedPath(std::stack<Coord> rpath, const std::stack<Coord> pointsToKeep) {
	std::unordered_set<Coord> mustKeep = stackToSet(pointsToKeep);
	std::vector<Coord> pathVec;
    while (!rpath.empty()) {
        pathVec.push_back(rpath.top());
        rpath.pop();
    }
    std::reverse(pathVec.begin(), pathVec.end());

    if (pathVec.empty()) {
        return std::stack<Coord>();
    }

    std::vector<Coord> simplifiedPathVec;
    simplifiedPathVec.push_back(pathVec[0]);

    size_t i = 1;
    while (i < pathVec.size()) {
        Coord lastSimplified = simplifiedPathVec.back();
        size_t furthest = i;

        for (size_t j = i + 1; j < pathVec.size(); ++j) {
            if (mustKeep.count(pathVec[j])) {
                furthest = j;
                break;
            }

            if (!isObstacleBetweenPoints(lastSimplified, pathVec[j])) {
                furthest = j;
            } else {
                break;
            }
        }

        simplifiedPathVec.push_back(pathVec[furthest]);
        i = furthest + 1;
    }

    std::vector<Coord> finalPathVec;
    finalPathVec.push_back(simplifiedPathVec[0]);

    for (size_t i = 1; i < simplifiedPathVec.size(); ++i) {
        Coord current = simplifiedPathVec[i];
        Coord last = finalPathVec.back();

        if (finalPathVec.size() >= 2) {
            Coord secondLast = finalPathVec[finalPathVec.size() - 2];
            if (isCollinear(secondLast, last, current) && !mustKeep.count(last)) {
                finalPathVec.pop_back();
            }
        }
        finalPathVec.push_back(current);
    }

    std::stack<Coord> result;
    for (const auto& pt : finalPathVec) {
        result.push(pt);
    }

    return result;
}

void Search::printMap(){
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			if(i == startX && j == startY)
				std::cout << "S ";
			else if(i == destX && j == destY)
				std::cout << "T ";
			else
				std::cout << map[i][j] << " ";
		}
		std::cout << std::endl;
	}
}

void Search::printMap(std::stack<Coord> points){
    std::set<Coord> pathCoords;
    Coord firstPathPoint = {-1, -1};
    Coord lastPathPoint = {-1, -1};

	lastPathPoint = points.top();
    bool isFirstElement = true;

    while(!points.empty()) {
        Coord current = points.top();

        if (isFirstElement) {
            isFirstElement = false;
        }

        firstPathPoint = current;

        pathCoords.insert(current);
        points.pop();
    }

	for(int i = 0; i < Row; i++){
        for(int j = 0; j < Col; j++){
            Coord currentCoord = {i, j};

            if(currentCoord == firstPathPoint) {
                std::cout << "S ";
            }
			else if(currentCoord == lastPathPoint) {
                std::cout << "T ";
            }
			else if (pathCoords.count(currentCoord) > 0) {
                std::cout << "* ";
            }
			else {
                switch (map[i][j]) {
                    case 0: std::cout << ". "; break;
                    case 1: std::cout << ". "; break;
                    case 2: std::cout << "# "; break;
                    case 3: std::cout << "X "; break;
                    default: std::cout << "? "; break;
                }
            }
        }
        std::cout << std::endl;
    }
}

void Search::addPointToStack(int x, int y){
    Coord point = Coord(x, y);
    points.push(point);
}