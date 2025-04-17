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
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			this->map[i][j] = 0;
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
}

void Search::setMap(int map[][COL]){
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			this->map[i][j] = map[i][j];
		}
	}
}

void Search::setObstacle(int x, int y, int type){
    this->map[x][y] = type;
}

void Search::setOpen(int x, int y){
    this->map[x][y] = 0;
}

bool Search::isValid(int x, int y){
    return (x >= 0) && (x < Row) && (y >= 0) && (y < Col);
}

bool Search::isOpen(int x, int y, bool includeHoles){
	if(includeHoles){
		return (this->map[x][y] == 0 || this->map[x][y] == 1);
	}
	else{
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
	std::stack<Coord> Path;
	Coord top = points.top();
	Point start = Point(top.first, top.second);
	std::cout << top.first << " " << top.second << std::endl;
	points.pop();
	bool first = true;
	while(!points.empty()){
		top = points.top();
		Point dest = Point(top.first, top.second);
		points.pop();
		std::cout << top.first << " " << top.second << std::endl;
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
		return getSimplifiedPath(Path);
	return Path;
}

std::stack<Coord> Search::getSimplifiedPath(std::stack<Coord> rpath){
	std::stack<Coord> path;
	while (!rpath.empty()){
		std::pair<int, int> value = rpath.top();
		rpath.pop();
		path.push(value);
	}
	std::stack<Coord> simplifiedPath;
	std::pair<int, int> initial = path.top();
	path.pop();
	simplifiedPath.push(initial);
	bool xSecond = false;
	bool ySecond = false;
	bool topRight = false;
	bool topLeft = false;
	bool bottomLeft = false;
	bool bottomRight = false;
	while (!path.empty()) {
		std::pair<int, int> previous = simplifiedPath.top();
		std::pair<int, int> p = path.top();
		path.pop();
		if(previous.first == p.first){
			if(xSecond){
				simplifiedPath.pop();
			}
			xSecond = true;
			ySecond = false;
			topRight = false;
			topLeft = false;
			bottomLeft = false;
			bottomRight = false;
		}
		else if(previous.second == p.second){
			if(ySecond){
				simplifiedPath.pop();
			}
			xSecond = false;
			ySecond = true;
			topRight = false;
			topLeft = false;
			bottomLeft = false;
			bottomRight = false;
		}
		else if(previous.first == p.first + 1 &&  previous.second == p.second + 1){
			if(bottomRight){
				simplifiedPath.pop();
			}
			xSecond = false;
			ySecond = false;
			topLeft = false;
			topRight = false;
			bottomLeft = false;
			bottomRight = true;
		}
		else if(previous.first == p.first - 1 &&  previous.second == p.second + 1){
			if(bottomLeft){
				simplifiedPath.pop();
			}
			xSecond = false;
			ySecond = false;
			topLeft = false;
			topRight = false;
			bottomLeft = true;
			bottomRight = false;
		}
		else if(previous.first == p.first + 1 &&  previous.second == p.second - 1){
			if(topRight){
				simplifiedPath.pop();
			}
			xSecond = false;
			ySecond = false;
			topLeft = false;
			topRight = true;
			bottomLeft = false;
			bottomRight = false;
		}
		else if(previous.first == p.first - 1 &&  previous.second == p.second - 1){
			if(topLeft){
				simplifiedPath.pop();
			}
			xSecond = false;
			ySecond = false;
			topLeft = true;
			topRight = false;
			bottomLeft = false;
			bottomRight = false;
		}
		else{
			xSecond = false;
			ySecond = false;
			topRight = false;
			topLeft = false;
			bottomLeft = false;
			bottomRight = false;
		}
		simplifiedPath.push(p);
	}
	return simplifiedPath;
}

void Search::printMap(){
	for(int i = 0; i < Row; i++){
		for(int j = 0; j < Col; j++){
			if(i == startX && j == startY)
				std::cout << "X ";
			else if(i == destX && j == destY)
				std::cout << "T ";
			else
				std::cout << map[i][j] << " ";
		}
		std::cout << std::endl;
	}
}