#include <algorithm>
#include <iostream>
#include <queue>
#include <stack>
#include <vector>

#include "API.h"

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
constexpr float DIRECTION_CHANGE_PENALTY = 0.95f;
constexpr float MAX_PENALTY = 5.0f;
constexpr float acceptableError = 0.05f;

struct pos {
  short x, y;
  pos() : x(-1), y(-1) {};
  pos(const int x, const int y) : x(x), y(y) {};
  pos(const pos& other) : x(other.x), y(other.y) {};
  bool operator==(const pos& other) const {
    return (x == other.x) && (y == other.y);
  }
  bool operator!=(const pos& other) const {
    return (x != other.x) || (y != other.y);
  }
  bool operator<(const pos& other) const {
    return x < other.x || (x == other.x && y < other.y);
  }
  bool operator>(const pos& other) const {
    return x > other.x || (x == other.x && y > other.y);
  }
  void operator+=(const pos& other) {
    x += other.x;
    y += other.y;
  }
  pos operator+(const pos& other) { return pos(x + other.x, y + other.y); }
};

const pos nul{-1, -1};
const pos startPos(0, 0);
const pos goalPos[] = {{7, 7}, {7, 8}, {8, 7}, {8, 8}};
const pos drs[] = {{-1, 0}, {0, 1}, {1, 0}, {0, -1}};
const pos drx[] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
const pos dr8[] = {{-1, 0}, {-1, 1}, {0, 1},  {1, 1},
                   {1, 0},  {1, -1}, {0, -1}, {-1, -1}};
pos prevP[2 * MAZE_HEIGHT][2 * MAZE_WIDTH];
pos currPos = startPos;
pos hfGoal(7, 7), fGoal = nul;
int facing = 1;  // floodfill: 0: left 1: up 2: right 3: down |
                 // aStar: 0: west 1: north west 2: north 3: north east 4: east
                 // 5: south east 6: south 7: south west
std::uint8_t maze[MAZE_HEIGHT][MAZE_WIDTH];
unsigned short bfs_map[MAZE_HEIGHT][MAZE_WIDTH];
constexpr unsigned short INF = MAZE_WIDTH * MAZE_HEIGHT + 1;
constexpr char directions[] = {'w', 'n', 'e', 's'};
bool surrounds[3];
unsigned int aStar1, aStar2;

void bitSet(std::uint8_t& byte, const int pos, const bool value) {
  byte = (byte & ~(1 << pos)) | (value << pos);
}

float heuristic(const pos& a, const pos& b, bool isDirectionChange = false) {
  float chebyshevDistance = std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
  float penalty =
      std::min(MAX_PENALTY, DIRECTION_CHANGE_PENALTY * chebyshevDistance);
  return chebyshevDistance - (isDirectionChange ? penalty : 0);
}

void move(const int dir) {
  while (facing != dir) {
    int rightSteps = (dir - facing + 4) % 4;
    int leftSteps = (facing - dir + 4) % 4;

    if (rightSteps <= leftSteps) {
      API::turnRight();
      facing = (facing + 1) % 4;
    } else {
      API::turnLeft();
      facing = (facing - 1 + 4) % 4;
    }
  }
  API::moveForward();
  currPos += drs[dir];
}

void rotate(const int dir) {
  int rightSteps = (dir - facing + 8) % 8;
  int leftSteps = (facing - dir + 8) % 8;

  while (facing != dir) {
    if (rightSteps <= leftSteps) {
      if (rightSteps % 2 == 0) {
        API::turnRight();
        facing = (facing + 2) % 8;  // 90-degree turn
        rightSteps -= 2;
      } else {
        API::turnRight45();
        facing = (facing + 1) % 8;  // 45-degree turn
        rightSteps -= 1;
      }
    } else {
      if (leftSteps % 2 == 0) {
        API::turnLeft();
        facing = (facing - 2 + 8) % 8;  // 90-degree turn
        leftSteps -= 2;
      } else {
        API::turnLeft45();
        facing = (facing - 1 + 8) % 8;  // 45-degree turn
        leftSteps -= 1;
      }
    }
  }
}

void moveDiag(pos& dest) {
  for (int i = 0; i < 8; ++i) {
    if (currPos + dr8[i] == dest) {
      rotate(i);
      API::moveForwardHalf();
      currPos = dest;
      return;
    }
  }
}

bool isValid(const pos& p) {
  return ((p.x >= 0) && (p.y >= 0) && (p.x < MAZE_WIDTH) &&
          (p.y < MAZE_HEIGHT));
}

int getBestMove() {
  pos p_temp;
  int min = INF;
  float minh = INF;
  int bestDir = -1;
  for (int i = 0; i < 4; ++i) {
    p_temp = currPos + drs[i];
    if (!isValid(p_temp) || maze[currPos.y][currPos.x] & (1 << i)) continue;
    if (bfs_map[p_temp.y][p_temp.x] < min) {
      min = bfs_map[p_temp.y][p_temp.x];
      minh = heuristic(p_temp, hfGoal);
      bestDir = i;
    } else if (bfs_map[p_temp.y][p_temp.x] == min) {
      float h = heuristic(p_temp, hfGoal);
      if (h < minh) {
        minh = h;
        bestDir = i;
      }
    }
  }
  return bestDir;
}

std::stack<pos> getPath() {
  std::stack<pos> path;
  pos p = fGoal;
  while (p != startPos) {
    path.push(p);
    p = prevP[p.y][p.x];
  }
  return path;
}

bool isGoal(const pos& p) {
  for (const pos& goal : goalPos)
    if (p == goal) return true;
  return false;
}

bool isFinish() {
  for (const pos& goal : goalPos)
    if (currPos == goal) return true;
  return false;
}

bool isEven(unsigned int x) { return (x % 2 == 0); }

bool isValidDiagonal(const pos& p) {
  return ((p.x >= 0) && (p.y >= 0) && (p.x < 2 * MAZE_WIDTH) &&
          (p.y < 2 * MAZE_HEIGHT));
}

bool isReachable(const pos p, bool ignoreSeen = false) {
  if (!isValidDiagonal(p)) return false;
  if (isEven(p.x) && isEven(p.y)) return true;
  pos op = {p.x - !isEven(p.x), p.y - !isEven(p.y)};
  int offset = (p.y > op.y) + 2 * (p.x > op.x);
  std::uint8_t block = maze[op.y / 2][op.x / 2];
  return (ignoreSeen || (block & (1 << (offset + 4)))) &&
         !(block & (1 << offset));  //(block & (1 << (offset + 4))) &&
}

bool areCollinear(const pos& A, const pos& B, const pos& C) {
  return (B.x - A.x) * (C.y - A.y) == (B.y - A.y) * (C.x - A.x);
}

bool aStar(bool ignoreSeen = false) {
  std::priority_queue<std::tuple<float, int, pos, pos>,
                      std::vector<std::tuple<float, int, pos, pos>>,
                      std::greater<std::tuple<float, int, pos, pos>>>
      openList;
  std::fill(&prevP[0][0], &prevP[0][0] + 2 * MAZE_WIDTH * 2 * MAZE_HEIGHT, nul);
  openList.push({0, 0, startPos, startPos});

  while (!openList.empty()) {
    auto [fCost, gCost, current, prev] = openList.top();
    gCost += 1;
    openList.pop();
    if (prevP[current.y][current.x] != nul) continue;
    prevP[current.y][current.x] = prev;
    if (current == fGoal) {
      if (ignoreSeen)
        aStar2 = gCost;
      else
        aStar1 = gCost;
      return true;
    }
    // normal position
    for (int i = 0; i < 4; ++i) {
      if (!isEven(current.x) && !isEven(i)) continue;
      if (!isEven(current.y) && isEven(i)) continue;
      pos next = current + drs[i];
      if (!isReachable(next, ignoreSeen) || (prevP[next.y][next.x] != nul))
        continue;
      openList.push(
          {gCost + heuristic(next, fGoal, areCollinear(prev, current, next)),
           gCost, next, current});
    }
    // nowhere position
    if (isEven(current.x) && isEven(current.y)) continue;
    for (int i = 0; i < 4; ++i) {
      pos next = current + drx[i];
      if (!isReachable(next, ignoreSeen) || (prevP[next.y][next.x] != nul))
        continue;
      openList.push(
          {gCost + heuristic(next, fGoal, areCollinear(prev, current, next)),
           gCost, next, current});
    }
  }
  return false;
}

void bfs() {
  std::fill(&bfs_map[0][0], &bfs_map[0][0] + MAZE_WIDTH * MAZE_HEIGHT, INF);
  pos p_front, p_temp;
  std::uint8_t block;
  std::queue<pos> q;

  for (const pos& p : goalPos) {
    bfs_map[p.y][p.x] = 0;
    q.push(p);
  }

  while (!q.empty()) {
    p_front = q.front();
    q.pop();
    // get surrounding of a block
    block = maze[p_front.y][p_front.x];
    for (int i = 0; i < 4; ++i)
      if (!(block & (1 << i))) {
        p_temp = p_front + drs[i];
        if (!isValid(p_temp) || bfs_map[p_temp.y][p_temp.x] != INF ||
            isGoal(p_temp))
          continue;
        bfs_map[p_temp.y][p_temp.x] = bfs_map[p_front.y][p_front.x] + 1;
        q.push(p_temp);
      }
  }
}

void debugUpdateWalls() {  // simulation only
  if (surrounds[0])
    API::setWall(currPos.x, currPos.y, directions[(facing + 3) % 4]);
  if (surrounds[1]) 
    API::setWall(currPos.x, currPos.y, directions[facing]);
  if (surrounds[2])
    API::setWall(currPos.x, currPos.y, directions[(facing + 1) % 4]);
}

bool updateSurrounding() {
  int offset;
  pos p_temp;
  std::uint8_t prev = maze[currPos.y][currPos.x];
  surrounds[0] = API::wallLeft();
  surrounds[1] = API::wallFront();
  surrounds[2] = API::wallRight();
  debugUpdateWalls();
  for (int i = 0; i < 3; ++i) {
    offset = (facing + i + 3) % 4;
    bitSet(maze[currPos.y][currPos.x], offset, surrounds[i]);
    bitSet(maze[currPos.y][currPos.x], offset + 4, 1);
    p_temp = currPos + drs[offset];
    if (isValid(p_temp)) {
      bitSet(maze[p_temp.y][p_temp.x], (offset + 2) % 4, surrounds[i]);
      bitSet(maze[p_temp.y][p_temp.x], (offset + 2) % 4 + 4, 1);
    }
  }
  return prev != maze[currPos.y][currPos.x];
}

void init() {
  memset(maze, 0, sizeof(std::uint8_t) * MAZE_WIDTH * MAZE_HEIGHT);
  return;
}

void reset() {
  API::ackReset();
  currPos = startPos;
  facing = 1;
}

bool aStarTrigger() {
  aStar(false);
  aStar(true);
  return ((aStar1 - aStar2) / (aStar1 * 1.f) <= acceptableError) ||
         aStar1 == aStar2;
}

int main(int argc, char* argv[]) {
  std::stack<pos> path;
  bool breakFlag = false;
  int bestDir, ct = 0;

  init();
  while (true) {
    while (!isFinish()) {
      ++ct;
      if (updateSurrounding()) {
        bfs();
        if (fGoal != nul && aStarTrigger()) {
          breakFlag = true;
          break;
        }
      }
      bestDir = getBestMove();
      if (bestDir == -1) {
        std::cerr << "No path found!" << std::endl;
        return 0;
      }
      move(bestDir);
      API::setColor(currPos.x, currPos.y, 'G');
    }
    if (breakFlag) {
      std::cerr << "Breaked by A*!" << std::endl;
      break;
    }
    if (fGoal == nul) {
      hfGoal = currPos;
      fGoal = currPos;
      fGoal.x *= 2;
      fGoal.y *= 2;
    } else if (aStarTrigger())
      break;
    if (ct == bfs_map[startPos.y][startPos.x]) {
      std::cerr << "Breaked by BFS!" << std::endl;
      break;
    }
    ct = 0;
    reset();
  }
  reset();
  facing = 2;
  aStar();
  path = getPath();
  while (!path.empty()) {
    pos next = path.top();
    path.pop();
    moveDiag(next);
    API::setColor(currPos.x / 2, currPos.y / 2, 'C');
  }
  return 0;
}