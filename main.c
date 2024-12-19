// Build like any raylib project. 
// Msvc build command: 
// Make sure to change the path raylib
// cl.exe /nologo /Zi gdi32.lib kernel32.lib msvcrt.lib opengl32.lib raylib.lib shell32.lib user32.lib winmm.lib teardrop.c -Id:\raylib\include /link /libpath:d:\raylib\lib /NODEFAULTLIB:libcmt
// 

#include <math.h>
#include "raylib.h"

typedef struct Enemy {

    int x;
    int y;

} Enemy;

typedef struct PathFindingNode {

    int         x;
    int         y;
    int         obstacle;
    int         visited;
    float       globalGoal;
    float       localGoal;
    struct      PathFindingNode *parent;

} PathFindingNode;

typedef struct {

    int x, y;
    float risk;

} QueueNode;

#define MAX_GRID    50
#define CELL_EMPTY  0
#define CELL_WALL   1

int     screenWidth                         = 1024;
int     screenHeight                        = 1024;
int     cellSize                            = 14;
int     editMode                            = 0;
float   moveSpeed                           = 30.0f;
float   maxRisk                             = 0.0f;
float   riskPropogationFactor               = 0.9f;
float   riskGrid[MAX_GRID][MAX_GRID]        = {0};
int     pathLength                          = 0;
Color   backgroundColor                     = {21, 33, 42, 255};
Color   backgroundColorEditing              = {21, 21, 21, 255};
Color   gridColor                           = {31, 51, 63, 255};
Color   enemyColor                          = {206, 191, 158, 255};
Color   cellHoverColor                      = {150, 150, 150, 90};
Color   wallColor                           = {91, 166, 156, 90};
char    worldGrid[MAX_GRID][MAX_GRID]       = {0};
PathFindingNode nodes[MAX_GRID][MAX_GRID]   = {0};
PathFindingNode *path[MAX_GRID * MAX_GRID]  = {0};
Enemy   enemy1                              = {10, 5};


float distance (PathFindingNode *a, PathFindingNode *b) {

    return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
}

int solveAStar (int startX, int startY, int targetX, int targetY) {

    int result = 0;
    int i;
    int x, y;
    int dx, dy;

    // InitializeNodes();
    for (x = 0; x < MAX_GRID; x++) {

        for (y = 0; y < MAX_GRID; y++) {

            nodes[x][y] = (PathFindingNode){x, y, worldGrid[x][y] == CELL_WALL, 0, INFINITY, INFINITY, NULL};
        }
    }

    PathFindingNode *startNode = &nodes[startX][startY];
    PathFindingNode *targetNode = &nodes[targetX][targetY];

    startNode->localGoal = 0.0f;
    startNode->globalGoal = distance(startNode, targetNode);

    PathFindingNode *nodeList[MAX_GRID * MAX_GRID];
    int nodeCount = 0;
    nodeList[nodeCount++] = startNode;

    while (nodeCount > 0) {
        // Sort by global goal
        for (i = 0; i < nodeCount - 1; i++) {
            if (nodeList[i]->globalGoal > nodeList[i + 1]->globalGoal) {
                PathFindingNode *temp = nodeList[i];
                nodeList[i] = nodeList[i + 1];
                nodeList[i + 1] = temp;
            }
        }

        PathFindingNode *currentNode = nodeList[0];
        if (currentNode == targetNode)
            break;

        for (i = 1; i < nodeCount; i++)
            nodeList[i - 1] = nodeList[i];
        nodeCount--;

        currentNode->visited = 1;

        for (dx = -1; dx <= 1; dx++) {
            for (dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0)
                    continue;

                if (abs(dx) + abs(dy) > 1)
                    continue;

                int nx = currentNode->x + dx;
                int ny = currentNode->y + dy;

                if (nx < 0 || ny < 0 || nx >= MAX_GRID || ny >= MAX_GRID)
                    continue;

                PathFindingNode *neighbor = &nodes[nx][ny];

                if (neighbor->obstacle || neighbor->visited)
                    continue;

                // Combine distance and risk for the cost
                float riskFactor = riskGrid[nx][ny] / maxRisk * 10;
                float tentativeLocalGoal = currentNode->localGoal + distance(currentNode, neighbor) + riskFactor;

                if (tentativeLocalGoal < neighbor->localGoal) {

                    neighbor->parent = currentNode;
                    neighbor->localGoal = tentativeLocalGoal;
                    neighbor->globalGoal = neighbor->localGoal + distance(neighbor, targetNode);
                    nodeList[nodeCount++] = neighbor;
                }
            }
        }
    }

    pathLength = 0;
    PathFindingNode *current = targetNode;
    while (current->parent) {
        path[pathLength++] = current;
        current = current->parent;
    }

    result = pathLength;

    return result;
}


bool isValidCell (int x, int y) {

    return x >= 0 && y >= 0 && x < MAX_GRID && y < MAX_GRID && worldGrid[x][y] != CELL_WALL;
}

bool hasLineOfSight (int x1, int y1, int x2, int y2) {

    int dx = x2 - x1;
    int dy = y2 - y1;
    int length = sqrtf(dx * dx + dy * dy);

    if (length == 0) {
        return true; // Same point, always has line of sight
    }

    dx /= length;
    dy /= length;

    int i;
    for (i = 1; i <= length; i++) {
        int nx = x1 + dx * i;
        int ny = y1 + dy * i;

        if (nx < 0 || ny < 0 || nx >= MAX_GRID || ny >= MAX_GRID) {
            return false; // Out of bounds
        }

        if (worldGrid[nx][ny] == CELL_WALL) {
            return false;
        }
    }

    return true;
}


void shootGun (int startX, int startY, int targetX, int targetY) {

    memset(riskGrid, 0, sizeof(riskGrid)); // Reset risk grid

    // Static queue for flood-fill
    QueueNode queue[MAX_GRID * MAX_GRID];
    int front = 0;
    int rear = 0;

    queue[rear++] = (QueueNode){startX, startY, 1.0f};
    riskGrid[startX][startY] = 1.0f;

    // Flood-fill algorithm
    while (front < rear) {

        QueueNode current = queue[front++];
        int x = current.x;
        int y = current.y;
        float risk = current.risk;

        if (hasLineOfSight(x, y, targetX, targetY)) {

            // Propagate risk in the direction of the enemy
            int dx = targetX - x;
            int dy = targetY - y;
            int length = sqrtf(dx * dx + dy * dy);

            if (length > 0) {
                dx /= length;
                dy /= length;

                int i;
                for (i = 1; i <= length; i++) {
                    int nx = x + dx * i;
                    int ny = y + dy * i;

                    if (isValidCell(nx, ny)) {
                        float newRisk = risk * riskPropogationFactor;

                        if (newRisk > maxRisk)
                            maxRisk = newRisk;

                        if (newRisk > riskGrid[nx][ny]) {
                            riskGrid[nx][ny] = newRisk;
                            if (rear < MAX_GRID * MAX_GRID) {
                                queue[rear++] = (QueueNode){nx, ny, newRisk};
                            }
                        }
                    }
                }
            } else {

                // If the length is zero, the enemy is at the same position
                // So, we can just update the risk at the current position
                float newRisk = risk * riskPropogationFactor;
                if (newRisk > riskGrid[x][y]) {

                    riskGrid[x][y] = newRisk;
                    if (rear < MAX_GRID * MAX_GRID) {
                        queue[rear++] = (QueueNode){x, y, newRisk};
                    }
                }
            }
        }


        // Flood-fill in all directions
        int dx, dy;
        for (dx = -1; dx <= 1; dx++) {
            for (dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;

                int nx = x + dx;
                int ny = y + dy;

                if (isValidCell(nx, ny)) {
                    float distance = sqrtf(dx * dx + dy * dy);
                    float newRisk = risk * riskPropogationFactor;

                    if (newRisk > maxRisk)
                        maxRisk = newRisk;

                    if (newRisk > riskGrid[nx][ny]) {
                        riskGrid[nx][ny] = newRisk;
                        if (rear < MAX_GRID * MAX_GRID) {
                            queue[rear++] = (QueueNode){nx, ny, newRisk};
                        }
                    }
                }
            }
        }
    }
}

void findSafeCell (Enemy *enemy) {

    int x, y;
    float lowestRisk = 2.0f;

    for (x = 0; x < MAX_GRID; x++) {
        for (y = 0; y < MAX_GRID; y++) {

            if (isValidCell(x, y) && riskGrid[x][y] < lowestRisk) {
                if (solveAStar(enemy->x, enemy->y, x, y)) {

                    lowestRisk = riskGrid[x][y];
                }
            }
        }
    }
}


int main (int argc, char **argv) {

    SetTargetFPS(60);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "Project Teardrop");

    int i, j;

    // Add walls all around the world
    for (i = 0; i < MAX_GRID; i++) {

        worldGrid[0][i] = CELL_WALL;
        worldGrid[i][0] = CELL_WALL;
        worldGrid[MAX_GRID - 1][i] = CELL_WALL;
        worldGrid[i][MAX_GRID - 1] = CELL_WALL;
    }

    while (!WindowShouldClose()) {

        float deltaTime = GetFrameTime();
        static float moveAccumulator = 0.0f;

        cellSize += GetMouseWheelMove();
        if (IsKeyPressed(KEY_TAB)) {

            editMode = !editMode;
            memset(riskGrid, 0, sizeof(riskGrid)); // Clear risk grid
        }


        int mouseX = GetMouseX();
        int mouseY = GetMouseY();
        if (!editMode) {

            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {

                int cellX = mouseX / cellSize;
                int cellY = mouseY / cellSize;
                if (IsKeyDown(KEY_LEFT_SHIFT)) {

                    if (mouseX < cellSize * MAX_GRID && mouseY < cellSize * MAX_GRID) {

                        solveAStar(enemy1.x, enemy1.y, cellX, cellY);
                    }
                }

                else {

                    if (mouseX < cellSize * MAX_GRID && mouseY < cellSize * MAX_GRID) {

                        shootGun(cellX, cellY, enemy1.x, enemy1.y);
                        if (riskGrid[enemy1.x][enemy1.y] > 0.2f) {

                            findSafeCell(&enemy1);
                        }
                    }
                }
            }
        }

        if (pathLength > 0) {

            moveAccumulator += moveSpeed * deltaTime;
            if (moveAccumulator >= 1.0f) {

                moveAccumulator -= 1.0f;
                PathFindingNode *nextNode = path[--pathLength];
                enemy1.x = nextNode->x;
                enemy1.y = nextNode->y;
            }
        }

        BeginDrawing();

        if (editMode)
            ClearBackground(backgroundColorEditing);
        else
            ClearBackground(backgroundColor);

        for (i = 0; i < MAX_GRID; i++) {

            for (j = 0; j < MAX_GRID; j++) {

                if (worldGrid[i][j] == CELL_WALL)
                    DrawRectangle(i * cellSize, j * cellSize, cellSize, cellSize, gridColor);
                else {

                    float riskFactor = riskGrid[i][j];
                    Color cellColor = backgroundColor;
                    if (riskFactor > 0.1) {

                        cellColor.r = riskFactor * maxRisk * 255;
                    }

                    DrawRectangle(i * cellSize, j * cellSize, cellSize, cellSize, cellColor);
                    DrawRectangleLines(i * cellSize, j * cellSize, cellSize, cellSize, gridColor);

                    if (cellSize > 30)
                        DrawText(TextFormat("%.2f", riskFactor), 2 + cellSize * i, 2 + cellSize * j, cellSize / 3, gridColor);
                }
            }
        }

        if (editMode) {

            if (mouseX < cellSize * MAX_GRID && mouseY < cellSize * MAX_GRID) {

                int cellX = mouseX / cellSize;
                int cellY = mouseY / cellSize;

                if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {

                    worldGrid[cellX][cellY] = CELL_WALL;
                }

                if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {

                    worldGrid[cellX][cellY] = CELL_EMPTY;
                }

                DrawRectangle(cellX * cellSize, cellY * cellSize, cellSize, cellSize, cellHoverColor);
            }
        }

        // Draw the enemy
        DrawRectangle(enemy1.x * cellSize, enemy1.y * cellSize, cellSize, cellSize, enemyColor);

        if (editMode) {

            DrawText("Edit Mode", (MAX_GRID + 1) * cellSize, 20, 20, LIGHTGRAY);
            DrawText(" Zoom - Scroll wheel",            (MAX_GRID + 1) * cellSize, 50, 20, GRAY);
            DrawText(" Toggle mode - Tab",              (MAX_GRID + 1) * cellSize, 70, 20, GRAY);
            DrawText(" Add wall - Mouse 1",             (MAX_GRID + 1) * cellSize, 90, 20, GRAY);
            DrawText(" Remove wall - Mouse 2",          (MAX_GRID + 1) * cellSize, 110, 20, GRAY);
        }
        else {

            DrawText("Play Mode",                       (MAX_GRID + 1) * cellSize, 20, 20, LIGHTGRAY);
            DrawText(" Zoom - Scroll wheel",            (MAX_GRID + 1) * cellSize, 50, 20, GRAY);
            DrawText(" Toggle mode - Tab",              (MAX_GRID + 1) * cellSize, 70, 20, GRAY);
            DrawText(" Shoot gun - Mouse 1",            (MAX_GRID + 1) * cellSize, 90, 20, GRAY);
            DrawText(" Move enemy - Shift + Mouse 1",   (MAX_GRID + 1) * cellSize, 110, 20, GRAY);
        }

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
