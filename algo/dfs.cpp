#include<bits/stdc++.h>
using namespace std;


const int dr[] = {-1, 0, 1, 0};
const int dc[] = {0, 1, 0, 1};
const int R_CNT = 30;
const int C_CNT = 30;
const double LEN_X = 3.0;
const double LEN_Z = 3.0;
const double sqrH = LEN_X / R_CNT;
const double sqrW = LEN_Z / C_CNT;
const double xBase = -LEN_X / 2 + sqrH / 2;
const double zBase = -LEN_Z / 2 + sqrW / 2;

char maze[R_CNT][C_CNT];
bool mark[R_CNT][C_CNT];

vector<pair<int, int> > squares;

bool fin = false;


void dfs(int r, int c)
{
    mark[r][c] = true;
    squares.push_back({r, c});
    if(r == 0 && c == 29)
    {
        fin = true;
        return;
    }
    for(int i = 0; i < 4; ++i)
    {
        int nr = r + dr[i];
        int nc = c + dc[i];
        if(nr < 0 || nr >= R_CNT || nc < 0 || nc >= C_CNT)
        {
            continue;
        }
        if(mark[nr][nc] || maze[nr][nc] == '1')
        {
            continue;
        }
        dfs(nr, nc);
        if(fin)
        {
            return;
        }
        squares.pop_back();
    }
}



int main()
{
    FILE* fileIn = fopen("obstacles.txt", "r");
    FILE* fileOut = fopen("xzOut.txt", "w");
    for(int i = 0; i < R_CNT; ++i)
    {
        for(int j = 0; j < C_CNT; ++j)
        {
            fscanf(fileIn, " %c", &maze[i][j]);
        }
    }
    dfs(29, 0);
    if(fin)
    {
        for(int k = 0; k < squares.size(); ++k)
        {
            double xCoor = xBase + (R_CNT - 1 - squares[k].first) * sqrH;
            double zCoor = zBase + squares[k].second * sqrW;
            fprintf(fileOut, "%.6f %.6f\n", xCoor, zCoor);
        }
        {
            FILE* fs = fopen("dfsXRes.txt", "w");
            for(int k = 0; k < squares.size(); ++k)
            {
                maze[squares[k].first][squares[k].second] = 'x';
            }


            for(int i = 0; i < R_CNT; ++i)
            {
                for(int j = 0; j < C_CNT; ++j)
                {
                    fprintf(fs, "%c ", maze[i][j]);
                }
                fprintf(fs, "\n");
            }
        }
    }





}
