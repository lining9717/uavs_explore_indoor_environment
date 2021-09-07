#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

#include "configuration.h"

const int INF = 1e8 + 7;

//四联通
const std::vector<std::pair<int, int>> NEIGHTBORS = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}};

namespace trackingplanner
{
    struct SetElem
    {
        int id;
        double fvalue;
        SetElem(int id, double fv) : id(id), fvalue(fv) {}
        SetElem()
        {
            id = -1;
            fvalue = INF;
        }
        bool operator<(const SetElem &d) const
        {
            return (fvalue != d.fvalue) ? (fvalue < d.fvalue) : (id < d.id);
        }
    };

    class TrackingPlanner
    {
    private:
        //创建算法所需的集合
        std::set<SetElem> OPEN;
        std::set<SetElem> CLOSED;
        std::set<SetElem> INCONS;
        std::set<int> DELETED;

        //地图
        std::vector<std::vector<int>> graph;

        double eps_max = 1.8;
        double eps = eps_max;

        // 步长
        double step = 0.3;
        // 迷宫顶点编号，起始变量，上次访问
        int map_size, start, goal, last_start;

        int x_bias;
        int y_bias;

        //迷宫的大小。 输入后更改，m为列数，n为行数
        int width;
        int height;

        //文字图
        std::vector<std::string> lab;

        int path_len = INF;
        int old_goal = INF;

        //重定向的运行时间
        clock_t time_cur;
        //图顶点的v值和g值
        std::vector<int> rhs;
        std::vector<int> g;
        //还原路径的父向量
        std::vector<int> parents;
        //最后找到的路径
        std::vector<int> path;

        int getXFromCol(int col);
        int getYFromRow(int row);
        int getColFromCoordinate(int x);
        int getRowFromCoordinate(int y);

    public:
        TrackingPlanner();
        ~TrackingPlanner();

        //将迷宫中的坐标转换为顶点编号
        int getIndex(int x, int y);

        //获取曼哈顿距离
        int getHeruestic(int x, int y);

        //计算顶点优先级（迭代启发式）
        double getFvalue(int i);

        // 迭代
        bool improvePath();

        //更新所选SetElem fvalue的函数
        void updateFvalues(std::set<SetElem> &s);

        //在更改迭代epsilon的同时启动迭代的函数
        bool computePath();

        //删除
        void dfsDelete(int current);

        void step1();
        void step2();
        void step3();
        void step4();

        // 获取路径
        void getCurentPath();

        std::string getPlannerName();

        void init(const Node &nstart, const Node &ngoal, const std::string &map_file_path);

        std::pair<int, int> getNextPosition();
        void updateNextTarget(int x, int y);

        int sum(std::vector<int> &v);
    };
};