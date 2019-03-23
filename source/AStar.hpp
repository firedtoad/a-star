/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP__
#define __ASTAR_HPP__

#include <vector>
#include <functional>
#include <set>


namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_);
    };
    class Heuristic;
    using uint = unsigned int;
//    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using HeuristicFunction = uint (*)(const Vec2i&, const Vec2i&);
    using CollisionFunction = bool (*)(const Vec2i&);
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(const Vec2i &coord_, Node *parent_ = nullptr);
        uint getScore();
    };

    using NodeSet = std::set<Node*>;

    class Generator
    {
        bool detectCollision(const Vec2i &coordinates_);
        Node* findNodeOnList(const NodeSet& nodes_,const Vec2i &coordinates_);
        void releaseNodes(NodeSet& nodes_);

    public:
        Generator();
        void setWorldSize(const Vec2i &worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        void SetCollision(CollisionFunction collision_);
        CoordinateList findPath(const Vec2i &source_,const Vec2i &target_);
        void addCollision(const Vec2i &coordinates_);
        void removeCollision(const Vec2i &coordinates_);
        void clearCollisions();

    private:
        HeuristicFunction heuristic;
        CollisionFunction collision;
        CoordinateList direction, walls;
        Vec2i worldSize;
        uint directions;
    };

    class Heuristic
    {
        static Vec2i getDelta(const Vec2i &source_,const Vec2i &target_);

    public:
        static uint manhattan(const Vec2i &source_,const Vec2i &target_);
        static uint euclidean(const Vec2i &source_,const Vec2i &target_);
        static uint octagonal(const Vec2i &source_,const Vec2i &target_);
    };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
