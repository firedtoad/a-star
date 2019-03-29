/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP__
#define __ASTAR_HPP__

#include <vector>
#include <functional>
#include <unordered_set>
#include <unordered_map>
namespace AStar
{
    struct Vec2i
    {
        int x, y;

        bool operator == (const Vec2i& coordinates_) const;
    };

    template<class T>
    class NodeAlloc
    {
    public:
        template<typename... _Args>
        T* construct(_Args&&... __args)
        {
            auto p=alloc.allocate(1);
            alloc.construct(p,std::forward<_Args>(__args)...);
            return p;
        }
        void destroy(T* p)
        {
            alloc.destroy(p);
            alloc.deallocate(p,sizeof(T));
        }
    private:
        std::allocator<T> alloc;
    };

    using uint = unsigned int;
//    using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
    using HeuristicFunction = uint (*)(const Vec2i&, const Vec2i&);
//    using CollisionFunction = bool (*)(const Vec2i&);
    using CollisionFunction = std::function<bool(const Vec2i&)>;
    using CoordinateList = std::vector<Vec2i>;

    struct Node
    {
        uint G, H;
        Vec2i coordinates;
        Node *parent;

        Node(const Vec2i &coord_, Node *parent_ = nullptr);
        uint getScore() const;
    };

    struct CoordHash
    {
        size_t operator ()(const Vec2i &coord) const
        {
            return coord.x*100000+coord.y;
        }
    };

    auto comp=[](Node *pNode1,Node *pNode2){
        return pNode1->getScore()>pNode2->getScore();
    };

//    using NodeSet = std::set<Node*>;
    using NodeSet = std::unordered_set<Node*>;
//    using NodeHeap=std::priority_queue<Node*,std::vector<Node*>,decltype(comp)>;
    using NodeHeap=std::vector<Node*>;
    using CoordMap = std::unordered_map<Vec2i,Node*,CoordHash>;
    using Alloc=NodeAlloc<Node>;
//    using Alloc=boost::object_pool<Node>;

    class Generator
    {
        bool detectCollision(const Vec2i &coordinates_);
        Node* findNodeOnList(const NodeSet& nodes_,const Vec2i &coordinates_);
        Node* findNodeOnMap(const CoordMap& nodes_,const Vec2i &coordinates_);
        void releaseNodes(NodeSet& nodes_);
        void releaseNodes(CoordMap& nodes_);
        void releaseNodes(NodeHeap& nodes_);

    public:
        Generator();
        void setWorldSize(const Vec2i &worldSize_);
        void setDiagonalMovement(bool enable_);
        void setHeuristic(HeuristicFunction heuristic_);
        template<typename F>
        void SetCollision(F &&collison_)
        {
            collision=collison_;
        }
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
        Alloc alloc;
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

#endif // __ASTAR_HPP__
