#pragma once

#include <set>
#include <vector>

#include <boost/pool/pool_alloc.hpp>

#include "point.h"
#include "cyclicset.h"

namespace algorithms
{

struct BstConvexHull
{
   void AddPoint(const Point& pt)
   {
      const Point point = pt - center;
      auto next = convexHull.upper_bound(point);
      auto prev = next;
      --prev;

      if (ccw(*prev, point, *next))
      {
         while (true)
         {
            auto afterNext = next;
            ++afterNext;
            if (ccw(point, *next, *afterNext))
               break;
            ++next;
         }

         while (true)
         {
            auto beforePrev = prev;
            --beforePrev;
            if (ccw(*beforePrev, *prev, point))
               break;
            --prev;
         }
         ++prev;
         convexHull.erase(prev, next);
         convexHull.insert(point);
      }
   }

   BstConvexHull(Point center) : center(center) {}

   static BstConvexHull Create(const std::vector<Point>& points)
   {
     Point maxPoint = points[0];
     Point minPoint = points[1];
     if (maxPoint.x < minPoint.x)
        std::swap(minPoint, maxPoint);

     for (const Point& point : points)
     {
        if (point.x > maxPoint.x) maxPoint = point;
        if (point.x < minPoint.x) minPoint = point;
     }
     Point furthestPoint = maxPoint;
     double furthestDist = 0;


     const auto divideVect = maxPoint - minPoint;
     for (const Point& point : points)
     {
        const double dist = abs((point - minPoint) * divideVect);
        if (dist > furthestDist)
        {
           furthestPoint = point;
           furthestDist = dist;
        }
     }

     BstConvexHull convexHull{ (minPoint + maxPoint + furthestPoint) / 3 };
     convexHull.convexHull.insert(minPoint - convexHull.center);
     convexHull.convexHull.insert(maxPoint - convexHull.center);
     convexHull.convexHull.insert(furthestPoint - convexHull.center);

     for (const Point& point : points)
        convexHull.AddPoint(point);

     return convexHull;
   }

   struct less
   {
      bool operator()(const Point& left, const Point& right) const noexcept
      {
         const bool leftUp =  0 < left.y;
         const bool rightUp = 0 < right.y;
         if (leftUp != rightUp)
            return leftUp < rightUp;
         if (left.y == right.y && left.y == 0)
            return (0 < left.x) < (0 < right.x);

         return left * right > 0.0;
      }
   };

   const utils::CyclicSet<Point, less>& GetPoints() const noexcept { return convexHull; }
   const Point& GetCenter() const noexcept { return center; }

private:
   bool ccw(const Point& a, const Point& b, const Point& c) const noexcept
   {
      return (b.x*c.y - c.x*b.y) -
         (a.x*c.y - c.x * a.y) +
         (a.x*b.y - b.x*a.y) > 0;
   }

   const Point center;
   utils::CyclicSet<Point, less> convexHull;
};

}
