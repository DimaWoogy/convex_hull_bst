#pragma once

#include <set>
#include <vector>
#include <iterator>

#include <boost/pool/pool_alloc.hpp>

#include <CGAL/basic.h>
#include <CGAL/ch_selected_extreme_points_2.h>

namespace algorithms
{

// TODO: Add to traits
template<class Point>
Point sub(const Point& a, const Point& b)
{
   return{ a.x() - b.x(), a.y() - b.y() };
}

template<class Point, class Traits>
struct BstConvexHull
{
   void AddPoint(const Point& pt)
   {
      const auto point = sub(pt, center);
      auto next = getIt(convexHull.upper_bound(point));
      auto prev = prevIt(next);

      if (ccw(*prev, point, *next))
      {
         while (true)
         {
            const auto afterNext = nextIt(next);
            if (ccw(point, *next, *afterNext))
               break;
            convexHull.erase(next);
            next = afterNext;
         }

         while (true)
         {
            const auto beforePrev = prevIt(prev);
            if (ccw(*beforePrev, *prev, point))
               break;
            convexHull.erase(prev);
            prev = beforePrev;
         }
         convexHull.emplace_hint(prev, point);
      }
   }

   BstConvexHull(Point c, Traits traits)
      : center(std::move(c))
      , ccw(std::move(traits.less_rotate_ccw_2_object()))
   {}

   struct less
   {
      bool operator()(const Point& left, const Point& right) const noexcept
      {
         const bool leftUp =  0 < left.y();
         const bool rightUp = 0 < right.y();
         if (leftUp != rightUp)
            return leftUp < rightUp;
         if (left.y() == right.y() && left.y() == 0)
            return (0 < left.x()) < (0 < right.x());

         return left.x() * right.y() - left.y() * right.x() > 0.0;
      }
   };


   // TODO: I never free singleton memory
   using setallocator = boost::fast_pool_allocator<Point,
         boost::default_user_allocator_new_delete, boost::details::pool::default_mutex, 1048576>;

   const std::set<Point, less, setallocator>& GetPoints() const noexcept { return convexHull; }
   const Point& GetCenter() const noexcept { return this->center; }

//private: TODO
//TODO: refactor

   using TIt = typename std::set<Point, less>::iterator;
   TIt getIt(TIt it) const noexcept
   {
      return it != convexHull.end() ? it : convexHull.begin();
   }
   TIt nextIt(TIt it) const noexcept
   {
      return getIt(std::next(it));
   }
   TIt prevIt(TIt it) const noexcept
   {
      return it != convexHull.begin() ? std::prev(it) : std::prev(convexHull.end());
   }

   const Point center;
   const typename Traits::Less_rotate_ccw_2 ccw;
   typename std::set<Point, less, setallocator> convexHull;
};

template<class ForwardIterator, class Traits>
auto CreateBstHull(ForwardIterator first, ForwardIterator last, Traits&& traits)
{
   ForwardIterator n, s, w, e;
   CGAL::ch_nswe_point(first, last, n, s, w, e, traits);

   using TPoint = typename std::iterator_traits<ForwardIterator>::value_type;
   // TODO: Add to traits
   const TPoint center{
      (n->x() + s->x() + w->x() + e->x()) / 4 ,
      (n->y() + s->y() + w->y() + e->y()) / 4 };
   BstConvexHull<TPoint, Traits>  hull{ center, std::forward<Traits>(traits) };

   hull.convexHull.emplace(sub(*n, center));
   hull.convexHull.emplace(sub(*w, center));
   hull.convexHull.emplace(sub(*s, center));
   hull.convexHull.emplace(sub(*e, center));

   while (first != last)
   {
      hull.AddPoint(*first);
      ++first;
   }

   return hull;
}

template<class ForwardIterator>
auto CreateBstHull(ForwardIterator first, ForwardIterator last)
{
   using value_type = typename std::iterator_traits<ForwardIterator>::value_type;
   using Kernel = typename CGAL::Kernel_traits<value_type>::Kernel;
   return CreateBstHull(first, last, Kernel());
}

}

