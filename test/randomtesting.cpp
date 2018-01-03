#include <vector>
#include <random>
#include <set>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_akl_toussaint.h>
#include <CGAL/ch_graham_andrew.h>

#include "bsthull.h"
#include "graham.h"
#include "chan.h"

std::vector<Point> getRandomPoints(std::mt19937& i_gen, size_t n)
{
   std::uniform_real_distribution<double> dist;
   std::vector<Point> result;
   result.reserve(n);
   for (size_t i = 0; i < n; ++i) result.push_back({ dist(i_gen), dist(i_gen) });
   return result;
}

template<class TContainer1, class TContainer2, class TPred>
bool areEqualHulls(const TContainer1& first, const TContainer2& second, TPred pred)
{
   if (first.size() != second.size()) return false;

   auto it = first.begin();
   while (it != first.end() && !pred(*it, *second.begin()))
      ++it;
   if (it == first.end())
      return false;

   for (const auto& p : second)
   {
      if (!pred(*it, p))
         return false;
      ++it;
      if (it == first.end()) it = first.begin();
   }
   return true;
}

using Point_2 = CGAL::Exact_predicates_inexact_constructions_kernel::Point_2;

int main()
{
   std::mt19937 gen(19);
   const size_t testsNum = 100;
   const size_t pointsNum = 1000;

   for (size_t test = 0; test < testsNum; ++test)
   {
      auto points = getRandomPoints(gen, pointsNum);

      auto resBst = algorithms::BstConvexHull::Create(points);
      auto resGraham = algorithms::GrahamScan(points);
      auto resChan = algorithms::Chan(points);

      std::vector<Point_2> points2;
      points2.reserve(pointsNum);
      for (const Point& point : points)
         points2.emplace_back(point.x, point.y);
      std::vector<Point_2> result;
      CGAL::ch_akl_toussaint(points2.begin(), points2.end(), std::back_inserter(result));

      if (!areEqualHulls(resBst.GetPoints(), resGraham,
         [&](const Point& left, const Point& right)
         {
            return left + resBst.GetCenter() == right;
         }))
      {
         std::cerr << "bst error\n" << test;
         return 1;
      }

      if (!areEqualHulls(resGraham, resChan,
         [](const Point& left, const Point& right)
         {
            return left == right;
         }))
      {
         std::cerr << "chan error\n" << test;
         return 2;
      }
      if (!areEqualHulls(resGraham, result,
         [](const Point& left, const Point_2& right)
         {
            return left.x == right.x() && left.y == right.y();
         }))
      {
         std::cerr << "akl error\n" << test;
         return 2;
      }
   }

   return 0;
}