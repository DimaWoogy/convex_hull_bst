#include <cmath>
#include <vector>
#include <random>
#include <set>
#include <iostream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_akl_toussaint.h>
#include <CGAL/ch_graham_andrew.h>

#include "bsthull.h"

using Point_2 = CGAL::Exact_predicates_inexact_constructions_kernel::Point_2;

auto getRandomPoints(std::mt19937& i_gen, size_t n)
{
   std::vector<Point_2> result;
   result.reserve(n);

   std::uniform_real_distribution<double> dist;
   for (size_t i = 0; i < n; ++i)
      result.emplace_back(dist(i_gen), dist(i_gen));

   return result;
}

template<class TContainer1, class TContainer2, class TPred>
bool areEqualHulls(const TContainer1& first, const TContainer2& second, TPred pred)
{
   if (first.size() != second.size()) return false;

   auto it = std::find_if(first.begin(), first.end(),
      [&](const Point_2& point)
      {
         return pred(point, *second.begin());
      });
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


int main()
{
   std::mt19937 gen(19);
   const size_t testsNum = 100;
   const size_t pointsNum = 1000;

   for (size_t test = 0; test < testsNum; ++test)
   {
      auto points = getRandomPoints(gen, pointsNum);

      auto resBst = algorithms::CreateBstHull(points.begin(), points.end());

      std::vector<Point_2> resAkl;
      CGAL::ch_akl_toussaint(points.begin(), points.end(), std::back_inserter(resAkl));

      if (!areEqualHulls(resBst.GetPoints(), resAkl,
         [&](const Point_2& left, const Point_2& right)
         {
            return std::abs(left.x() + resBst.GetCenter().x() - right.x()) < 1e-7 &&
                   std::abs(left.y() + resBst.GetCenter().y() - right.y()) < 1e-7;
         }))
      {
         std::cerr << "test: " << test << '\n';
         for (const auto& point : resBst.GetPoints())
         {
            std::cerr << '(' << point.x() + resBst.GetCenter().x() << ','
                             << point.y() + resBst.GetCenter().y() << ')';
         }
         std::cerr << '\n';
         for (const auto& point : resAkl)
         {
            std::cerr << '(' << point.x() << ',' << point.y() << ')';
         }
         std::cerr << '\n';
         return 1;
      }
   }

   return 0;
}
