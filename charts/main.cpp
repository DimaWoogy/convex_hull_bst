#include <algorithm>
#include <chrono>
#include <fstream>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_akl_toussaint.h>
#include <CGAL/ch_graham_andrew.h>

#include "bsthull.h"
#include "graham.h"
#include "chan.h"

using Point_2 = CGAL::Exact_predicates_inexact_constructions_kernel::Point_2;

template<class TFn, class ...TArgs>
double measureAlgo(TFn fn, size_t testsNum, TArgs&&... args)
{
   using namespace std::chrono;

   const auto tStart = high_resolution_clock::now();

   for (size_t test = 0; test < testsNum; ++test)
      (void) fn(std::forward<TArgs>(args)...);

   const auto tEnd = high_resolution_clock::now();

   return duration_cast<duration<double>>(tEnd - tStart).count() / testsNum;
}

int main()
{
   std::ifstream fin("in.txt");
   std::ofstream fout("out.txt");

   std::ofstream("algoNames.txt") << "new algo\ngraham\nakl_toussaint";

   size_t testsNum;
   fin >> testsNum;
   for (size_t test = 0; test < testsNum; ++test)
   {
      int pointsNum;
      fin >> pointsNum;

      std::vector<Point_2> points;
      points.reserve(pointsNum);
      for (int i = 0; i < pointsNum; ++i)
      {
         double x, y;
         fin >> x >> y;
         points.emplace_back(x, y);
      }

      using iterator = std::vector<Point_2>::iterator;
      std::vector<Point_2> result(pointsNum);

      const size_t runsNum = std::max(10, 100'000 / pointsNum);

      const auto createBstHull = [&]()
         {
            return algorithms::CreateBstHull<iterator>(points.begin(), points.end());
         };

      fout << measureAlgo(createBstHull, runsNum) << ' ';

      fout << measureAlgo(
            CGAL::ch_graham_andrew<iterator, iterator>,
               runsNum, points.begin(), points.end(), result.begin()) << ' ';

      fout << measureAlgo(
            CGAL::ch_akl_toussaint<iterator, iterator>,
               runsNum, points.begin(), points.end(), result.begin()) << '\n';
   }

   return 0;
}
