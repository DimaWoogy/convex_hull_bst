#include <cassert>
#include <random>
#include <vector>
#include <array>

namespace utils
{

template<class TValue, class TLess>
class CyclicSet
{
   static const int endIdx = -1;
   static const int maxLevelsNum = 30;
   struct Node
   {
      std::array<int, maxLevelsNum> next;
      int level;
      int cyclenext;
      int cycleprev;
      TValue value;
   };
public:
   struct iterator
   {
      iterator(int index, std::vector<Node>& nodes)
         : index(index)
         , nodes(nodes)
      {}

      iterator& operator++() noexcept { index = nodes[index].cyclenext; return *this; }
      iterator& operator--() noexcept { index = nodes[index].cycleprev; return *this; }
      bool operator==(const iterator& it) const noexcept { return it.index == index; }
      bool operator!=(const iterator& it) const noexcept { return !(it == *this); }
      TValue& operator*() noexcept { return nodes[index].value; }
      const TValue& operator*() const noexcept { return nodes[index].value; }

      int index;
      std::vector<Node>& nodes;
   };

   struct noncyclic_iterator
   {
      typedef std::forward_iterator_tag iterator_category;

      noncyclic_iterator(int index, const std::vector<Node>* nodes)
         : index(index)
         , nodes(nodes)
      {}
      noncyclic_iterator& operator++() noexcept { index = (*nodes)[index].next[0]; return *this; }
      bool operator==(const noncyclic_iterator& it) const noexcept { return it.index == index; }
      bool operator!=(const noncyclic_iterator& it) const noexcept { return !(it == *this); }
      const TValue& operator*() const noexcept { return (*nodes)[index].value; }

      int index;
      const std::vector<Node>* nodes;
   };

   noncyclic_iterator begin() const noexcept { return{ m_begin, &m_nodes }; }
   noncyclic_iterator end() const noexcept { return{ -1, &m_nodes }; }

   bool empty() const noexcept { return m_begin == -1; }
   int size() const noexcept { return m_size; }

   iterator upper_bound(const TValue& value) noexcept
   {
      if (empty())
         return{ -1, m_nodes };

      if (m_less(value, m_nodes[m_begin].value))
         return{ m_begin, m_nodes };

      int curIdx = m_begin;
      for (int i = maxLevelsNum - 1; i >= 0; --i)
      {
         while (m_nodes[curIdx].next[i] != endIdx)
         {
            if (m_less(value, m_nodes[m_nodes[curIdx].next[i]].value)) break;
            curIdx = m_nodes[curIdx].next[i];
         }
      }
      return{ m_nodes[curIdx].cyclenext, m_nodes };
   }

   void erase(const iterator beg, const iterator end) noexcept
   {
      if (beg == end)
         return;
      auto prevbeg = beg;
      --prevbeg;

      auto it = beg;
      bool isBeginInside = false;
      while (it != end)
      {
         if (m_begin == it.index && it != beg)
            isBeginInside = true;
         ++it;
         deleteNode(m_nodes[it.index].cycleprev);
      }

      auto itEnd = end;
      --itEnd;
      if (isBeginInside)
      {
         iterator div{ m_begin, m_nodes };
         updateNext(div, itEnd);
         --div;
         updateNext(beg, div);
      }
      else
      {
         updateNext(beg, itEnd);
      }

      m_nodes[prevbeg.index].cyclenext = end.index;
      m_nodes[end.index].cycleprev = prevbeg.index;
   }

   void insert(TValue value)
   {
      static std::mt19937 engine{ std::random_device{}() };
      static std::uniform_int_distribution<int> dist{ 1, maxLevelsNum };
      const int newIndex = getNew();
      m_nodes[newIndex].value = value;

      if (m_begin == -1)
      {
         m_begin = newIndex;
         m_nodes[newIndex].level = maxLevelsNum;
         m_nodes[newIndex].cyclenext = newIndex;
         m_nodes[newIndex].cycleprev = newIndex;
         std::fill(m_nodes[newIndex].next.begin(), m_nodes[newIndex].next.end(), -1);
         return;
      }
      else if (m_less(value, m_nodes[m_begin].value))
      {
         std::swap(m_nodes[newIndex].value, m_nodes[m_begin].value);
         value = m_nodes[m_begin].value;
      }

      const int level = dist(engine);
      m_nodes[newIndex].level = level;

      int curIdx = m_begin;
      for (int i = maxLevelsNum - 1; i >= 0; --i)
      {
         while (m_nodes[curIdx].next[i] != endIdx)
         {
            if (m_less(value, m_nodes[m_nodes[curIdx].next[i]].value)) break;
            if (value == m_nodes[m_nodes[curIdx].next[i]].value)
               return;
            curIdx = m_nodes[curIdx].next[i];
         }
         if (i < level)
         {
            m_nodes[newIndex].next[i] = m_nodes[curIdx].next[i];
            m_nodes[curIdx].next[i] = newIndex;
         }
      }
      int n = m_nodes[newIndex].next[0];
      if (n == -1)
         n = m_begin;

      m_nodes[n].cycleprev = newIndex;
      m_nodes[newIndex].cyclenext = n;


      m_nodes[curIdx].cyclenext = newIndex;
      m_nodes[newIndex].cycleprev = curIdx;
   }

private:
   void updateNext(iterator beg, iterator end) noexcept
   {
      const auto& rightValue = *end;

      if (beg.index == m_begin)
      {
         int index = m_nodes[end.index].next[0];
         m_nodes[index].level = maxLevelsNum;

         int cur = m_begin;
         for (int i = maxLevelsNum - 1; i >= 0; --i)
         {
            for (; m_nodes[cur].next[i] != endIdx;
               cur = m_nodes[cur].next[i])
            {
               if (m_less(rightValue, m_nodes[m_nodes[cur].next[i]].value)) break;
            }
            if (index == m_nodes[cur].next[i])
               break;
            m_nodes[index].next[i] = m_nodes[cur].next[i];
         }
         m_begin = index;
         return;
      }

      const auto& leftValue = *beg;
      int leftIdx = m_begin;
      int rightIdx = m_begin;
      for (int i = maxLevelsNum - 1; i >= 0; --i)
      {
         for (; m_nodes[leftIdx].next[i] != endIdx;
            leftIdx = m_nodes[leftIdx].next[i])
         {
            if (!m_less(m_nodes[m_nodes[leftIdx].next[i]].value, leftValue)) break;
         }
         for (; m_nodes[rightIdx].next[i] != endIdx;
            rightIdx = m_nodes[rightIdx].next[i])
         {
            if (m_less(rightValue, m_nodes[m_nodes[rightIdx].next[i]].value)) break;
         }
         m_nodes[leftIdx].next[i] = m_nodes[rightIdx].next[i];
      }
   }
   int getNew()
   {
      ++m_size;
      if (m_deletedBegin == -1)
      {
         m_nodes.emplace_back();
         return (int)m_nodes.size() - 1;
      }
      const int res = m_deletedBegin;
      m_deletedBegin = m_nodes[m_deletedBegin].cyclenext;
      return res;
   }

   void deleteNode(int index) noexcept
   {
      --m_size;
      m_nodes[index].cyclenext = m_deletedBegin;
      m_nodes[index].level = -1;
      m_deletedBegin = index;
   }

private:
   int m_deletedBegin = -1;
   int m_size = 0;
   int m_begin = -1;
   std::vector<Node> m_nodes;
   TLess m_less;
};


}