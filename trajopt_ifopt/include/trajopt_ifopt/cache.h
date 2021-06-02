#ifndef TRAJOPT_IFOPT_CACHE_H
#define TRAJOPT_IFOPT_CACHE_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <memory>
#include <algorithm>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
template <typename KeyT, class ValueT, std::size_t bufsize = 666>
class Cache
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cache() : keybuf_(bufsize), valbuf_(bufsize) {}

  void put(const KeyT& key, const ValueT& value)
  {
    keybuf_[m_] = key;
    valbuf_[m_] = value;
    ++m_;
    if (static_cast<unsigned>(m_) == bufsize)
      m_ = 0;
  }

  ValueT* get(const KeyT& key)
  {
    auto it = std::find(keybuf_.begin(), keybuf_.end(), key);
    return (it == keybuf_.end()) ? nullptr : &valbuf_[std::distance(keybuf_.begin(), it)];
  }

private:
  int m_{ 0 };
  std::vector<KeyT> keybuf_;    // circular buffer
  std::vector<ValueT> valbuf_;  // circular buffer
};

}  // namespace trajopt
#endif  // TRAJOPT_IFOPT_CACHE_H
