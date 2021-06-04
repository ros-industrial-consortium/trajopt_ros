#pragma once

#include <Eigen/Core>
#include <vector>
#include <algorithm>

template <typename KeyT, class ValueT>
class Cache
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Cache(std::size_t bufsize = 666) : bufsize_(bufsize), keybuf_(bufsize), valbuf_(bufsize) {}

  void put(const KeyT& key, const ValueT& value)
  {
    keybuf_[m_] = key;
    valbuf_[m_] = value;
    ++m_;
    if (static_cast<unsigned>(m_) == bufsize_)
      m_ = 0;
  }

  ValueT* get(const KeyT& key)
  {
    auto it = std::find(keybuf_.begin(), keybuf_.end(), key);
    return (it == keybuf_.end()) ? nullptr : &valbuf_[std::distance(keybuf_.begin(), it)];
  }

private:
  int m_{ 0 };
  std::size_t bufsize_;
  std::vector<KeyT> keybuf_;    // circular buffer
  std::vector<ValueT> valbuf_;  // circular buffer
};
