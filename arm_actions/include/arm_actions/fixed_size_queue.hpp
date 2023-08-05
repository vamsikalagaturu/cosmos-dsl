#ifndef FIXED_SIZE_QUEUE_HPP
#define FIXED_SIZE_QUEUE_HPP

#include <cstddef>
#include <queue>

/**
 * @brief A class representing a fixed size queue
 * @tparam T the type of the queue
 * @tparam N the size of the queue
 */
template <typename T, size_t N>
class FixedSizeQueue : public std::queue<T>
{
public:
  /**
   * @brief Construct a new Fixed Size Queue object
   */
  FixedSizeQueue() : std::queue<T>() {}

  /**
   * @brief push a value to the queue and pop the oldest value if the queue is full
   * @param value the value to push
   */
  void push(const T& value)
  {
    if (this->size() == N)
    {
      this->pop();
    }

    std::queue<T>::push(value);
  }

  /**
   * @brief method to check if the queue is full
   * @return true if the queue is full
   */
  bool isFull() const
  {
    return this->size() == N;
  }

  /**
   * @brief method to get the value at a specific index
   * @param index the index of the value to get
   * @return the value at the index
   */
  T at(size_t index) const
  {
    return this->c.at(index);
  }

private:
  size_t maxSize;
};

#endif  // FIXED_SIZE_QUEUE_HPP
