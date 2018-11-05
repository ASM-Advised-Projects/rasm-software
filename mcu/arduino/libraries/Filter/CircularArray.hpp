/**
 * Defines the CircularArray<T> class.
 */

#ifndef FILTER_CIRCULAR_ARRAY_HPP
#define FILTER_CIRCULAR_ARRAY_HPP

/**
 * This class represents a ring buffer of elements. It's basically a fixed-size
 * array that will remove the oldest values in order to add new values. Its
 * elements are indexed according to how recently they were added.
 */
template <typename T>
class CircularArray {
private:
  T *array;
  unsigned int next_index;
  unsigned int max_size_;
  unsigned int size_;

public:
  /**
   * Constructs an empty circular array with the given fixed size as its
   * capacity.
   */
  CircularArray(unsigned int max_size)
  : array(new T[max_size])
  , next_index(0)
  , max_size_(max_size)
  , size_(0)
  {
  }

  /**
   * Copy constructor.
   */
  CircularArray(const CircularArray<T> &other)
  {
    next_index = other.next_index;
    max_size_ = other.max_size_;
    size_ = other.size_;

    array = new T[other.max_size_];
    for (int i = 0; i < size_; i++)
      array[i] = other.array[i];
  }

  /**
   * Destructs this circular array.
   */
  ~CircularArray()
  {
    delete[] array;
  }

  /**
   * Pushes a new value onto the front of this array. This will delete the
   * oldest value if this array is full.
   */
  void push(T val)
  {
    if (max_size_ == 0)
      return;

    array[next_index] = val;
    next_index = (next_index + 1) % max_size_;
    if (size_ < max_size_)
      ++size_;
  }

  /**
   * Returns the value at the given index, which is a number of entries back.
   * If the entries back is greater than or equal to this array's size then 0
   * is returned.
   */
  T get(unsigned int entries_back) const
  {
    if (entries_back >= size_)
      return 0;
    return array[((next_index-1) - entries_back + max_size_) % max_size_];
  }

  /**
   * The equivalent of calling the get(unsigned int) method.
   */
  T operator[](unsigned int entries_back) const
  {
    return get(entries_back);
  }

  /**
   * Changes the value held at the given index (entries_back) to val. If the
   * given index is greater than size()-1 then this method does nothing.
   */
  void modify(unsigned int entries_back, T val)
  {
    if (entries_back < size_)
      array[(next_index-1-entries_back) % max_size_] = val;
  }

  /**
   * Returns the current amount of values in this array.
   */
  unsigned int size() const
  {
    return size_;
  }

  /**
   * Returns that max amount of values this array can hold.
   */
  unsigned int capacity() const
  {
    return max_size_;
  }

  /**
   * Equals assignment operator.
   */
  CircularArray & operator=(const CircularArray<T> &other)
  {
    if (*this == other)
      return *this;

    next_index = other.next_index;
    max_size_ = other.max_size_;
    size_ = other.size_;

    array = new T[other.max_size_];
    for (int i = 0; i < size_; i++)
      array[i] = other.array[i];

    return *this;
  }

  /**
   * Equals comparison operator.
   */
  bool operator==(const CircularArray<T> &other) const
  {
    for (int i = 0; i < size_; i++)
    {
      if (get(i) != other.get(i))
        return false;
    }

    return size_ == other.size_ && max_size_ == other.max_size_;
  }

  /**
   * Not-equals comparison operator.
   */
  bool operator!=(const CircularArray<T> &other) const
  {
    return !operator==(other);
  }
};

#endif
