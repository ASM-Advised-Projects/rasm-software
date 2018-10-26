/**
 * Unit tests for the rasm2/util/circular_array.hpp file.
 */

#ifndef TEST_RASM2_UTIL_CIRCULAR_ARRAY_HPP
#define TEST_RASM2_UTIL_CIRCULAR_ARRAY_HPP

#include <cppunit/extensions/HelperMacros.h>
#include "rasm2/util/circular_array.hpp"

class CircularArrayTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(CircularArrayTests);
  CPPUNIT_TEST(testCopyConstructor);
  CPPUNIT_TEST(testPushGet);
  CPPUNIT_TEST(testBracketsOperator);
  CPPUNIT_TEST(testModify);
  CPPUNIT_TEST(testSize);
  CPPUNIT_TEST(testCapacity);
  CPPUNIT_TEST(testEqualsComparison);
  CPPUNIT_TEST(testEqualsAssignment);
  CPPUNIT_TEST(testZeroLength);
  CPPUNIT_TEST_SUITE_END();

  CircularArray<int> *caI1;
  CircularArray<int> *caI2;
  CircularArray<double> *caD1;

public:
  void setUp()
  {
    caI1 = new CircularArray<int>(5);
    caI2 = new CircularArray<int>(10);
    caD1 = new CircularArray<double>(5);
  }

  void tearDown()
  {
    delete caI1;
    delete caI2;
    delete caD1;
  }

  void testZeroLength()
  {
    caI1 = new CircularArray<int>(0);
    CPPUNIT_ASSERT(caI1->get(0) == 0);
    caI1->push(1);
    CPPUNIT_ASSERT(caI1->get(0) == 0);
  }

  void testCopyConstructor()
  {
    caI2->push(1);
    caI2->push(1);
    caI2 = new CircularArray<int>(*caI1);
    CPPUNIT_ASSERT(caI2->size() == 0);
    CPPUNIT_ASSERT(caI2->capacity() == 5);

    caI2->push(-1);
    caI2->push(1);
    caI1 = new CircularArray<int>(*caI2);
    CPPUNIT_ASSERT(caI1->size() == 2);
    CPPUNIT_ASSERT(caI1->capacity() == 5);
    CPPUNIT_ASSERT(caI1->get(0) == 1);
    CPPUNIT_ASSERT(caI1->get(1) == -1);
  }

  void testPushGet()
  {
    CPPUNIT_ASSERT(caD1->get(0) == 0.0);
    CPPUNIT_ASSERT(caD1->get(5) == 0.0);
    CPPUNIT_ASSERT(caD1->get(10) == 0.0);

    caD1->push(1.0);
    caD1->push(1.1);
    caD1->push(1.2);
    caD1->push(1.3);
    CPPUNIT_ASSERT(caD1->get(0) == 1.3);
    CPPUNIT_ASSERT(caD1->get(1) == 1.2);
    CPPUNIT_ASSERT(caD1->get(2) == 1.1);
    CPPUNIT_ASSERT(caD1->get(3) == 1.0);
    CPPUNIT_ASSERT(caD1->get(4) == 0.0);
    CPPUNIT_ASSERT(caD1->get(5) == 0.0);
    CPPUNIT_ASSERT(caD1->get(6) == 0.0);

    caD1->push(1.4);
    caD1->push(1.5);
    caD1->push(1.6);
    caD1->push(1.7);
    CPPUNIT_ASSERT(caD1->get(0) == 1.7);
    CPPUNIT_ASSERT(caD1->get(1) == 1.6);
    CPPUNIT_ASSERT(caD1->get(2) == 1.5);
    CPPUNIT_ASSERT(caD1->get(3) == 1.4);
    CPPUNIT_ASSERT(caD1->get(4) == 1.3);
    CPPUNIT_ASSERT(caD1->get(5) == 0.0);
    CPPUNIT_ASSERT(caD1->get(6) == 0.0);
  }

  void testBracketsOperator()
  {

    CPPUNIT_ASSERT((*caD1)[0] == 0.0);
    CPPUNIT_ASSERT((*caD1)[5] == 0.0);
    CPPUNIT_ASSERT((*caD1)[10] == 0.0);

    caD1->push(1.0);
    caD1->push(1.1);
    caD1->push(1.2);
    caD1->push(1.3);
    CPPUNIT_ASSERT((*caD1)[0] == 1.3);
    CPPUNIT_ASSERT((*caD1)[1] == 1.2);
    CPPUNIT_ASSERT((*caD1)[2] == 1.1);
    CPPUNIT_ASSERT((*caD1)[3] == 1.0);
    CPPUNIT_ASSERT((*caD1)[4] == 0.0);
    CPPUNIT_ASSERT((*caD1)[5] == 0.0);
    CPPUNIT_ASSERT((*caD1)[6] == 0.0);

    caD1->push(1.4);
    caD1->push(1.5);
    caD1->push(1.6);
    caD1->push(1.7);
    CPPUNIT_ASSERT((*caD1)[0] == 1.7);
    CPPUNIT_ASSERT((*caD1)[1] == 1.6);
    CPPUNIT_ASSERT((*caD1)[2] == 1.5);
    CPPUNIT_ASSERT((*caD1)[3] == 1.4);
    CPPUNIT_ASSERT((*caD1)[4] == 1.3);
    CPPUNIT_ASSERT((*caD1)[5] == 0.0);
    CPPUNIT_ASSERT((*caD1)[6] == 0.0);
  }

  void testModify()
  {
    caI1->modify(0, 2);
    caI1->modify(1, 2);
    CPPUNIT_ASSERT(caI1->get(0) == 0);
    CPPUNIT_ASSERT(caI1->get(1) == 0);
    CPPUNIT_ASSERT(caI1->size() == 0);

    caI1->push(1);
    caI1->push(2);
    caI1->push(3);
    caI1->modify(0, -3);
    CPPUNIT_ASSERT(caI1->get(0) == -3);
    CPPUNIT_ASSERT(caI1->get(1) == 2);
    CPPUNIT_ASSERT(caI1->get(2) == 1);
    CPPUNIT_ASSERT(caI1->size() == 3);

    caI1->modify(2, -1);
    CPPUNIT_ASSERT(caI1->get(0) == -3);
    CPPUNIT_ASSERT(caI1->get(1) == 2);
    CPPUNIT_ASSERT(caI1->get(2) == -1);
    CPPUNIT_ASSERT(caI1->size() == 3);
  }

  void testSize()
  {
    CPPUNIT_ASSERT(caI1->size() == 0);

    caI1->push(0);
    CPPUNIT_ASSERT(caI1->size() == 1);

    caI1->modify(0, 1);
    CPPUNIT_ASSERT(caI1->size() == 1);

    for (int i = 0; i < 10; ++i)
      caI1->push(0);
    CPPUNIT_ASSERT(caI1->size() == 5);

    (*caI1)[0];
    CPPUNIT_ASSERT(caI1->size() == 5);
  }

  void testCapacity()
  {
    CPPUNIT_ASSERT(caI1->capacity() == 5);

    caI1->push(0);
    CPPUNIT_ASSERT(caI1->capacity() == 5);

    caI1->modify(0, 1);
    CPPUNIT_ASSERT(caI1->capacity() == 5);

    for (int i = 0; i < 10; ++i)
      caI1->push(0);
    CPPUNIT_ASSERT(caI1->capacity() == 5);

    (*caI1)[0];
    CPPUNIT_ASSERT(caI1->capacity() == 5);
  }

  void testEqualsAssignment()
  {
    caI2->push(1);
    caI2->push(1);
    *caI2 = *caI1;
    CPPUNIT_ASSERT(caI2->size() == 0);
    CPPUNIT_ASSERT(caI2->capacity() == 5);

    caI2->push(-1);
    caI2->push(1);
    *caI1 = *caI2;
    CPPUNIT_ASSERT(caI1->size() == 2);
    CPPUNIT_ASSERT(caI1->capacity() == 5);
    CPPUNIT_ASSERT(caI2->get(0) == 1);
    CPPUNIT_ASSERT(caI2->get(1) == -1);
  }

  void testEqualsComparison()
  {
    CircularArray<int> *caI11 = new CircularArray<int>(caI1->capacity());

    // same size, capacity, and contents
    CPPUNIT_ASSERT(*caI1 == *caI11);
    caI1->push(1);
    caI1->push(2);
    caI11->push(1);
    caI11->push(2);
    CPPUNIT_ASSERT(*caI1 == *caI11);

    // same size and capacity; different contents
    caI11->modify(0, 3);
    CPPUNIT_ASSERT(*caI1 != *caI11);

    // same size and contents; different capacity
    caI2->push(1);
    caI2->push(2);
    CPPUNIT_ASSERT(*caI1 != *caI2);

    // same capacity and contents; different size
    caI11->push(0);
    caI11->push(0);
    caI11->push(0);
    caI11->push(1);
    caI11->push(2);
    CPPUNIT_ASSERT(*caI1 != *caI11);
  }
};

#endif
