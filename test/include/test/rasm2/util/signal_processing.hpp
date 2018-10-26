/**
 * Unit tests for the rasm2/util/signal_processing.hpp file.
 */

#ifndef TEST_RASM2_UTIL_SIGNAL_PROCESSING_HPP
#define TEST_RASM2_UTIL_SIGNAL_PROCESSING_HPP

#include <cppunit/extensions/HelperMacros.h>
#include "rasm2/util/signal_processing.hpp"

class CausalLTIFilterTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(CausalLTIFilterTests);
  CPPUNIT_TEST(testEmptyCoeffs);
  CPPUNIT_TEST(testFeedforwardCoeffs);
  CPPUNIT_TEST(testFeedbackCoeffs);
  CPPUNIT_TEST(testCombinedCoeffs);
  CPPUNIT_TEST(testParenthesesOperator);
  CPPUNIT_TEST_SUITE_END();

  std::vector<double> ff_coeffs;
  std::vector<double> fb_coeffs;
  double prec = 0.000001;

public:
  void setUp()
  {
    ff_coeffs.clear();
    fb_coeffs.clear();
  }

  void tearDown()
  {
  }

  void testEmptyCoeffs()
  {
    CausalLTIFilter filter(ff_coeffs, fb_coeffs);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    filter.input(1.4);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    filter.input(-2.2);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
  }

  void testFeedforwardCoeffs()
  {
    ff_coeffs.push_back(3.0);
    ff_coeffs.push_back(2.0);
    ff_coeffs.push_back(1.0);
    CausalLTIFilter filter(ff_coeffs, fb_coeffs);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    filter.input(1.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3*1, filter.output(), prec);
    filter.input(2.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3*2 + 2*1, filter.output(), prec);
    filter.input(3.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3*3 + 2*2 + 1*1, filter.output(), prec);
    filter.input(4.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3*4 + 2*3 + 1*2, filter.output(), prec);
    filter.input(5.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3*5 + 2*4 + 1*3, filter.output(), prec);
  }

  void testFeedbackCoeffs()
  {
    ff_coeffs.push_back(1.0);
    fb_coeffs.push_back(0.5);
    fb_coeffs.push_back(0.25);
    fb_coeffs.push_back(0.25);
    CausalLTIFilter filter(ff_coeffs, fb_coeffs);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    filter.input(1.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1*1, filter.output(), prec);
    filter.input(2.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1*2 + 0.5*1, filter.output(), prec);
    filter.input(3.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1*3 + 0.5*2.5 + 0.25*1, filter.output(), prec);
    filter.input(4.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1*4 + 0.5*4.5 + 0.25*2.5 + 0.25*1, filter.output(), prec);
    filter.input(5.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1*5 + 0.5*7.125 + 0.25*4.5 + 0.25*2.5, filter.output(), prec);
  }

  void testCombinedCoeffs()
  {
    ff_coeffs.push_back(0.2);
    ff_coeffs.push_back(0.3);
    fb_coeffs.push_back(0.2);
    fb_coeffs.push_back(0.3);
    CausalLTIFilter filter(ff_coeffs, fb_coeffs);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    filter.input(5.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*5, filter.output(), prec);
    filter.input(4.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*4 + 0.3*5 + 0.2*1, filter.output(), prec);
    filter.input(3.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*3 + 0.3*4 + 0.2*2.5 + 0.3*1, filter.output(), prec);
    filter.input(2.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*2 + 0.3*3 + 0.2*2.6 + 0.3*2.5, filter.output(), prec);
    filter.input(1.0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*1 + 0.3*2 + 0.2*2.57 + 0.3*2.6, filter.output(), prec);
  }

  void testParenthesesOperator()
  {
    ff_coeffs.push_back(0.2);
    ff_coeffs.push_back(0.3);
    fb_coeffs.push_back(0.2);
    fb_coeffs.push_back(0.3);
    CausalLTIFilter filter(ff_coeffs, fb_coeffs);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, filter.output(), prec);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*5, filter(5), prec);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*4 + 0.3*5 + 0.2*1, filter(4), prec);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*3 + 0.3*4 + 0.2*2.5 + 0.3*1, filter(3), prec);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*2 + 0.3*3 + 0.2*2.6 + 0.3*2.5, filter(2), prec);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.2*1 + 0.3*2 + 0.2*2.57 + 0.3*2.6, filter(1), prec);
  }
};

class DifferentiatorTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(DifferentiatorTests);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals1, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals2, std::invalid_argument);
  CPPUNIT_TEST(testUniformIntegerDomain);
  CPPUNIT_TEST(testUniformNonIntegerDomain);
  CPPUNIT_TEST(testNonUniformDomain);
  CPPUNIT_TEST_SUITE_END();

  RealTimeDifferentiator *diff;
  double prec = 0.000001;

public:
  void setUp()
  {
    diff = new RealTimeDifferentiator();
  }

  void tearDown()
  {
    delete diff;
  }

  void testSameIndVals1()
  {
    diff->input(0, 1);
    diff->input(0, 2);
  }

  void testSameIndVals2()
  {
    diff->input(0.1, 1);
    diff->input(0.2, 2);
    diff->input(0.3, 3);
    diff->input(0.3, 4);
  }

  void testUniformIntegerDomain()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(0, 1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(1, 2);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1, diff->derivative(), prec);
    diff->input(2, 1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(-2, diff->derivative(), prec);
    diff->input(3, 0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(-1, diff->derivative(), prec);
    diff->input(4, 0);
    diff->input(5, 0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
  }

  void testUniformNonIntegerDomain()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(0.0, 1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(0.1, 2);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(10, diff->derivative(), prec);
    diff->input(0.2, 1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(-20, diff->derivative(), prec);
    diff->input(0.3, 0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(-10, diff->derivative(), prec);
    diff->input(0.4, 0);
    diff->input(0.5, 0);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
  }

  void testNonUniformDomain()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(0.0, 1.1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, diff->derivative(), prec);
    diff->input(0.1, 2.1);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(10, diff->derivative(), prec);
    diff->input(0.25, 1);
    CPPUNIT_ASSERT(diff->derivative() < 0);
    diff->input(0.3, 0);
    CPPUNIT_ASSERT(diff->derivative() < 0);
    diff->input(0.4, 0);
    CPPUNIT_ASSERT(diff->derivative() > 0);
  }
};

class IntegratorTests : public CppUnit::TestFixture
{
private:
  CPPUNIT_TEST_SUITE(IntegratorTests);
  CPPUNIT_TEST_EXCEPTION(testZeroSegmentException1, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testZeroSegmentException2, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals1, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals2, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals3, std::invalid_argument);
  CPPUNIT_TEST_EXCEPTION(testSameIndVals4, std::invalid_argument);
  CPPUNIT_TEST(testRangeAbsolute);
  CPPUNIT_TEST(testRangeRelative);
  CPPUNIT_TEST(testUniformNonIntegerDomain);
  CPPUNIT_TEST(testNonUniformDomain);
  CPPUNIT_TEST_SUITE_END();

  RealTimeIntegrator *absInteg1;
  RealTimeIntegrator *absInteg2;
  RealTimeIntegrator *absInteg4;
  RealTimeIntegrator *relInteg1;
  RealTimeIntegrator *relInteg2;
  RealTimeIntegrator *relInteg4;
  double prec = 0.000001;

public:
  void setUp()
  {
    absInteg1 = new RealTimeIntegrator(1, false);
    absInteg2 = new RealTimeIntegrator(2, false);
    absInteg4 = new RealTimeIntegrator(4, false);
    relInteg1 = new RealTimeIntegrator(1, true);
    relInteg2 = new RealTimeIntegrator(2, true);
    relInteg4 = new RealTimeIntegrator(4, true);
  }

  void tearDown()
  {
    delete absInteg1;
    delete absInteg2;
    delete absInteg4;
    delete relInteg1;
    delete relInteg2;
    delete relInteg4;
  }

  void testZeroSegmentException1()
  {
    RealTimeIntegrator integ0(0, false);
  }

  void testZeroSegmentException2()
  {
    RealTimeIntegrator integ0(0, true);
  }

  void testSameIndVals1()
  {
    absInteg1->input(0, 1);
    absInteg1->input(0, 2);
  }

  void testSameIndVals2()
  {
    absInteg1->input(0.1, 1);
    absInteg1->input(0.2, 2);
    absInteg1->input(0.3, 3);
    absInteg1->input(0.3, 4);
  }

  void testSameIndVals3()
  {
    relInteg1->input(1, 1);
    relInteg1->input(1, 2);
  }

  void testSameIndVals4()
  {
    relInteg1->input(0.1, 1);
    relInteg1->input(0.2, 2);
    relInteg1->input(0.3, 3);
    relInteg1->input(0.3, 4);
  }

  void testRangeAbsolute()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg1->integral(), prec);
    absInteg1->input(1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg1->integral(), prec);
    absInteg1->input(2, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, absInteg1->integral(), prec);
    absInteg1->input(3, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(6.5, absInteg1->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg2->integral(), prec);
    absInteg2->input(2, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg2->integral(), prec);
    absInteg2->input(3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, absInteg2->integral(), prec);
    absInteg2->input(4, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(12, absInteg2->integral(), prec);
    absInteg2->input(5, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(13.5, absInteg2->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(2, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.5, absInteg4->integral(), prec);
    absInteg4->input(3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(11.5, absInteg4->integral(), prec);
    absInteg4->input(4, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(17.5, absInteg4->integral(), prec);
    absInteg4->input(5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(23.5, absInteg4->integral(), prec);
    absInteg4->input(6, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(24, absInteg4->integral(), prec);
  }

  void testRangeRelative()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg1->integral(), prec);
    relInteg1->input(0, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg1->integral(), prec);
    relInteg1->input(1, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, relInteg1->integral(), prec);
    relInteg1->input(2, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, relInteg1->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg2->integral(), prec);
    relInteg2->input(2, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg2->integral(), prec);
    relInteg2->input(3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, relInteg2->integral(), prec);
    relInteg2->input(4, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.0, relInteg2->integral(), prec);
    relInteg2->input(5, 7);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, relInteg2->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(2, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.5, relInteg4->integral(), prec);
    relInteg4->input(3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.5, relInteg4->integral(), prec);
    relInteg4->input(4, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.5, relInteg4->integral(), prec);
    relInteg4->input(5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(3.5, relInteg4->integral(), prec);
    relInteg4->input(6, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
  }

  void testUniformNonIntegerDomain()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(0.1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(0.2, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.55, absInteg4->integral(), prec);
    absInteg4->input(0.3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.15, absInteg4->integral(), prec);
    absInteg4->input(0.4, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.75, absInteg4->integral(), prec);
    absInteg4->input(0.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.35, absInteg4->integral(), prec);
    absInteg4->input(0.6, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.4, absInteg4->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(0.1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(0.2, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.05, relInteg4->integral(), prec);
    relInteg4->input(0.3, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.15, relInteg4->integral(), prec);
    relInteg4->input(0.4, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.25, relInteg4->integral(), prec);
    relInteg4->input(0.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.35, relInteg4->integral(), prec);
    relInteg4->input(0.6, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
  }

  void testNonUniformDomain()
  {
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, absInteg4->integral(), prec);
    absInteg4->input(1.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.75, absInteg4->integral(), prec);
    absInteg4->input(2.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(5.75, absInteg4->integral(), prec);
    absInteg4->input(3.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(11.75, absInteg4->integral(), prec);
    absInteg4->input(4.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(17.75, absInteg4->integral(), prec);
    absInteg4->input(4.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(18, absInteg4->integral(), prec);

    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(1, 5);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
    relInteg4->input(1.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.25, relInteg4->integral(), prec);
    relInteg4->input(2.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0.75, relInteg4->integral(), prec);
    relInteg4->input(3.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(1.75, relInteg4->integral(), prec);
    relInteg4->input(4.0, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(2.75, relInteg4->integral(), prec);
    relInteg4->input(4.5, 6);
    CPPUNIT_ASSERT_DOUBLES_EQUAL(0, relInteg4->integral(), prec);
  }
};

#endif
