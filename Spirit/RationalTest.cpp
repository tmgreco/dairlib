/*
 * File: RationalTest.cpp
 * ----------------------
 * This file conducts several tests of the Rational class.
 */

#include <iostream>
#include "rational.h"
#include "console.h"
using namespace std;

/* Function prototypes */

void testSumOfThree();
void testIntegerPromotion();
void testLowestTerms();
void testArithmeticOperators();

/* Main program */

int main() {
   testSumOfThree();
   testArithmeticOperators();
   testIntegerPromotion();
   return 0;
}

void testSumOfThree() {
   Rational a(1, 2);
   Rational b(1, 3);
   Rational c(1, 6);
   Rational sum = a + b + c;
   cout << a << " + " << b << " + " << c << " = " << sum << endl;
}

void testIntegerPromotion() {
   cout << "1 + 1/2 = " << Rational(1) + Rational(1, 2) << endl;
   cout << "1/3 - 1 = " << Rational(1, 3) - 1 << endl;
}

void testLowestTerms() {
   cout << " Rational(2, 4) = " << Rational(2, 4) << endl;
   cout << " Rational(0, 6) = " << Rational(0, 6) << endl;
   cout << " Rational(42, -1) = " << Rational(42, -1) << endl;
}

void testArithmeticOperators() {
   cout << "1/2 + 1/2 = " << Rational(1, 2) + Rational(1, 2) << endl;
   cout << "3/4 - 1/2 = " << Rational(3, 4) - Rational(1, 2) << endl;
   cout << "2/3 * 1/4 = " << Rational(2, 3) * Rational(1, 4) << endl;
   cout << "2/3 / 1/2 = " << Rational(2, 3) / Rational(1, 2) << endl;
   cout << "-1/2 + 1/2 = " << Rational(-1, 2) + Rational(1, 2) << endl;
   cout << "-3/4 - 1/2 = " << Rational(-3, 4) - Rational(1, 2) << endl;
   cout << "-2/3 * 1/4 = " << Rational(-2, 3) * Rational(1, 4) << endl;
   cout << "-2/3 / 1/2 = " << Rational(-2, 3) / Rational(1, 2) << endl;
}
