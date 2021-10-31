/*
 * File: rational.h
 * ----------------
 * This interface exports a class for representing rational numbers.
 */

#ifndef _rational_h
#define _rational_h

#include <string>
#include <iostream>

/*
 * Class: Rational
 * ---------------
 * The Rational class is used to represent rational numbers, which
 * are defined to be the quotient of two integers.
 */

class Rational {

/* Public section */

public:

/*
 * Constructor: Rational
 * Usage: Rational zero;
 *        Rational num(n);
 *        Rational r(x, y);
 * ------------------------
 * Creates a Rational object.  The default constructor creates the
 * rational number 0.  The single-argument form creates a rational
 * number equal to the specified integer, and the two-argument form
 * creates a rational number corresponding to the fraction x/y.
 */

   Rational();
   Rational(int n);
   Rational(int x, int y);

/*
 * Method: toString()
 * Usage: string str = r.toString();
 * ---------------------------------
 * Returns the string representation of this rational number.
 */

   std::string toString();

/*
 * Operator: +
 * Usage: r1 + r2
 * --------------
 * Creates a new Rational that is equal to r1 + r2.
 */

Rational operator+(Rational r2);

/*
 * Operator: -
 * Usage: r1 - r2
 * --------------
 * Creates a new Rational that is equal to r1 - r2.
 */

Rational operator-(Rational r2);

/*
 * Operator: *
 * Usage: r1 * r2
 * --------------
 * Creates a new Rational that is equal to r1 * r2.
 */

Rational operator*(Rational r2);

/*
 * Operator: /
 * Usage: r1 / r2
 * --------------
 * Creates a new Rational that is equal to r1 / r2.
 */

Rational operator/(Rational r2);

/* Private section */

private:

/* Instance variables */

   int num;    /* The numerator of this Rational object   */
   int den;    /* The denominator of this Rational object */

};

/*
 * Operator: <<
 * Usage: cout << rat;
 * -------------------
 * Overloads the << operator so that it is able to display Rational values.
 */

std::ostream & operator<<(std::ostream & os, Rational rat);

#endif
