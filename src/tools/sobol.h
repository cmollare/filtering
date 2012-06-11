/*!
 * \file sobol.h
 * \brief Functions for QRS, extracted from Mathias Fontmarty's thesis.
 */

int i4_bit_hi1 ( int n );
int i4_bit_lo0 ( int n );
int i4_max ( int i1, int i2 );
int i4_min ( int i1, int i2 );
void i4_sobol ( int dim_num, int *seed, float quasi[ ] );
int i4_uniform ( int b, int c, int *seed );
unsigned int i4_xor ( unsigned int i, unsigned int j );
int i8_bit_hi1 ( long int n );
int i8_bit_lo0 ( long int n );
long int i8_max ( long int i1, long int i2 );
long int i8_min ( long int i1, long int i2 );
void i8_sobol ( int dim_num, long int *seed, double quasi[ ] );
long int i8_uniform ( long int b, long int c, int *seed );
unsigned long int i8_xor ( unsigned long int i, unsigned long int j );
float r4_abs ( float x );
int r4_nint ( float x );
float r4_uniform_01 ( int *seed );
double r8_abs ( double x );
int r8_nint ( double x );
double r8_uniform_01 ( int *seed );
void timestamp ( void );

