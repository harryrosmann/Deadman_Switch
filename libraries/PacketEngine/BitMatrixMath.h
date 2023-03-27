#ifndef __BIT_MATRIX_MATH__
#define __BIT_MATRIX_MATH__

#include <stdint.h>
#include <stdbool.h>

// Allows for user to use different types
#ifndef matrix_type
#define matrix_type uint8_t
#endif

void copy(matrix_type *A, uint32_t rows, uint32_t cols, matrix_type *B);
void multiply(matrix_type *A, matrix_type *B, uint32_t m, uint32_t p, uint32_t n, matrix_type *C);
void AND(matrix_type *A, matrix_type *B, uint32_t m, uint32_t n, matrix_type *C);
void OR(matrix_type *A, matrix_type *B, uint32_t m, uint32_t n, matrix_type *C);
void XOR(matrix_type *A, matrix_type *B, uint32_t m, uint32_t n, matrix_type *C);
void NOT(matrix_type *A, uint32_t m, uint32_t n, matrix_type *C);
void transpose(matrix_type *A, uint32_t m, uint32_t n, matrix_type *C);
int32_t ismember(matrix_type *A, matrix_type *B, uint32_t m, uint32_t n, bool row = true);

#endif
