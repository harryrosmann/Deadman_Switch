#include "BitMatrixMath.h"
#include <stdint.h>
#include <string.h>

void copy(matrix_type * A, uint32_t rows, uint32_t cols, matrix_type * B)
{
  for (uint32_t i = 0; i < rows; i++)
  {
    for (uint32_t j = 0; j < cols; j++)
    {
      B[rows * i + j] = A[rows * i + j];
    }
  }
}

void multiply(matrix_type * A, matrix_type * B, uint32_t m, uint32_t p, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)

  memset(C, 0, sizeof(matrix_type) * m * n);

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      for (uint32_t k = 0; k < p; k++)
      {
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
      }
    }
  }

  // Convert to 1 and 0s since this is supposed to be a bit matrix
  for (uint32_t i = 0; i < m * n; i++)
  {
    C[i] &= 1;
  }
}

void AND(matrix_type * A, matrix_type * B, uint32_t m, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A & B (m x n)

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      C[n * i + j] = A[n * i + j] & B[n * i + j];
    }
  }
}

void OR(matrix_type * A, matrix_type * B, uint32_t m, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A & B (m x n)

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      C[n * i + j] = A[n * i + j] | B[n * i + j];
    }
  }
}

void XOR(matrix_type * A, matrix_type * B, uint32_t m, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A & B (m x n)

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      C[n * i + j] = A[n * i + j] ^ B[n * i + j];
    }
  }
}

void NOT(matrix_type * A, uint32_t m, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = ~A (m x n)

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      C[n * i + j] = ~A[n * i + j];
    }
  }
}

void transpose(matrix_type * A, uint32_t m, uint32_t n, matrix_type * C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)

  for (uint32_t i = 0; i < m; i++)
  {
    for (uint32_t j = 0; j < n; j++)
    {
      C[m * j + i] = A[n * i + j];
    }
  }
}

int32_t ismember(matrix_type * A, matrix_type * B, uint32_t m, uint32_t n, bool row)
{
  // A = input matrix (m x n)
  // B = input matrix (1 x n) if row is true, otherwise (m x 1)
  // m = number of rows in A
  // n = number of columns in A

  if (row)
  {
    for (uint32_t i = 0, j; i < m; i++)
    {
      for (j = 0; j < n; j++)
      {
        if (A[n * i + j] != B[j])
        {
          break;
        }
      }
      if (j == n)
      {
        return (int32_t)i;
      }
    }
  }
  else
  {
    for (uint32_t j = 0, i; j < n; j++)
    {
      for (i = 0; i < m; i++)
      {
        if (A[n * i + j] != B[i])
        {
          break;
        }
      }
      if (i == m)
      {
        return (int32_t)j;
      }
    }
  }

  return -1;
}
