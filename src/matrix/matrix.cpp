#include "ganisakta/matrix/matrix.hpp"

namespace ganisakta
{
    namespace matrix
    {
        Matrix::Matrix(int r, int c)
        {
            rows = r;
            cols = c;
            data = new float[rows * cols];
        }

        Matrix::~Matrix()
        {
            delete[] data;
        }

        float& Matrix::operator()(int i, int j) const
        {
            return data[i * cols + j];
        }

        Matrix Matrix::operator*(const Matrix& other) const
        {
            Matrix result(rows, other.cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < other.cols; ++j)
                {
                    result(i, j) = 0;
                    for (int k = 0; k < cols; ++k)
                        result(i, j) += (*this)(i, k) * other(k, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator+(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) + other(i, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator-(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) - other(i, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator/(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) / other(i, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator+=(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) + other(i, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator-=(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) - other(i, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator*=(const Matrix& other) const
        {
            Matrix result(rows, other.cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < other.cols; ++j)
                {
                    result(i, j) = 0;
                    for (int k = 0; k < cols; ++k)
                        result(i, j) += (*this)(i, k) * other(k, j);
                }
            }

            return result;
        }

        Matrix Matrix::operator/=(const Matrix& other) const
        {
            Matrix result(rows, cols);

            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    result(i, j) = (*this)(i, j) / other(i, j);
                }
            }

            return result;
        }
    } // namespace matrix
} // namespace ganisakta
