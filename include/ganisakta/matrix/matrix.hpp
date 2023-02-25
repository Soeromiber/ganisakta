#pragma once

namespace ganisakta
{
    namespace matrix
    {
        struct Matrix
        {
            int rows;
            int cols;
            float* data;

            Matrix(int r, int c);

            ~Matrix();

            float& operator()(int i, int j) const;

            Matrix operator*(const Matrix& other) const;
            Matrix operator+(const Matrix& other) const;
            Matrix operator-(const Matrix& other) const;
            Matrix operator/(const Matrix& other) const;
            Matrix operator+=(const Matrix& other) const;
            Matrix operator-=(const Matrix& other) const;
            Matrix operator*=(const Matrix& other) const;
            Matrix operator/=(const Matrix& other) const;
        };
        
    } // namespace matrix
} // namespace ganisakta
