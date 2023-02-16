#include "ganisakta/type/triangle.hpp"

namespace ganisakta
{
    namespace type
    {
        Triangle::Triangle()
        {
            a = 0;
            b = 0;
            c = 0;
            alpha = 0;
            beta = 0;
            gamma = 0;
        }

        Triangle::Triangle(double a, double b, double c, double alpha, double beta, double gamma)
        {
            this->a = a;
            this->b = b;
            this->c = c;
            this->alpha = alpha;
            this->beta = beta;
            this->gamma = gamma;
            this->calculateAll();
        }

        bool Triangle::operator==(const Triangle& triangle)
        {
            return a == triangle.a && b == triangle.b && c == triangle.c && alpha == triangle.alpha && beta == triangle.beta && gamma == triangle.gamma;
        }

        bool Triangle::operator!=(const Triangle& triangle)
        {
            return !(*this == triangle);
        }

        double& Triangle::operator[](const int& index)
        {
            switch (index)
            {
                case 0:
                    return a;
                case 1:
                    return b;
                case 2:
                    return c;
                case 3:
                    return alpha;
                case 4:
                    return beta;
                case 5:
                    return gamma;
                default:
                    throw std::out_of_range("Index out of range");
            }
        }

        void Triangle::calculateAll()
        {
            if (this->a > 0 && this->b > 0 && this->c > 0) {
                // Law of cosines
                this->alpha = acos((this->b * this->b + this->c * this->c - this->a * this->a) / (2 * this->b * this->c));
                this->beta = acos((this->a * this->a + this->c * this->c - this->b * this->b) / (2 * this->a * this->c));
                this->gamma = acos((this->a * this->a + this->b * this->b - this->c * this->c) / (2 * this->a * this->b));
            } else if (this->a > 0 && this->b > 0 && this->alpha > 0) {
                // Law of cosines and sine formula
                this->c = sqrt(this->a * this->a + this->b * this->b - 2 * this->a * this->b * cos(this->alpha));
                this->gamma = asin(this->b * sin(this->alpha) / this->c);
                this->beta = M_PI - this->alpha - this->gamma;
            } else if (this->a > 0 && this->c > 0 && this->beta > 0) {
                // Law of cosines and sine formula
                this->b = sqrt(this->a * this->a + this->c * this->c - 2 * this->a * this->c * cos(this->beta));
                this->gamma = asin(this->c * sin(this->beta) / this->b);
                this->alpha = M_PI - this->beta - this->gamma;
            } else if (this->b > 0 && this->c > 0 && this->gamma > 0) {
                // Law of cosines and sine formula
                this->a = sqrt(this->b * this->b + this->c * this->c - 2 * this->b * this->c * cos(this->gamma));
                this->alpha = asin(this->c * sin(this->gamma) / this->a);
                this->beta = M_PI - this->alpha - this->gamma;
            } else {
                throw std::out_of_range("Insufficient information");
                return;
            }
        }

        // double Triangle::getA()
        // {
        //     // Check if a is given
        //     if(this->a != 0)
        //     {
        //         return a;
        //     }

        //     // Check if b and c is given and is right triangle
        //     // if(this->b != 0 && this->c != 0 && this->alpha == M_PI_2)
        //     // {
        //     //     this->a = std::sqrt(std::pow(c, 2) - std::pow(b, 2));
        //     //     return this->a;
        //     // }

        //     // Check if b and c and alpha is given
        //     if(this->b != 0 && this->c != 0 && this->alpha != 0)
        //     {
        //         this->a = std::sqrt(std::pow(b, 2) + std::pow(c, 2) - 2 * b * c * std::cos(alpha));
        //         return this->a;
        //     }

        //     // Check if b and alpha is given
        //     if(this->b != 0 && this->alpha != 0)
        //     {
        //         this->a = b * std::tan(alpha);
        //         return this->a;
        //     }

        //     // Check if c and alpha is given
        //     if(this->c != 0 && this->alpha != 0)
        //     {
        //         this->a = c * std::sin(alpha);
        //         return this->a;
        //     }

        //     // Check if b and beta is given
        //     if(this->b != 0 && this->beta != 0)
        //     {
        //         this->a = b / std::tan(beta);
        //         return this->a;
        //     }

        //     // Check if c and beta is given
        //     if(this->c != 0 && this->beta != 0)
        //     {
        //         this->a = c * std::cos(beta);
        //         return this->a;
        //     }
        // }

        // double Triangle::getB()
        // {
        //     return b;
        // }

        // double Triangle::getC()
        // {
        //     return c;
        // }

        // double Triangle::getAlpha()
        // {
        //     return alpha;
        // }

        // double Triangle::getBeta()
        // {
        //     return beta;
        // }

        // double Triangle::getGamma()
        // {
        //     return gamma;
        // }
    }
}