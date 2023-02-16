#pragma once

namespace ganisakta
{
    namespace triangle
    {
        class Triangle{
            private:
                double _a;
                double _b;
                double _c;
                double _alpha;
                double _beta;
                double _gamma;
            public:
                Triangle();
                Triangle(double a, double b, double c, double alpha, double beta, double gamma);
                double getA();
                double getB();
                double getC();
                double getAlpha();
                double getBeta();
                double getGamma();
                void setA(double a);
                void setB(double b);
                void setC(double c);
                void setAlpha(double alpha);
                void setBeta(double beta);
                void setGamma(double gamma);
                void setAll(double a, double b, double c, double alpha, double beta, double gamma);
                void calculate();
        };
    }
}