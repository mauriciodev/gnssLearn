#ifndef IONOSPHERE_H
#define IONOSPHERE_H
#include <QStringList>

class ionosphere
{
public:
    ionosphere();
    double TECFromDualFreq(double p1, double p2, double phi1, double phi2, double f1, double f2);
};

#endif // IONOSPHERE_H
