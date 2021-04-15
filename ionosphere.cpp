#include "ionosphere.h"

ionosphere::ionosphere()
{

}

double ionosphere::TECFromDualFreq(double p1, double p2, double phi1, double phi2, double f1, double f2)
{
    double tec1=( (f1*f1*f2*f2)/ (f1*f1 - f2*f2) ) *(p2-p1)/40.3;
    double tec2=( (f1*f1*f2*f2)/ (f1*f1 - f2*f2) ) *(phi1-phi2)/40.3;
    //qDebug() << tec1 << tec2;
}
