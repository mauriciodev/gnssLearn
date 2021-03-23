#include "orbitreader.h"

orbitreader::orbitreader()
{

}
void orbitreader::setNavData(QString sat, QDateTime epoch, QVector<double> dvalues)
{//if data already exists, appends
    navDataEpochs[sat][epoch].append(dvalues);
}
QVector<double> orbitreader::getNavData(QString sat, QDateTime epoch)
{
    return navDataEpochs[sat][epoch];
}
