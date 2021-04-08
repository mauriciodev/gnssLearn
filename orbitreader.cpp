#include "orbitreader.h"

Vector4d orbitreader::QVector2Eigen(QVector<double> values)
{
    Vector4d res;
    for (int i=0;i<4;i++) res[i]=values[i];
    return res;
}

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

/*! Get an iterator for the latest navigation data available on the data structure. The epoch must contain the given sattelite.*/
QMap<QDateTime,QVector<double>>::iterator orbitreader::getLatestEpochIter(QString sat, QDateTime epoch) {
    auto res= this->navDataEpochs[sat].lowerBound(epoch.addSecs(1)); //had to add one because when epoch is exactly the same as in nav data, QMap was returning the equal, not the next one.
    if (res != this->navDataEpochs[sat].begin()) {
        res-=1;
    }
    if (epoch<res.key()) {
        return this->navDataEpochs[sat].end();
    } else {
        return res;
    } //TODO: Check what's happening to the last index
}
