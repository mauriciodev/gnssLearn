#ifndef ORBITREADER_H
#define ORBITREADER_H
#include <Eigen/Dense>
using namespace Eigen;
#include <QString>
#include <QDebug>
#include <QMap>
#include <QDateTime>
#include <QVector>

class orbitreader
{
private:

public:
    orbitreader();

    //map of satName > epoch > vector of data
    QMap<QString, //SatName
    QMap<QDateTime, //epoch
    QVector<double>
    >> navDataEpochs;

    // Should be virtual
    MatrixXd getSatPositionMatrix(QStringList sats, QDateTime epoch); //should have a virtual parent for every reading driver
    //not virtual
    void setNavData(QString sat, QDateTime epoch, QVector<double> dvalues);
    QVector<double> getNavData(QString sat, QDateTime epoch);
    Vector4d QVector2Eigen(QVector<double> values);
    QMap<QDateTime,QVector<double>>::iterator getLatestEpochIter(QString sat, QDateTime epoch);

};

#endif // ORBITREADER_H
