#ifndef RINEX3_H
#define RINEX3_H
#include <QString>
#include <QDebug>
#include <QFile>
#include <Qt>
#include <QMap>
#include <QDateTime>
#include <QVector>
#include <math.h>
#include "gnsstime.h"
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

#define LIGHTSPEED      2.99792458e8    /* m/sec */
#define PI              3.1415926535898
#define GM              3.986004418E14
#define omega_e         7.2921151467E-5


typedef QPair<QString,QDateTime> satEpoch;


class rinex3
{
private:
    QString ignoreSystems="SR";
    QVector<double> readNavDoubles(QString in);
    QVector<double> readObsDoubles(QString in);
    QStringList obsHeader,navHeader;
    double solveEk(double e, double Mk);

public:
    rinex3();
    //maps of epoch > satName > vector of data
    QMap<QDateTime, //epoch
    QMap<QString, //SatName
    QVector<double>
    >> obsDataEpochs;
    //map of satName > epoch > vector of data
    QMap<QString, //SatName
    QMap<QDateTime, //epoch
    QVector<double>
    >> navDataEpochs;
    QMap<QString, double> GNSSToUTCDelay;
    //QMap <QDateTime, QMap<QString,QVector<double>> obsDataEpochs;
    //QMap <satEpoch, QVector<double>> navDataEpochs;
    QMap <QString, QStringList> obsTypes; //map of observation types for each satellite.

    bool readNavFile(QString filePath);
    bool readObsFile(QString filePath);
    bool isPhase(QString obsType);
    bool isCode(QString obsType);

    QString rinexDateFormatString();


    //navigation rinex functions
    double getClockBias(QString sat, QDateTime epoch);
    VectorXd getClockBiasVector(QStringList sats, QDateTime epoch); //should have a virtual parent for every reading driver
    QVector<double> getSatPos(QString sat, QDateTime ttDate);
    QString getSatType(QString sat);
    QVector<double> getLatestNavData(QString sat, QDateTime epoch);
    QMap<QDateTime,QVector<double>>::iterator getLatestEpochIter(QString sat, QDateTime epoch);

    void fillSystemDelay(int leapSeconds=37);
    double getSystemDelay(QString systemChar);

    void setNavData(QString sat, QDateTime epoch, QVector<double> dvalues);
    QVector<double> getNavData(QString sat, QDateTime epoch);
    MatrixXd getSatPositionMatrix(QStringList sats, QDateTime epoch); //should have a virtual parent for every reading driver

    //observation rinex functions
    QStringList getObsSatellitesOnEpoch(QDateTime epoch);
    QVector<double> getObs(QString sat, QDateTime epoch);
    Vector3d getX0();

    VectorXd getC1Vector(QStringList sats, QDateTime epoch);//should have a virtual parent for every reading driver
};

#endif // RINEX3_H


