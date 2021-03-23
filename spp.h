#ifndef SPS_H
#define SPS_H
#include <Eigen/Dense>
#include <QStringList>
#include <QDateTime>
#include "rinex3.h" //this should be a reader driver later
#include "math.h"
using namespace Eigen;

class spp
{

public:
    spp();
    rinex3 readingDriver;
    Vector3d runSPP(QString rinexObsFile, QString rinexNavFile, QDateTime epoch);
    MatrixXd getObservationMatrix(QStringList sats, QDateTime epoch);
};

#endif // SPS_H
