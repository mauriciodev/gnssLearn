#ifndef SP3_H
#define SP3_H
#include "orbitreader.h"
#include <QFile>

class sp3 : public orbitreader
{
public:
    sp3();

    double getClockBias(QString sat, QDateTime epoch);
    virtual MatrixXd getSatPositionMatrix(QStringList sats, QDateTime epoch);
    bool readSP3File(QString filePath);
};

#endif // SP3_H
