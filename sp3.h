#ifndef SP3_H
#define SP3_H
#include "orbitreader.h"
#include <QFile>

class sp3 : public orbitreader
{
public:
    sp3();

    double getClockBias(QString sat, QDateTime epoch);
    MatrixXd getSatPositionMatrix(QStringList sats, QDateTime epoch);
    Vector4d getSatPosition(QString sat, QDateTime epoch);
    QString getDateTimeFormat();
    bool readSP3File(QString filePath);
    double lagrangeInterp(QVector<double> x, QVector<double> y, double x_new);

};

#endif // SP3_H
