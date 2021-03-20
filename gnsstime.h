#ifndef GNSSTIME_H
#define GNSSTIME_H
#include <QString>
#include <QDebug>
#include <QFile>
#include <Qt>
#include <QMap>
#include <QDateTime>
#include <QVector>
#include <math.h>

class gnsstime
{
public:
    gnsstime();

    static QDateTime toGPSTime(QDateTime utc_time, int leapSeconds);
    static QDateTime toUt1(QDateTime utc_time);
    static QDateTime toTAI(QDateTime utc_time);
    static QDateTime toTT(QDateTime utc_time);
    static double getGPSTimeOfWeek(QDateTime utc_time);
    static int getGPSWeekDay(QDateTime utc_time);
    static int getGPSWeek(QDateTime utc_time);
    static double julianDay(QDateTime utc_time);
    static QDateTime GPSTimeToUTC(QDateTime gps_time, int leapSeconds=37);
};

#endif // GNSSTIME_H
