#include "gnsstime.h"

gnsstime::gnsstime()
{

}

QDateTime gnsstime::toGPSTime(QDateTime utc_time, int leapSeconds=37)
{
    QDateTime TAI=utc_time.addSecs(leapSeconds);
    QDateTime t_gpps=TAI.addSecs(-19);
    //QDateTime
    //0h UTC (midnight) of January 5th to 6th 1980
    return t_gpps;
}



QDateTime gnsstime::toUt1(QDateTime utc_time)
{

}

QDateTime gnsstime::toTAI(QDateTime utc_time)
{

}

QDateTime gnsstime::toTT(QDateTime utc_time)
{

}



int gnsstime::getGPSWeek(QDateTime utc_time)
{
    double JD = gnsstime::julianDay(utc_time);
    int gpsWeek=floor((JD-2444244.5)/7);
    return gpsWeek;
}

double gnsstime::getGPSTimeOfWeek(QDateTime utc_time)
{
    double JD = gnsstime::julianDay(utc_time);
    int week=gnsstime::getGPSWeek(utc_time);
    int weekday=gnsstime::getGPSWeekDay(utc_time);
    double gpstime=weekday*24.*60*60+utc_time.time().hour()*60*60+utc_time.time().minute()*60+utc_time.time().second();
    return gpstime;

}

int gnsstime::getGPSWeekDay(QDateTime utc_time)
{
    double JD = gnsstime::julianDay(utc_time);
    return abs((int)floor(JD+1.5)%7);
}

double gnsstime::julianDay(QDateTime utc_time) {
    int m=utc_time.date().month();
    int d=utc_time.date().day();
    int y=utc_time.date().year();
    double h=utc_time.time().hour()+utc_time.time().minute()/60.+utc_time.time().second()/3600.;
    if (m<=2) {
        y=y-1;
        m=m+12;
    }
    double JD=floor(365.25*y)+floor(30.6001*(m+1))+d+h/24+1720981.5;
    return JD;
}

QDateTime gnsstime::GPSTimeToUTC(QDateTime gps_time, int leapSeconds)
{
    QDateTime utc=gps_time.addSecs(19).addSecs(-1*leapSeconds);
    return utc;
}
