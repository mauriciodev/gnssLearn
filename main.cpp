#include <QCoreApplication>
#include "rinex3.h"
#include "gnsstime.h"
#include "sps.h"

int main(int argc, char *argv[])
{



    QCoreApplication a(argc, argv);
    //gnsstime tests
    rinex3 test1;
    QString dateFormat=test1.rinexDateFormatString();

    QDateTime epoch2=QDateTime::fromString("2017 01 05 09 15 56", dateFormat);
    qDebug()<<"Julian Day:" << fixed << gnsstime::julianDay(epoch2) << "QT:" << epoch2.date().toJulianDay();
    qDebug()<<"GPS Week:" << fixed << gnsstime::getGPSWeek(epoch2);
    qDebug()<<"GPS time of week: "<< fixed<<gnsstime::getGPSTimeOfWeek(epoch2);
    //rinex tests

    test1.readNavFile("./data/BRDC00WRD_S_20210680000_01D_MN.rnx");
    qDebug()<<test1.navDataEpochs.size() << " satellite epochs read.";
    test1.readObsFile("./data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx");
    qDebug()<<test1.obsDataEpochs.size() << " observations read.";

    //orbit tests
    //QDateTime epoch=QDateTime::fromString("2021 03 10 00 00 00", dateFormat);
    QDateTime epoch=QDateTime::fromString("2021 03 09 23 30 00", dateFormat);

    qDebug()<< "Date:"<< epoch;
    qDebug()<<"G15 satelite position: "<<test1.getSatPos("G15",epoch);
    //epoch=QDateTime::fromString("2021 03 09 05 29 30", dateFormat);
    qDebug()<<"GPS Week:" << fixed << gnsstime::getGPSWeek(epoch);
    qDebug()<<"GPS day of week:" << fixed << gnsstime::getGPSWeekDay(epoch);
    qDebug()<<"GPS time of week: "<< fixed<<gnsstime::getGPSTimeOfWeek(epoch);
    //qDebug()<<"E11 clock bias"<<test1.getClockBias("G15",epoch);
    //qDebug()<<"E11 satelite position: "<<test1.getSatPos("G15",epoch);
    auto sats=test1.getObsSatellitesOnEpoch(epoch);

    qDebug() << sats;
    std::cout << test1.getSatPositionMatrix(sats,epoch) <<endl;
    qDebug() << test1.getObs("G14",epoch);
    sps spsTest;
    spsTest.runSPS("./data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx","./data/BRDC00WRD_S_20210680000_01D_MN.rnx");

    //getSatPositionMatrix
    return a.exec();
}
