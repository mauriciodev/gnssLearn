#include <QtTest>
#include "rinex3.h"
#include "sp3.h"
#include "gnsstime.h"
#include "spp.h"
// add necessary includes here

class gnssLearnTests : public QObject
{
    Q_OBJECT

public:
    gnssLearnTests();
    ~gnssLearnTests();

private slots:
    void test_gnsstime();
    void test_RINEX3();
    void test_spp();
    void test_sp3();

};

gnssLearnTests::gnssLearnTests()
{

}

gnssLearnTests::~gnssLearnTests()
{

}

void gnssLearnTests::test_gnsstime()
{
    rinex3 test1;
    QString dateFormat=test1.rinexDateFormatString();

    QDateTime epoch2=QDateTime::fromString("2017 01 05 09 15 56", dateFormat);
    qDebug()<<"Julian Day:" << fixed << gnsstime::julianDay(epoch2) << "QT:" << epoch2.date().toJulianDay();
    qDebug()<<"GPS Week:" << fixed << gnsstime::getGPSWeek(epoch2);
    qDebug()<<"GPS time of week: "<< fixed<<gnsstime::getGPSTimeOfWeek(epoch2);
}

void gnssLearnTests::test_RINEX3() {
    rinex3 test1;
    QString dateFormat=test1.rinexDateFormatString();

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
    MatrixXd satPos=test1.getSatPositionMatrix(sats,epoch);
    std::cout <<  satPos <<endl;
    qDebug() << test1.getObs("G14",epoch);
}
void gnssLearnTests::test_spp() {
    spp sppTest;
    //TODO: should be a for for each epoch on obs file
    QDateTime epoch;
    epoch=QDateTime::fromString("2021 03 09 22 00 00", sppTest.readingDriver.rinexDateFormatString());
    //sppTest.runSPP("./data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx","./data/BRDC00WRD_S_20210680000_01D_MN.rnx",epoch);
    epoch=QDateTime::fromString("2021 03 09 23 30 00", sppTest.readingDriver.rinexDateFormatString());
    sppTest.runSPP("./data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx","./data/BRDC00WRD_S_20210680000_01D_MN.rnx",epoch);
};

void gnssLearnTests::test_sp3() {
    sp3 sp3Test;

    sp3Test.readSP3File("./data/gfz21482.sp3");
};

QTEST_APPLESS_MAIN(gnssLearnTests)
#include "test.moc"

