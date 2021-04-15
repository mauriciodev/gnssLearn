#include <QtTest>
#include "rinex3.h"
#include "sp3.h"
#include "gnsstime.h"
#include "spp.h"
#include "ionosphere.h"
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
    void test_Lagrange();
    void test_sp3vsnav();
    void test_TEC();
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
    QBENCHMARK {
    test1.readObsFile("./data/RJNI00BRA_R_20210680000_01D_15S_MO.rnx");
    }
    qDebug()<<test1.obsDataEpochs.size() << " observations read.";

    //orbit tests
    //QDateTime epoch=QDateTime::fromString("2021 03 10 00 00 00", dateFormat);
    QDateTime epoch=QDateTime::fromString("2021 03 09 23 59 45", dateFormat);
    qDebug()<<"Map of observables: " <<test1.getObsMap("G14",epoch)<<endl; //this function requires exact epoch
    qDebug()<<"Single observable C1C: " <<test1.getObs("G14",epoch,"C1C")<<endl;
    epoch=QDateTime::fromString("2021 03 09 23 30 00", dateFormat);
    qDebug()<< "Date:"<< epoch;
    cout<<"G15 satelite position: "<<test1.getSatPosition("G15",epoch).transpose();
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
}

void gnssLearnTests::test_sp3() {
    sp3 sp3Test;
    sp3Test.readSP3File("./data/gfz21482.sp3");
    qDebug()<<sp3Test.navDataEpochs["G11"];
    QDateTime epoch=QDateTime::fromString("2021 3 9 0 15 0.000", sp3Test.getDateTimeFormat());
    cout<<sp3Test.getSatPosition("G11",epoch)<<endl;
    epoch=QDateTime::fromString("2021 3 9 0 7 0.000", sp3Test.getDateTimeFormat());
    cout<<sp3Test.getSatPosition("G11",epoch)<<endl;
}


void gnssLearnTests::test_sp3vsnav() {
    sp3 sp3Test;
    sp3Test.readSP3File("./data/COD0MGXFIN_20210680000_01D_05M_ORB.SP3");
    rinex3 rinexTest;
    rinexTest.readNavFile("./data/BRDC00WRD_S_20210680000_01D_MN.rnx");
    QString satName="E03";

    QDateTime epoch=QDateTime::fromString("2021 3 9 2 7 0.000", sp3Test.getDateTimeFormat());

    foreach (auto e,sp3Test.navDataEpochs[satName].keys()) {
        Vector4d res1=sp3Test.getSatPosition(satName,e);
        res1[3]=res1[3]*1e6; //dt is microsecs
        cout<<std::fixed <<"SP3: "<<res1.transpose() << " ";
        Vector4d res2=rinexTest.getSatPosition(satName,e);
        res2[3]=res2[3]*1e6; //dt is microsecs
        cout<<std::fixed <<" RINEx: "<<res2.transpose();
        //Let's set the dt to zero to compute the distance
        res1[3]=res2[3]=0;
        double dist=(res1-res2).norm();
        cout<< " Distance: "<<dist<<endl;
    }
}

void gnssLearnTests::test_Lagrange() {
    QVector<double> x= {1, 1.3, 1.6, 1.9, 2.2};
    QVector<double> y= {0.1411, -0.6878, -0.9962, -0.5507, 0.3115};
    sp3 sp3Test;
    qDebug()<<sp3Test.lagrangeInterp(x,y,1.5) << -0.9773;
}

void gnssLearnTests::test_TEC() {

}

QTEST_APPLESS_MAIN(gnssLearnTests)
#include "test.moc"

