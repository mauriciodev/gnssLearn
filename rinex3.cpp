#include "rinex3.h"

rinex3::rinex3()
{
    this->fillSystemDelay();
}

QVector<double> rinex3::readNavDoubles(QString in) {
    //returns
    QVector<double> res;
    while (in.length() > 0) {
        res.append(in.left(19).toDouble());
        in=in.mid(19);
    }
    return res;
}

QVector<double> rinex3::readObsDoubles(QString in) {
    //returns
    QVector<double> res;
    while (in.length() > 0) {
        res.append(in.left(14).toDouble());
        in=in.mid(16); //skipping loss of lock and signal strength digits
    }
    return res;
}


/*! Newton Raphson's solve for the elevation angle .
 * x(i+1) = x(i) - f(x) / f'(x) */
double rinex3::solveEk(double e, double Mk)
{
    double EPSILON=0.0001;
    double x=0;
    double h = 10;
    while (abs(h) >= EPSILON)
    {
        double fx=Mk+e*sin(x)-x;
        double dfx=e*cos(x)-1;
        h = fx/dfx;
        x = x - h;
    }
    return x;
}

/*! Reads a Rinex 3 navigation file and saves the data on the navDataEpochs map.
 * A pair of PRN (Ex.: E31) and qdatetime is used as index for each satellite epoch.*/
bool rinex3::readNavFile(QString filePath)
{
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: " << file.errorString();
        return false;
    }
    QTextStream in(&file);

    //skip header
    QString line="";
    this->navHeader.clear();
    while(!in.atEnd() and !line.contains("END OF HEADER")) {
        line = in.readLine();
        this->navHeader.append(line);
    }
    QDateTime epoch;
    QString satName;
    //reading epochs
    this->navDataEpochs.clear();
    while(!in.atEnd()) {
        QVector<double> dvalues;
        line = in.readLine();

        if (line[0].isLetter()) {
            //This is a new satellite epoch "M1d1y9800:01:02",
            satName=line.mid(0,4).trimmed();
            //epoch.addSecs(this->getSystemDelay(satName));
            if (!this->ignoreSystems.contains(satName[0])) {
                QString ds=line.mid(4, 19);
                epoch=QDateTime::fromString(ds, this->rinexDateFormatString());
                QString values=line.mid(23);
                dvalues.append(this->readNavDoubles(values));
                this->setNavData(satName,epoch,dvalues);
                //qDebug() << satName << epoch << this->navDataEpochs[satName].size();
            }
        } else {
            //these is more data for the epoch
            QString values=line.mid(4);
            if (!this->ignoreSystems.contains(satName[0])) {
                this->setNavData(satName,epoch,this->readNavDoubles(values));
            }
        }
    }

    file.close();
    return true;
}


/*! Reads a Rinex 3 observation file and saves the data on the navDataEpochs map.
 * A pair of PRN (Ex.: E31) and qdatetime is used as index for each satellite epoch.*/
bool rinex3::readObsFile(QString filePath)
{
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: " << file.errorString();
        return false;
    }
    QTextStream in(&file);

    //skip header
    QString line="";
    QString currSystem="";
    this->obsHeader.clear();
    while(!in.atEnd() and !line.contains("END OF HEADER")) {
        line = in.readLine();
        this->obsHeader.append(line);
        if (line.contains("SYS / # / OBS TYPES")) {
            //Filling what kind of codes each system describes.
            QString t=line.mid(7,line.indexOf("SYS")-8).trimmed();
            if (line[0]!=" ") { //new system
                currSystem=line[0];
                obsTypes[currSystem]=t.split(" ");
            } else { //line continuing the current system
                obsTypes[currSystem].append(t.split(" "));
            }

            //qDebug()<<currSystem << ": "<< obsTypes[currSystem];
        }
    }
    QDateTime epoch;
    QString satName;
    //reading epochs
    obsDataEpochs.clear();
    while(!in.atEnd()) {
        QVector<double> dvalues;
        line = in.readLine();

        if (line[0]=='>') {
            //This is a new satellite epoch
            epoch=QDateTime::fromString( line.mid(2, 19), this->rinexDateFormatString());
            int msec=line.mid(22,7).toInt();
            epoch=epoch.addMSecs(msec);
        } else {
            //these are more data for the epoch
            QString values=line.mid(3);
            satName=line.left(3); //prn
            dvalues.append(this->readObsDoubles(values));
            obsDataEpochs[epoch][satName]=dvalues;
        }
    }

    file.close();
    return true;
}

bool rinex3::isPhase(QString obsType)
{
    return (obsType[0]=='L');
}

bool rinex3::isCode(QString obsType)
{
    return (obsType[0]=='C');
}

/*! Returns a string that can be used to transforms RINEX's date as text to QDateTime.*/
QString rinex3::rinexDateFormatString()
{
    return "yyyy MM dd hh mm ss";
}

/*! This method should work for all systems.*/
double rinex3::getClockBias(QString sat, QDateTime epoch)
{
    auto it=this->getLatestEpochIter(sat,epoch);
    return (*it)[0];
}

VectorXd rinex3::getClockBiasVector(QStringList sats, QDateTime epoch) {
    VectorXd v;
    v.resize(sats.length());
    for (int i=0; i<sats.length(); i++){
        v[i]=this->getClockBias(sats[i],epoch);
    }
    return v;
}

/*! Returns a vector of doubles with the satellite coordinates at a given time. If computations fail, returns an empty vector.*/
Vector4d rinex3::getSatPosition(QString sat, QDateTime ttDate)
{

    Vector4d coords=Vector4d::Zero();
    if (! this->ignoreSystems.contains(this->getSatType(sat))) {
        //I should be searching for the latest epoch here. Not sure what to do.
        auto latestEpochIter=this->getLatestEpochIter(sat,ttDate);
        if (latestEpochIter.key()>ttDate) {
            qDebug()<<"Could not find satellite data for "<<sat<<" at "<<ttDate;
            return coords; // empty vector
        }
        QVector<double> nav=(*latestEpochIter);
        QDateTime tocDate=latestEpochIter.key();
        double toc=gnsstime::getGPSTimeOfWeek(tocDate);
        double toe=nav[11];
        double tt=gnsstime::getGPSTimeOfWeek(ttDate);
        //from Monico (2008) chapter 4
        double dtSat=nav[0]+nav[1]*(tt-toc)+nav[2]* pow(tt-toc ,2);
        double t_gps=tt-dtSat;
        double dtk=t_gps-toe;
        if (dtk>302400) dtk-=604800;
        if (dtk<-302400) dtk+=604800;

        double crs=nav[4];
        double dn=nav[5];
        double M0=nav[6];
        double cuc=nav[7];
        double e=nav[8];
        double cus=nav[9];
        double srqt_a=nav[10];

        double cic=nav[12];
        double OMEGA=nav[13];
        double cis=nav[14];
        double i0=nav[15];
        double crc=nav[16];
        double w=nav[17];
        double OMEGA_DOT=nav[18];
        double IDOT=nav[19];

        double Mk=M0+(sqrt(GM)/pow(srqt_a,3)+dn)*dtk;
        double Ek=this->solveEk(e,Mk); //newton raphson, really?
        double cosv=(cos(Ek)-e)/(1-e*cos(Ek));
        double sinv=sqrt(1-e*e)*sin(Ek)/(1-e*cos(Ek));
        double v=acos(cosv);
        if (sinv<0) v=v*-1.;
        if (v<0) v+=2*PI;
        double phi=w+v;
        double uk=phi+cuc*cos(2*phi)+cus*sin(2*phi);
        double r=srqt_a*srqt_a*(1-e*cos(Ek))+crc*cos(2*phi)+crs*sin(2*phi);
        double i=i0+IDOT*dtk+cic*cos(2*phi)+cis*sin(2*phi);

        double xk=r*cos(uk);
        double yk=r*sin(uk);
        double om=OMEGA+(OMEGA_DOT)*dtk-omega_e*t_gps;
        double X=xk*cos(om)-yk*sin(om)*cos(i);
        double Y=xk*sin(om)+yk*cos(om)*cos(i);
        double Z=yk*sin(i);
        //qDebug()<<fixed << sat<< X << Y << Z;

        coords={X,Y,Z,dtSat};
    }
    return coords;
}

QString rinex3::getSatType(QString sat)
{
    return sat.at(0);
}



QVector<double> rinex3::getLatestNavData(QString sat, QDateTime epoch)
{
    auto latestEpoch= this->navDataEpochs[sat].lowerBound(epoch); //this is an iterator
    return (*latestEpoch);
}

/*! This method sets up the leap seconds to be used later when syncronizing observations and ephemeris. This could be changed for a QSettings or a json file later.*/
void rinex3::fillSystemDelay(int leapSeconds)
{
    //TODO: Check these values
    this->GNSSToUTCDelay["G"]=leapSeconds-19;
    this->GNSSToUTCDelay["E"]=leapSeconds-13;
    this->GNSSToUTCDelay["R"]=0;
    this->GNSSToUTCDelay["C"]=leapSeconds;
}

double rinex3::getSystemDelay(QString systemChar)
{
    return this->GNSSToUTCDelay[systemChar.at(0)];
}




MatrixXd rinex3::getSatPositionMatrix(QStringList sats, QDateTime epoch)
{
    MatrixXd m;
    m.resize(sats.length(),4);
    for (int i=0;i<sats.length(); i++){
        auto v = this->getSatPosition(sats[i],epoch);
        if (v==Vector4d::Zero()) { //sorry, couldn't find a sattelite position. This should be aborted.
            return m;
        } else {
            //qDebug() << sats[i] << v ;
            m(i,0)=v[0];
            m(i,1)=v[1];
            m(i,2)=v[2];
            m(i,3)=v[3];
        }
    }
    return m;
}

QStringList rinex3::getObsSatellitesOnEpoch(QDateTime epoch)
{
    QStringList sats=this->obsDataEpochs[epoch].keys();
    for (int i=sats.length()-1; i>-1;i--) {
        if ( this->ignoreSystems.contains(sats[i][0])) {
            sats.removeAt(i);
        }
    }
    return sats;
}



QVector<double> rinex3::getObs(QString sat, QDateTime epoch)
{
    if (this->obsDataEpochs[epoch].contains(sat)) {
        QVector<double> values=this->obsDataEpochs[epoch][sat];
        return values;
    } else {
        QVector<double> res;
        return res;
    }
}

Vector3d rinex3::getX0()
{
    Vector3d X0; X0<<0,0,0;
    foreach (QString line, this->obsHeader) {
        if (line.contains("APPROX POSITION XYZ")){
            auto parts=line.split(" ",QString::SkipEmptyParts);
            X0[0]=parts[0].toDouble();
            X0[1]=parts[1].toDouble();
            X0[2]=parts[2].toDouble();
            return X0;
        }
    }
    return X0;
}

/*! Returns a 1xN vector containing only the P1 observable for each satellite on the list.*/
VectorXd rinex3::getC1Vector(QStringList sats, QDateTime epoch) {
    VectorXd v;
    v.resize(sats.length());
    for (int i=0;i<sats.length(); i++){
        v[i]=this->obsDataEpochs[epoch][sats[i]][0];
    }
    return v;
}








