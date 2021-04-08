#include "sp3.h"

sp3::sp3()
{

}

Vector4d sp3::getSatPosition(QString sat, QDateTime epoch) {
    Vector4d res;

    if (this->navDataEpochs[sat].keys().contains(epoch)) {
        //if there is no need to interpolate
        QVector<double> values=this->navDataEpochs[sat][epoch];
        res=this->QVector2Eigen(values);
        res[3]=res[3]*1e-6;
    } else { //interpolate
        //it's not on the file. Let's interpolate
        auto iter = this->getLatestEpochIter(sat,epoch);
        QVector<double>x,y,z,t;
        //finding first available epoch -5
        int i=0;
        auto begin=iter;
        while ((begin !=this->navDataEpochs[sat].begin()) and (i<5) ) {
            begin-=1;
            i++;
        }

        auto end=begin+10;
        for (auto it=begin; it!=end;it+=1) {
            //qDebug()<< it.key() << it.value();
            auto coords=it.value();
            x.append(coords[0]);
            y.append(coords[1]);
            z.append(coords[2]);
            t.append(this->navDataEpochs[sat].begin().key().msecsTo(it.key()));
        }
        auto t_new=this->navDataEpochs[sat].begin().key().msecsTo(epoch);
        res[3]=iter.value()[3]*1e-6;
        t_new-=res[3];
        res[0]=this->lagrangeInterp(t,x,t_new);
        res[1]=this->lagrangeInterp(t,y,t_new);
        res[2]=this->lagrangeInterp(t,z,t_new);
    }
    res[0]=res[0]*1000;
    res[1]=res[1]*1000;
    res[2]=res[2]*1000;
    return res;
}

double sp3::lagrangeInterp(QVector<double> x, QVector<double> y, double x_new) {
    double res=0,p=0;
    for (int i=0; i<x.length();i++) {
        p=1;
        for (int j=0; j<x.length();j++) {
            if (j!=i) {
                p*=(x_new-x[j])/(x[i]-x[j]);
            }
        }
        res+=y[i]*p;
    }
    return res;
}


QString sp3::getDateTimeFormat()
{
    return "yyyy M d h m s.zzz";
}


MatrixXd sp3::getSatPositionMatrix(QStringList sats, QDateTime epoch)
{
    //foreach (QString sat in )

}

bool sp3::readSP3File(QString filePath)
{
    QFile file(filePath);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Error: " << file.errorString();
        return false;
    }
    QTextStream in(&file);

    //skip header
    QString line="";
    QDateTime epoch;
    QString satName;
    //reading epochs
    QString skipChars="#/%+";
    while(!in.atEnd()) {
        QVector<double> dvalues;
        line = in.readLine();

        //Skipping header lines
        if (skipChars.contains(line[0])) {
            continue;
        }

        if (line.left(3)=="*  ") {
            QString epochString=line.mid(3,line.indexOf(".")+1).replace("  "," "); //3 msecs digits

            //This is a new satellite epoch          "2021  3  9  0  0  0"
            //epochString=                             "2021 10 19 13 12 10";
            epoch=QDateTime::fromString(epochString, this->getDateTimeFormat());

        } else if (line.left(1)=="P"){ //this is a position line
            satName=line.mid(1,3).trimmed();
            //these is more data for the epoch
            QStringList values=line.mid(4).split(" ", QString::SkipEmptyParts);
            foreach(QString value, values) {
                dvalues.append(value.toDouble());
            }
            this->setNavData(satName,epoch,dvalues);
        }
    }

    file.close();
    return true;
}
