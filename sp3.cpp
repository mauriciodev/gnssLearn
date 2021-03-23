#include "sp3.h"

sp3::sp3()
{

}

MatrixXd sp3::getSatPositionMatrix(QStringList sats, QDateTime epoch)
{

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
            epoch=QDateTime::fromString(epochString, "yyyy M d h m s.zzz");

        } else if (line.left(1)=="P"){ //this is a position line
            satName=line.mid(0,4).trimmed();
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
