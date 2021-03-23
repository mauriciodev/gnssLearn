#include "spp.h"

spp::spp()
{

}

Vector3d spp::runSPP(QString rinexObsFile, QString rinexNavFile, QDateTime epoch)
{
    this->readingDriver.readNavFile(rinexNavFile);
    this->readingDriver.readObsFile(rinexObsFile);



    //let's remember which satellites are going to be used so that we don't have to search for the pairs over and over.
    auto sats=this->readingDriver.getObsSatellitesOnEpoch(epoch);
    int nsats=sats.length();
    auto clkVector=this->readingDriver.getClockBiasVector(sats,epoch);
    auto satPosMatrix=this->readingDriver.getSatPositionMatrix(sats,epoch);
    auto C1Vector=this->readingDriver.getC1Vector(sats,epoch);
    //TODO: nobs is going to grow when we have more codes and frequencies;
    int nobs=nsats;
    //TODO: read X0 from rinex
    Vector3d X0 =this->readingDriver.getX0();

    //Let's begin the computation
    double cdt_r=0;
    int i=0; //iterations
    double dX0X1=1; //initialing X1 <> X0 so that we enter the while loop
    Vector3d X1=X0;
    MatrixXd P, A;
    //TODO: compute the deviation for each sattelite
    P=MatrixXd::Identity(nobs, nobs);

    A.resize(nobs,4);
    A.col(3)=MatrixXd::Constant(nobs,1,1);

    VectorXd Lb(nobs);
    Lb=C1Vector.replicate(1,1); //TODO: replicate once for every observable
    VectorXd dt(nobs);
    dt=clkVector.replicate(1,1); //TODO: replicate once for every observable
    double c=LIGHTSPEED;
    MatrixXd MVC_L;
    while (dX0X1 > 0.1 and (i<20)){
        i+=1;
        X0=X1;
        auto X0m=X0.transpose().replicate(nsats,1);

        if (i==1) {
            P = MatrixXd::Identity(nobs, nobs); //TODO:* 0.8;*sin(2.*PI*elev[:,0]/360);
        } else {
            P=MVC_L;
        }

        //Computing PseudoDistance from X0 to every satelite
        MatrixXd DX0=X0m-satPosMatrix;
        //TODO: Something might be wrong with the precision here. Long double might be needed.
        //cout<<DX0<<endl;
        MatrixXd rho0=DX0.rowwise().squaredNorm().cwiseSqrt();
        //cout<<rho0<<endl;
        A.block(0,0,nobs,3)=DX0.cwiseQuotient(rho0.replicate(1,3));
        //cout<<A<<endl;
        //element-wise power then sum the line

        VectorXd L=Lb-rho0+c*dt-VectorXd::Constant(nobs,1,cdt_r); //- or + ?

        //Least Squares
        MatrixXd AtPA=A.transpose() * P * A;
        VectorXd X_fit=(AtPA).ldlt().solve(A.transpose() * P * L);
        X1=X0+X_fit.segment(0,3);
        cdt_r=X_fit[3];

        VectorXd V=L-A * X_fit;
        double sigma2=(V.transpose() * P * V)[0];
        sigma2/= (nsats-4.);
        MatrixXd MVC_X=sigma2*AtPA.inverse();
        MVC_L=A * MVC_X * A.transpose();
        cout<< "Estimated XYZ: " << fixed << X1.transpose() << " dt_r: "<< cdt_r/c <<endl;
        dX0X1=X_fit.segment(0,3).norm();


    }
    cout<< fixed << (X1-this->readingDriver.getX0()).norm()<<endl;;
    return X1;
}

MatrixXd spp::getObservationMatrix(QStringList sats, QDateTime epoch)
{
    MatrixXd m;
    //listing the satellites used on the given epoch

    m.resize(sats.length(),4);
    // TODO: Measures should be read only once, but the observation matrix should change every iteration
    //getting their C1 vector -
    auto C1Vector=this->readingDriver.getC1Vector(sats,epoch);

    return m;
    //TODO: use other measures. For now, lets go with C1

}
