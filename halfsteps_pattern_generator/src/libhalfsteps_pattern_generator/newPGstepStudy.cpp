//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include <cassert>
#include "newPGstepStudy.h"

#define DELAY_1 0.005
#define DELAY_2 0.2

using namespace std;

CnewPGstepStudy::CnewPGstepStudy()
{

}

CnewPGstepStudy::~CnewPGstepStudy()
{

}

void CnewPGstepStudy::drawSeqStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable, double coefFeet)
{

  double downBound, upBound, leftBound, rightBound;

  StepFeatures stepF;
  produceSeqStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, t4, t5, vect_input, leftOrRightFootStable);

  double centre_x;
  double centre_y;
  double abs_orientation;

  for(int i=0; i < (int) (vect_input.size()-6)/4+2 ; i++)
    {

      double abs_orientationRAD;

      if(i==0) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2] * PI/180;

	leftBound = -centre_y;
	rightBound = -centre_y;
	upBound = centre_x;
	downBound = centre_x;
      }
      else if(i==1) {
	centre_x = vect_input[3];
	centre_y = vect_input[4];
	abs_orientationRAD = vect_input[5] * PI/180;
      }
      else if(i==2) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2];
      }

      if(i>=2){

	abs_orientationRAD = abs_orientation * PI/180;

	centre_x = centre_x + cos(abs_orientationRAD)*vect_input[4*i-1]-sin(abs_orientationRAD)*vect_input[4*i];
	centre_y = centre_y + sin(abs_orientationRAD)*vect_input[4*i-1]+cos(abs_orientationRAD)*vect_input[4*i];
	abs_orientation = abs_orientation + vect_input[4*i+1];

	abs_orientationRAD = abs_orientation * PI/180;

      }

      leftBound = min(leftBound,-centre_y-0.24*coefFeet);
      rightBound = max(rightBound,-centre_y+0.24*coefFeet);
      downBound = min(downBound,centre_x-0.24*coefFeet);
      upBound = max(upBound,centre_x+0.24*coefFeet);

      vector<double> cosinuss (4, 0);
      vector<double> sinuss (4, 0);
      vector<double> x (4, 0);
      vector<double> y (4, 0);

      cosinuss[0] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*0.065;
      sinuss[0] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*0.115;

      cosinuss[1] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*0.065;
      sinuss[1] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*(-0.115);

      cosinuss[2] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*(-0.065);
      sinuss[2] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*(-0.115);

      cosinuss[3] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*(-0.065);
      sinuss[3] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*0.115;

      for(int j = 0; j<4; j++)
	{
	  x[j] = centre_x + cosinuss[j]*coefFeet;
	  y[j] = centre_y + sinuss[j]*coefFeet;
	}

      for(int j = 0; j<4; j++)
	{
	  fb << -y[j]
	     << " " << x[j]
	     << " " << -y[(j+1) % 4] + y[j]
	     << " " << x[(j+1) % 4] - x[j]
	     << endl;
	}
      fb << -y[0]
	 << " " << +x[0]
	 << " " << -y[0]
	 << " " << +x[0]
	 << endl << endl;

    }

  double squareCenter_h = (rightBound + leftBound)/2;
  double squareCenter_v = (upBound + downBound)/2;
  double halfSide = max((rightBound - leftBound)/2,(upBound - downBound)/2);

  leftBound = squareCenter_h - halfSide;
  rightBound = squareCenter_h + halfSide;
  downBound = squareCenter_v - halfSide;
  upBound =squareCenter_v + halfSide;

  //we plot a big rectangle which contains the whole track:
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << upBound
     << endl;
  fb << leftBound
     << " " << upBound
     << " " << rightBound
     << " " << upBound
     << endl;
  fb << rightBound
     << " " << upBound
     << " " << rightBound
     << " " << downBound
     << endl;
  fb << rightBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl;
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl << endl;

  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.rightfootYtraj[i]
	 << " " << stepF.rightfootXtraj[i]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl;
      fb << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.leftfootYtraj[i]
	 << " " << stepF.leftfootXtraj[i]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl;
      fb << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.comTrajY[i]
	 << " " << stepF.comTrajX[i]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl;
      fb << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.zmpTrajY[i]
	 << " " << stepF.zmpTrajX[i]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl;
      fb << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;


  ofstream fb_com("com.dat");
  ofstream fb_zmp("zmp.dat");
  ofstream fb_la("left-ankle.dat");
  ofstream fb_ra("right-ankle.dat");

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb_com << stepF.comTrajX[i]
	     << " " << stepF.comTrajY[i] << endl;

      fb_zmp << stepF.zmpTrajX[i]
	     << " " << stepF.zmpTrajY[i] << endl;

      fb_la << stepF.leftfootXtraj[i]
	    << " " << stepF.leftfootYtraj[i]
	    << " " << stepF.leftfootHeight[i]
	    << " " << stepF.leftfootOrient[i] << endl;

      fb_ra << stepF.rightfootXtraj[i]
	    << " " << stepF.rightfootYtraj[i]
	    << " " << stepF.rightfootHeight[i]
	    << " " << stepF.rightfootOrient[i] << endl;
    }
}

void CnewPGstepStudy::drawSeqHalfStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable, double coefFeet)
{

  StepFeatures stepF;
  produceSeqHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  double downBound, upBound, leftBound, rightBound;

  double centre_x;
  double centre_y;
  double abs_orientation;

  for(int i=0; i < (int) (vect_input.size()-6)/5+2 ; i++)
    {

      double abs_orientationRAD;

      if(i==0) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2] * PI/180;

	leftBound = -centre_y;
	rightBound = -centre_y;
	upBound = centre_x;
	downBound = centre_x;
      }
      else if(i==1) {
	centre_x = vect_input[3];
	centre_y = vect_input[4];
	abs_orientationRAD = vect_input[5] * PI/180;
      }
      else if(i==2) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2];
      }

      if(i>=2){

	abs_orientationRAD = abs_orientation * PI/180;

	centre_x = centre_x + cos(abs_orientationRAD)*vect_input[5*i-2]-sin(abs_orientationRAD)*vect_input[5*i-1];
	centre_y = centre_y + sin(abs_orientationRAD)*vect_input[5*i-2]+cos(abs_orientationRAD)*vect_input[5*i-1];
	abs_orientation = abs_orientation + vect_input[5*i];

	abs_orientationRAD = abs_orientation * PI/180;

      }

      leftBound = min(leftBound,-centre_y-0.24*coefFeet);
      rightBound = max(rightBound,-centre_y+0.24*coefFeet);
      downBound = min(downBound,centre_x-0.24*coefFeet);
      upBound = max(upBound,centre_x+0.24*coefFeet);

      vector<double> cosinuss (4, 0);
      vector<double> sinuss (4, 0);
      vector<double> x (4, 0);
      vector<double> y (4, 0);

      cosinuss[0] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*0.065;
      sinuss[0] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*0.115;

      cosinuss[1] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*0.065;
      sinuss[1] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*(-0.115);

      cosinuss[2] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*(-0.065);
      sinuss[2] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*(-0.115);

      cosinuss[3] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*(-0.065);
      sinuss[3] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*0.115;

      for(int j = 0; j<4; j++)
	{
	  x[j] = centre_x + cosinuss[j]*coefFeet;
	  y[j] = centre_y + sinuss[j]*coefFeet;
	}

      for(int j = 0; j<4; j++)
	{
	  fb << -y[j]
	     << " " << x[j]
	     << " " << -y[(j+1) % 4] + y[j]
	     << " " << x[(j+1) % 4] - x[j]
	     << endl;
	}
      fb << -y[0]
	 << " " << +x[0]
	 << " " << -y[0]
	 << " " << +x[0]
	 << endl << endl;
    }

  double squareCenter_h = (rightBound + leftBound)/2;
  double squareCenter_v = (upBound + downBound)/2;
  double halfSide = max((rightBound - leftBound)/2,(upBound - downBound)/2);

  leftBound = squareCenter_h - halfSide;
  rightBound = squareCenter_h + halfSide;
  downBound = squareCenter_v - halfSide;
  upBound =squareCenter_v + halfSide;

  //we plot a big rectangle which contains the whole track:
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << upBound
     << endl;
  fb << leftBound
     << " " << upBound
     << " " << rightBound
     << " " << upBound
     << endl;
  fb << rightBound
     << " " << upBound
     << " " << rightBound
     << " " << downBound
     << endl;
  fb << rightBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl;
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl << endl;

  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.rightfootYtraj[i]
	 << " " << stepF.rightfootXtraj[i]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl;
      fb << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.leftfootYtraj[i]
	 << " " << stepF.leftfootXtraj[i]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl;
      fb << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.comTrajY[i]
	 << " " << stepF.comTrajX[i]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl;
      fb << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.zmpTrajY[i]
	 << " " << stepF.zmpTrajX[i]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl;
      fb << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;

  ofstream fb_com("com.dat");
  ofstream fb_zmp("zmp.dat");
  ofstream fb_la("left-ankle.dat");
  ofstream fb_ra("right-ankle.dat");

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb_com << stepF.comTrajX[i]
	     << " " << stepF.comTrajY[i] << endl;

      fb_zmp << stepF.zmpTrajX[i]
	     << " " << stepF.zmpTrajY[i] << endl;

      fb_la << stepF.leftfootXtraj[i]
	    << " " << stepF.leftfootYtraj[i]
	    << " " << stepF.leftfootHeight[i]
	    << " " << stepF.leftfootOrient[i] << endl;

      fb_ra << stepF.rightfootXtraj[i]
	    << " " << stepF.rightfootYtraj[i]
	    << " " << stepF.rightfootHeight[i]
	    << " " << stepF.rightfootOrient[i] << endl;
    }
}

void CnewPGstepStudy::drawSeqSlidedHalfStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable, double coefFeet)
{

  StepFeatures stepF;
  produceSeqSlidedHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  double downBound, upBound, leftBound, rightBound;

  double centre_x;
  double centre_y;
  double abs_orientation;

  for(int i=0; i < (int) (vect_input.size()-6)/7+2 ; i++)
    {

      double abs_orientationRAD;

      if(i==0) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2] * PI/180;

	leftBound = -centre_y;
	rightBound = -centre_y;
	upBound = centre_x;
	downBound = centre_x;
      }
      else if(i==1) {
	centre_x = vect_input[3];
	centre_y = vect_input[4];
	abs_orientationRAD = vect_input[5] * PI/180;
      }
      else if(i==2) {
	centre_x = vect_input[0];
	centre_y = vect_input[1];
	abs_orientation = vect_input[2];
      }

      if(i>=2){

	abs_orientationRAD = abs_orientation * PI/180;

	centre_x = centre_x + cos(abs_orientationRAD)*vect_input[7*i-4]-sin(abs_orientationRAD)*vect_input[7*i-3];
	centre_y = centre_y + sin(abs_orientationRAD)*vect_input[7*i-4]+cos(abs_orientationRAD)*vect_input[7*i-3];
	abs_orientation = abs_orientation + vect_input[7*i-2];

	abs_orientationRAD = abs_orientation * PI/180;

      }

      leftBound = min(leftBound,-centre_y-0.24*coefFeet);
      rightBound = max(rightBound,-centre_y+0.24*coefFeet);
      downBound = min(downBound,centre_x-0.24*coefFeet);
      upBound = max(upBound,centre_x+0.24*coefFeet);

      vector<double> cosinuss (4, 0);
      vector<double> sinuss (4, 0);
      vector<double> x (4, 0);
      vector<double> y (4, 0);

      cosinuss[0] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*0.065;
      sinuss[0] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*0.115;

      cosinuss[1] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*0.065;
      sinuss[1] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*(-0.115);

      cosinuss[2] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*(-0.065);
      sinuss[2] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*(-0.115);

      cosinuss[3] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*(-0.065);
      sinuss[3] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*0.115;

      for(int j = 0; j<4; j++)
	{
	  x[j] = centre_x + cosinuss[j]*coefFeet;
	  y[j] = centre_y + sinuss[j]*coefFeet;
	}

      for(int j = 0; j<4; j++)
	{
	  fb << -y[j]
	     << " " << x[j]
	     << " " << -y[(j+1) % 4] + y[j]
	     << " " << x[(j+1) % 4] - x[j]
	     << endl;
	}
      fb << -y[0]
	 << " " << +x[0]
	 << " " << -y[0]
	 << " " << +x[0]
	 << endl << endl;
    }

  double squareCenter_h = (rightBound + leftBound)/2;
  double squareCenter_v = (upBound + downBound)/2;
  double halfSide = max((rightBound - leftBound)/2,(upBound - downBound)/2);

  leftBound = squareCenter_h - halfSide;
  rightBound = squareCenter_h + halfSide;
  downBound = squareCenter_v - halfSide;
  upBound =squareCenter_v + halfSide;

  //we plot a big rectangle which contains the whole track:
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << upBound
     << endl;
  fb << leftBound
     << " " << upBound
     << " " << rightBound
     << " " << upBound
     << endl;
  fb << rightBound
     << " " << upBound
     << " " << rightBound
     << " " << downBound
     << endl;
  fb << rightBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl;
  fb << leftBound
     << " " << downBound
     << " " << leftBound
     << " " << downBound
     << endl << endl;

  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.rightfootYtraj[i]
	 << " " << stepF.rightfootXtraj[i]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl;
      fb << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << " " << -stepF.rightfootYtraj[i+1]
	 << " " << stepF.rightfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.leftfootYtraj[i]
	 << " " << stepF.leftfootXtraj[i]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl;
      fb << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << " " << -stepF.leftfootYtraj[i+1]
	 << " " << stepF.leftfootXtraj[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.comTrajY[i]
	 << " " << stepF.comTrajX[i]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl;
      fb << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << " " << -stepF.comTrajY[i+1]
	 << " " << stepF.comTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;
  //After 3 endl, new index in gnuplot.

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb << -stepF.zmpTrajY[i]
	 << " " << stepF.zmpTrajX[i]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl;
      fb << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << " " << -stepF.zmpTrajY[i+1]
	 << " " << stepF.zmpTrajX[i+1]
	 << endl << endl;
    }
  fb << endl;

  ofstream fb_com("com.dat");
  ofstream fb_zmp("zmp.dat");
  ofstream fb_la("left-ankle.dat");
  ofstream fb_ra("right-ankle.dat");

  for(int i=0; i < stepF.size-1 ; i++)
    {
      fb_com << stepF.comTrajX[i]
	     << " " << stepF.comTrajY[i] << endl;

      fb_zmp << stepF.zmpTrajX[i]
	     << " " << stepF.zmpTrajY[i] << endl;

      fb_la << stepF.leftfootXtraj[i]
	    << " " << stepF.leftfootYtraj[i]
	    << " " << stepF.leftfootHeight[i]
	    << " " << stepF.leftfootOrient[i] << endl;

      fb_ra << stepF.rightfootXtraj[i]
	    << " " << stepF.rightfootYtraj[i]
	    << " " << stepF.rightfootHeight[i]
	    << " " << stepF.rightfootOrient[i] << endl;
    }
}

void CnewPGstepStudy::plotOneDimensionCOMZMPSeqStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable) {

  StepFeatures stepF;
  produceSeqStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, t4, t5, vect_input, leftOrRightFootStable);

  if(whichDimension=='x') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
  else if(whichDimension=='y') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
}

void CnewPGstepStudy::plotFootHeightSeqStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable) {

  StepFeatures stepF;
  produceSeqStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, t4, t5, vect_input, leftOrRightFootStable);

  if(whichDimension=='L') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.leftfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
  else if(whichDimension=='R') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.rightfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
}

void CnewPGstepStudy::plotOneDimensionCOMZMPSeqHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable){

  StepFeatures stepF;
  produceSeqHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  if(whichDimension=='x') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
  else if(whichDimension=='y') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
}

void CnewPGstepStudy::plotFootHeightSeqHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable){

  StepFeatures stepF;
  produceSeqHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  if(whichDimension=='L') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.leftfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
  else if(whichDimension=='R') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.rightfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
}

void CnewPGstepStudy::plotOneDimensionCOMZMPSeqSlidedHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable){

  StepFeatures stepF;
  produceSeqSlidedHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  if(whichDimension=='x') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajX[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
  else if(whichDimension=='y') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.comTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.zmpTrajY[i] << endl;
      }
    fb << endl << endl;
    //After 3 endl, new index in gnuplot.

  }
}

void CnewPGstepStudy::plotFootHeightSeqSlidedHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable){

  StepFeatures stepF;
  produceSeqSlidedHalfStepFeatures(stepF, incrTime, zc, g, t1, t2, t3, vect_input, leftOrRightFootStable);

  if(whichDimension=='L') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.leftfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
  else if(whichDimension=='R') {

    for(int i=0; i < stepF.size-1 ; i++)
      {
	fb << (double) i*incrTime << "  " << stepF.rightfootHeight[i] << endl;
      }
    fb << endl << endl;

  }
}




double w (double t, double g, double zc, double delta0, double deltaX, double t1, double t2, double V, double W)
{

  return(delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t-t1))+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc
																		 )+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
																											  sqrt(g/zc)*(t-t1))/sqrt(g/zc)-2.0*deltaX*pow(t-t1,3.0)/pow(t2-t1,3.0)+3.0*
	 deltaX*pow(t-t1,2.0)/pow(t2-t1,2.0)-12.0*deltaX*zc*(t-t1)/pow(t2-t1,3.0
								       )/g+6.0*deltaX*zc/pow(t2-t1,2.0)/g);

};

double w2 (double t, double g, double zc, double deltaX2, double t2, double t3, double t4, double K2, double V2, double W2)
{

  return(K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
	     deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t-t3))+(V2*sinh(sqrt(g/zc)*(
										       t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/
								   pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t-t3))/sqrt(g/zc)-2.0*deltaX2*pow(t-t3,
																	3.0)/pow(t4-t3,3.0)+3.0*deltaX2*pow(t-t3,2.0)/pow(t4-t3,2.0)-12.0*deltaX2
	 *zc*(t-t3)/pow(t4-t3,3.0)/g+6.0*deltaX2*zc/pow(t4-t3,2.0)/g);

};

double u (double t, double g, double zc, double t2, double K2, double V2, double W2)
{

  return(V2*cosh(sqrt(g/zc)*(t-t2))+W2*sinh(sqrt(g/zc)*(t-t2))+K2);

};

double u2 (double t, double g, double zc, double t4, double K3, double V3, double W3)
{

  return(V3*cosh(sqrt(g/zc)*(t-t4))+W3*sinh(sqrt(g/zc)*(t-t4))+K3);

};

double hZMP (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
{

  if(t <= t1)
    {
      return delta0;
    }
  else if(t <= t2)
    {
      return (delta0*t1*t1*t1-delta0*t2*t2*t2+2.0*deltaX*t*t*t+deltaX*t1*t1*t1-3.0*deltaX*t*t*t1-3.0*deltaX*t*t*t2+6.0*deltaX*t*t1*t2
	      -3.0*delta0*t1*t1*t2+3.0*delta0*t1*t2*t2-3.0*deltaX*t1*t1*t2)/pow(t1-
										t2,3.0);
    }
  else if(t <= t3)
    {
      return K2;
    }
  else if(t <= t4)
    {
      return (-K2*t4*t4*t4+K2*t3*t3*t3+2.0*deltaX2*t*t*t+deltaX2*t3*t3*t3
	      -3.0*deltaX2*t*t*t3-3.0*deltaX2*t*t*t4+6.0*deltaX2*t*t3*t4-3.0*
	      deltaX2*t4*t3*t3+3.0*K2*t4*t4*t3-3.0*K2*t4*t3*t3)/pow(-t4+t3,3.0);
    }
  else
    {
      return K3;
    }

};

double h (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
{

  if(t <= t1)
    {
      return V*cosh(sqrt(g/zc)*t)+W*sinh(sqrt(g/zc)*t)+delta0;
    }
  else if(t <= t2)
    {
      return w(t, g, zc, delta0, deltaX, t1, t2, V, W);
    }
  else if(t <= t3)
    {
      return u(t, g, zc, t2, K2, V2, W2);
    }
  else if(t <= t4)
    {
      return w2(t, g, zc, deltaX2, t2, t3, t4, K2, V2, W2);
    }
  else
    {
      return u2(t, g, zc, t4, K3, V3, W3);
    }

};

vector<double> hVinit (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

  vector<double> PairToReturn;
  double V = pinit-delta0;
  double W = vinit/sqrt(g/zc);
  double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		      *deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
										    )*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
									     /g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
																				  cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
																				 cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
																													     zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
																																				      )*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
  double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		     /pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
												  zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
																					     sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
  double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
									      t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
																			    /zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
	       (sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
  double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
						      )-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
																/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
														   *zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
																											    V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
																																				       t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
																																														     /zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
																																										    g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
  double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		     deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
											      zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
										 *zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
																			   t4-t3,2.0));
  double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
										       zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
									  *zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
																	     3.0)/g)/sqrt(g/zc);
  PairToReturn.push_back(h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
  PairToReturn.push_back(hZMP(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
  return PairToReturn;
};

double hVinitCOMonly (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

  double V = pinit-delta0;
  double W = vinit/sqrt(g/zc);
  double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		      *deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
										    )*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
									     /g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
																				  cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
																				 cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
																													     zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
																																				      )*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
  double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		     /pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
												  zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
																					     sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
  double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
									      t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
																			    /zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
	       (sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
  double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
						      )-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
																/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
														   *zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
																											    V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
																																				       t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
																																														     /zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
																																										    g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
  double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		     deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
											      zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
										 *zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
																			   t4-t3,2.0));
  double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
										       zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
									  *zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
																	     3.0)/g)/sqrt(g/zc);
  return h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3);
};

double searchVinit (double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit)
{

  double vinitBmin = -10.0;

  double vinitBmax = 10.0;

  if (
      hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmin) >= delta0 + deltaX + deltaX2
      ||
      hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmax) <= delta0 + deltaX + deltaX2
      ) return -999;

  while (vinitBmax - vinitBmin > 0.00000001)
    {

      if (hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, (vinitBmax + vinitBmin) / 2) > delta0 + deltaX + deltaX2)
	{
	  vinitBmax = (vinitBmax + vinitBmin) / 2;
	}
      else
	{
	  vinitBmin = (vinitBmax + vinitBmin) / 2;
	}

    }

  return (vinitBmax + vinitBmin) / 2;

}


// void CnewPGstepStudy::plotOneDimensionCOMtrajectory(ofstream & fb, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5)
// {
//
// 	double vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);
//
// 	for(double i = 0.0 ; i < t5 ; i += incrTime)
// 	{
//
// 		fb << i << " " << hVinitCOMonly(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit) << endl;
// 	}
//
// }


void CnewPGstepStudy::genCOMZMPtrajectory(vector<double> & outputCOM, vector<double> & outputZMP, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5)
{

  double sensitivityLimit = 0.00001; //because of the instability of the formula.

  outputCOM.clear();
  outputZMP.clear();

  double vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);

  if(abs(deltaX) < sensitivityLimit && abs(deltaX2) < sensitivityLimit) {
    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {
	outputCOM.push_back(delta0);
	outputZMP.push_back(delta0);
      }
  }
  else {

    int count = 0;
    int countSav = 0;
    //double minVal = 99999999;
    double valPrev = 99999999;
    double valTmp;
    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	//fb << i << " " << hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, vinit) << endl;
	vector<double> ComZmp = hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit);

	outputCOM.push_back(ComZmp[0]);
	outputZMP.push_back(ComZmp[1]);

	valTmp = abs(ComZmp[0] - delta0 - deltaX - deltaX2);
	if(valTmp < valPrev) {
	  countSav = count;
	}
	valPrev = valTmp;
	count++;
      }

    //again, due to the instability of the formula:
    if(countSav != outputCOM.size()-1) {
      for(unsigned int i = countSav ; i < outputCOM.size() ; i++)
	{
	  outputCOM[i] = (outputCOM[countSav]*(outputCOM.size()-1-i) + (delta0 + deltaX + deltaX2)*(i-countSav))/(outputCOM.size()-1-countSav);
	}
    }



  }

}


void CnewPGstepStudy::genFOOTposition(vector<double> & outputX, vector<double> & outputY, double incrTime, double xinit, double yinit, double xend, double yend, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

  if(du == '2') {

    outputX.clear();
    outputY.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    outputX.push_back(xinit);
	    outputY.push_back(yinit);

	  }
	else if(i < t3-delay)
	  {

	    outputX.push_back(xinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(xend-xinit));
	    outputY.push_back(yinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(yend-yinit));

	  }
	else
	  {

	    outputX.push_back(xend);
	    outputY.push_back(yend);

	  }

      }

  }

  if(du == 'd') {

    outputX.clear();
    outputY.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2)
	  {

	    outputX.push_back(xinit);
	    outputY.push_back(yinit);

	  }
	else if(i < t3-delay)
	  {

	    outputX.push_back(xinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(xend-xinit));
	    outputY.push_back(yinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(yend-yinit));

	  }
	else
	  {

	    outputX.push_back(xend);
	    outputY.push_back(yend);

	  }

      }

  }

  if(du == 'u') {

    outputX.clear();
    outputY.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    outputX.push_back(xinit);
	    outputY.push_back(yinit);

	  }
	else if(i < t3)
	  {

	    outputX.push_back(xinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(xend-xinit));
	    outputY.push_back(yinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(yend-yinit));

	  }
	else
	  {

	    outputX.push_back(xend);
	    outputY.push_back(yend);

	  }

      }

  }

}

void CnewPGstepStudy::genFOOTheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3, double t4, double t5)
{

  output.clear();

  for(double i = 0.0 ; i < t5 ; i += incrTime)
    {

      if(i < t2+delay)
	{

	  output.push_back(0);

	}
      else if(i < t3-delay)
	{

	  output.push_back( 16*heightMax/pow(t3-t2-2*delay,4.0)*pow(i-t2-delay,4.0)  -  32*heightMax/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)  +  16*heightMax/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0));

	}
      else
	{

	  output.push_back(0);

	}

    }

}

void CnewPGstepStudy::genFOOTdownUPheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3)
{

  output.clear();

  for(double i = 0.0 ; i < t3 ; i += incrTime)
    {

      if(i < t2+delay)
	{

	  output.push_back(0);

	}
      else if(i < t3-delay)
	{

	  output.push_back( -2*heightMax/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3*heightMax/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0));

	}
      else
	{

	  output.push_back(heightMax);

	}

    }

}

void CnewPGstepStudy::genFOOTupDOWNheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3)
{

  output.clear();

  for(double i = 0.0 ; i < t3 ; i += incrTime)
    {
      if(i < delay) {
	output.push_back(heightMax);
      } else if(i < t1-delay)
	{

	  output.push_back( -2*heightMax/pow(delay-t1+delay,3.0)*pow(i-t1+delay,3.0)  +  3*heightMax/pow(delay-t1+delay,2.0)*pow(i-t1+delay,2.0));

	}
      else
	{

	  output.push_back(0);

	}

    }

}

void CnewPGstepStudy::genFOOTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

  if(du == '2') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3-delay)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }

  if(du == 'd') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3-delay)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }

  if(du == 'u') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }

}


void CnewPGstepStudy::genWAISTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

  if(du == '2') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3-delay)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }
  if(du == 'd') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3-delay)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }
  if(du == 'u') {

    output.clear();

    for(double i = 0.0 ; i < t5 ; i += incrTime)
      {

	if(i < t2+delay)
	  {

	    output.push_back(initOrient);

	  }
	else if(i < t3)
	  {

	    output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );

	  }
	else
	  {

	    output.push_back(endOrient);

	  }

      }

  }


}


void CnewPGstepStudy::addStepFeaturesWithSlide(
					       StepFeatures & stepF1,
					       StepFeatures & stepF2,
					       double negativeSlideTime
					       )
{

  //PHASE 1: modify stepF2 according to the change of origin
  double lastwaistX = stepF1.comTrajX[stepF1.size - 1];
  double lastwaistY = stepF1.comTrajY[stepF1.size - 1];
  double radlastwaistOrient = stepF1.waistOrient[stepF1.size - 1]*PI/180;

  for(unsigned int count = 0 ; count < stepF2.size ; count++) {

    /*	double newcomX = (stepF2.comTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
	-(stepF2.comTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newcomY = (stepF2.comTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
	+(stepF2.comTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newzmpX = (stepF2.zmpTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
	-(stepF2.zmpTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newzmpY = (stepF2.zmpTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
	+(stepF2.zmpTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newlfX = (stepF2.leftfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
	-(stepF2.leftfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);
	double newlfY = (stepF2.leftfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)
	+(stepF2.leftfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);
	double newrfX = (stepF2.rightfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
	-(stepF2.rightfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);
	double newrfY = (stepF2.rightfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)
	+(stepF2.rightfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);*/

    double newcomX = (stepF2.comTrajX[count])*cos(radlastwaistOrient)
      -(stepF2.comTrajY[count])*sin(radlastwaistOrient)+lastwaistX;
    double newcomY = (stepF2.comTrajX[count])*sin(radlastwaistOrient)
      +(stepF2.comTrajY[count])*cos(radlastwaistOrient)+lastwaistY;
    double newzmpX = (stepF2.zmpTrajX[count])*cos(radlastwaistOrient)
      -(stepF2.zmpTrajY[count])*sin(radlastwaistOrient)+lastwaistX;
    double newzmpY = (stepF2.zmpTrajX[count])*sin(radlastwaistOrient)
      +(stepF2.zmpTrajY[count])*cos(radlastwaistOrient)+lastwaistY;
    double newlfX = (stepF2.leftfootXtraj[count])*cos(radlastwaistOrient)
      -(stepF2.leftfootYtraj[count])*sin(radlastwaistOrient)+lastwaistX;
    double newlfY = (stepF2.leftfootXtraj[count])*sin(radlastwaistOrient)
      +(stepF2.leftfootYtraj[count])*cos(radlastwaistOrient)+lastwaistY;
    double newrfX = (stepF2.rightfootXtraj[count])*cos(radlastwaistOrient)
      -(stepF2.rightfootYtraj[count])*sin(radlastwaistOrient)+lastwaistX;
    double newrfY = (stepF2.rightfootXtraj[count])*sin(radlastwaistOrient)
      +(stepF2.rightfootYtraj[count])*cos(radlastwaistOrient)+lastwaistY;

    stepF2.comTrajX[count] = newcomX;
    stepF2.zmpTrajX[count] = newzmpX;

    stepF2.comTrajY[count] = newcomY;
    stepF2.zmpTrajY[count] = newzmpY;

    stepF2.leftfootXtraj[count] = newlfX;
    stepF2.leftfootYtraj[count] = newlfY;

    stepF2.leftfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

    stepF2.rightfootXtraj[count] = newrfX;
    stepF2.rightfootYtraj[count] = newrfY;

    stepF2.rightfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

    stepF2.waistOrient[count] += stepF1.waistOrient[stepF1.size - 1];

  }

  int delayInt = (int) (abs(negativeSlideTime)/stepF2.incrTime);

  //PHASE 2: add the new stepF2 to stepF1
  for(unsigned int count = 0 ; count < stepF2.size ; count++) {

    if(count < delayInt) {

      stepF1.comTrajX[stepF1.size - delayInt + count] =
	(stepF1.comTrajX[stepF1.size - delayInt + count] + stepF2.comTrajX[count])
	-stepF1.comTrajX[stepF1.size - 1];
      stepF1.zmpTrajX[stepF1.size - delayInt + count] =
	(stepF1.zmpTrajX[stepF1.size - delayInt + count] + stepF2.zmpTrajX[count])
	-stepF1.zmpTrajX[stepF1.size - 1];

      stepF1.comTrajY[stepF1.size - delayInt + count] =
	( stepF1.comTrajY[stepF1.size - delayInt + count] + stepF2.comTrajY[count])
	-stepF1.comTrajY[stepF1.size - 1];
      stepF1.zmpTrajY[stepF1.size - delayInt + count] =
	( stepF1.zmpTrajY[stepF1.size - delayInt + count] + stepF2.zmpTrajY[count])
	-stepF1.zmpTrajY[stepF1.size - 1];

      stepF1.leftfootXtraj[stepF1.size - delayInt + count] =
	( stepF1.leftfootXtraj[stepF1.size - delayInt + count] + stepF2.leftfootXtraj[count])
	-stepF1.leftfootXtraj[stepF1.size - 1];
      stepF1.leftfootYtraj[stepF1.size - delayInt + count] =
	( stepF1.leftfootYtraj[stepF1.size - delayInt + count] + stepF2.leftfootYtraj[count])
	-stepF1.leftfootYtraj[stepF1.size - 1];

      stepF1.leftfootOrient[stepF1.size - delayInt + count] =
	( stepF1.leftfootOrient[stepF1.size - delayInt + count] + stepF2.leftfootOrient[count])
	-stepF1.leftfootOrient[stepF1.size - 1];
      stepF1.leftfootHeight[stepF1.size - delayInt + count] =
	(  stepF1.leftfootHeight[stepF1.size - delayInt + count] + stepF2.leftfootHeight[count])
	-stepF1.leftfootHeight[stepF1.size - 1];

      stepF1.rightfootXtraj[stepF1.size - delayInt + count] =
	( stepF1.rightfootXtraj[stepF1.size - delayInt + count] + stepF2.rightfootXtraj[count])
	-stepF1.rightfootXtraj[stepF1.size - 1];
      stepF1.rightfootYtraj[stepF1.size - delayInt + count] =
	( stepF1.rightfootYtraj[stepF1.size - delayInt + count] + stepF2.rightfootYtraj[count])
	-stepF1.rightfootYtraj[stepF1.size - 1];

      stepF1.rightfootOrient[stepF1.size - delayInt + count] =
	( stepF1.rightfootOrient[stepF1.size - delayInt + count] + stepF2.rightfootOrient[count])
	-stepF1.rightfootOrient[stepF1.size - 1];
      stepF1.rightfootHeight[stepF1.size - delayInt + count] =
	( stepF1.rightfootHeight[stepF1.size - delayInt + count] + stepF2.rightfootHeight[count])
	-stepF1.rightfootHeight[stepF1.size - 1];

      stepF1.waistOrient[stepF1.size - delayInt + count] =
	( stepF1.waistOrient[stepF1.size - delayInt + count] + stepF2.waistOrient[count])
	-stepF1.waistOrient[stepF1.size - 1];

    } else {

      stepF1.comTrajX.push_back(stepF2.comTrajX[count]);
      stepF1.zmpTrajX.push_back(stepF2.zmpTrajX[count]);

      stepF1.comTrajY.push_back(stepF2.comTrajY[count]);
      stepF1.zmpTrajY.push_back(stepF2.zmpTrajY[count]);

      stepF1.leftfootXtraj.push_back(stepF2.leftfootXtraj[count]);
      stepF1.leftfootYtraj.push_back(stepF2.leftfootYtraj[count]);

      stepF1.leftfootOrient.push_back(stepF2.leftfootOrient[count]);
      stepF1.leftfootHeight.push_back(stepF2.leftfootHeight[count]);

      stepF1.rightfootXtraj.push_back(stepF2.rightfootXtraj[count]);
      stepF1.rightfootYtraj.push_back(stepF2.rightfootYtraj[count]);

      stepF1.rightfootOrient.push_back(stepF2.rightfootOrient[count]);
      stepF1.rightfootHeight.push_back(stepF2.rightfootHeight[count]);

      stepF1.waistOrient.push_back(stepF2.waistOrient[count]);

    }

  }

  stepF1.size = stepF1.size + stepF2.size - delayInt;

}


void CnewPGstepStudy::produceOneStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable)
{

  double stepHeight = vectStep_input[6];

  vector<double> comTrajX;
  vector<double> zmpTrajX;
  genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, vectStep_input[0], vectStep_input[7]/2, t1, t2, t3, t4, t5);

  vector<double> comTrajY;
  vector<double> zmpTrajY;
  genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, vectStep_input[1], vectStep_input[8]/2, t1, t2, t3, t4, t5);

  vector<double> footXtraj;
  vector<double> footYtraj;
  genFOOTposition(footXtraj, footYtraj, incrTime, vectStep_input[3], vectStep_input[4], vectStep_input[0]+vectStep_input[7], vectStep_input[1]+vectStep_input[8], DELAY_2, t1, t2, t3, t4, t5, '2');

  vector<double> footHeight;
  genFOOTheight(footHeight, incrTime, stepHeight, DELAY_1, t1, t2, t3, t4, t5);

  vector<double> footOrient;
  genFOOTorientation(footOrient, incrTime, vectStep_input[5], vectStep_input[9], DELAY_2, t1, t2, t3, t4, t5, '2');

  vector<double> stablefootXtraj;
  vector<double> stablefootYtraj;
  vector<double> stablefootHeight;
  vector<double> stablefootOrient;

  int count = -1;

  for(double i = 0.0 ; i < t5 ; i += incrTime)
    {
      count++;
      stablefootXtraj.push_back(vectStep_input[0]);
      stablefootYtraj.push_back(vectStep_input[1]);
      stablefootHeight.push_back(0);
      stablefootOrient.push_back(0);
    }

  vector<double> waistOrient;
  genWAISTorientation(waistOrient, incrTime, 0, vectStep_input[9], DELAY_1, t1, t2, t3, t4, t5, '2');


  stepF.comTrajX = comTrajX;
  stepF.zmpTrajX = zmpTrajX;
  stepF.comTrajY = comTrajY;
  stepF.zmpTrajY = zmpTrajY;

  if(leftOrRightFootStable == 'L') {
    stepF.leftfootXtraj = stablefootXtraj;
    stepF.leftfootYtraj = stablefootYtraj;
    stepF.leftfootHeight = stablefootHeight;
    stepF.leftfootOrient = stablefootOrient;
    stepF.rightfootXtraj = footXtraj;
    stepF.rightfootYtraj = footYtraj;
    stepF.rightfootHeight = footHeight;
    stepF.rightfootOrient = footOrient;
  } else {
    stepF.leftfootXtraj = footXtraj;
    stepF.leftfootYtraj = footYtraj;
    stepF.leftfootHeight = footHeight;
    stepF.leftfootOrient = footOrient;
    stepF.rightfootXtraj = stablefootXtraj;
    stepF.rightfootYtraj = stablefootYtraj;
    stepF.rightfootHeight = stablefootHeight;
    stepF.rightfootOrient = stablefootOrient;
  }

  stepF.waistOrient =  waistOrient;
  stepF.incrTime = incrTime;
  stepF.zc = zc;
  stepF.size = waistOrient.size();

}


void CnewPGstepStudy::produceOneUPHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable)
{

  vector<double> comTrajX;
  vector<double> zmpTrajX;
  genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectUPHalfStep_input[0], t1/2, t1*3/4, t1, t2, t3);

  vector<double> comTrajY;
  vector<double> zmpTrajY;
  genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectUPHalfStep_input[1], t1/2, t1*3/4, t1, t2, t3);

  vector<double> footXtraj;
  vector<double> footYtraj;
  int leftRightCoef = 0;
  if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
  genFOOTposition(footXtraj, footYtraj, incrTime, vectUPHalfStep_input[3], vectUPHalfStep_input[4], vectUPHalfStep_input[0], vectUPHalfStep_input[1]+leftRightCoef*vectUPHalfStep_input[6], DELAY_2, t1, t2, t3, t3, t3, 'u');

  vector<double> footHeight;
  genFOOTdownUPheight(footHeight, incrTime, vectUPHalfStep_input[7], DELAY_1, t1, t2, t3);

  vector<double> footOrient;
  genFOOTorientation(footOrient, incrTime, vectUPHalfStep_input[5], 0, DELAY_2, t1, t2, t3, t3, t3, 'u');

  vector<double> stablefootXtraj;
  vector<double> stablefootYtraj;
  vector<double> stablefootHeight;
  vector<double> stablefootOrient;

  int count = -1;

  for(double i = 0.0 ; i < t3 ; i += incrTime)
    {
      count++;
      stablefootXtraj.push_back(vectUPHalfStep_input[0]);
      stablefootYtraj.push_back(vectUPHalfStep_input[1]);
      stablefootHeight.push_back(0);
      stablefootOrient.push_back(0);
    }

  vector<double> waistOrient;
  genWAISTorientation(waistOrient, incrTime, 0, 0, DELAY_1, t1, t2, t3, t3, t3, 'u');


  stepF.comTrajX = comTrajX;
  stepF.zmpTrajX = zmpTrajX;
  stepF.comTrajY = comTrajY;
  stepF.zmpTrajY = zmpTrajY;

  if(leftOrRightFootStable == 'L') {
    stepF.leftfootXtraj = stablefootXtraj;
    stepF.leftfootYtraj = stablefootYtraj;
    stepF.leftfootHeight = stablefootHeight;
    stepF.leftfootOrient = stablefootOrient;
    stepF.rightfootXtraj = footXtraj;
    stepF.rightfootYtraj = footYtraj;
    stepF.rightfootHeight = footHeight;
    stepF.rightfootOrient = footOrient;
  } else {
    stepF.leftfootXtraj = footXtraj;
    stepF.leftfootYtraj = footYtraj;
    stepF.leftfootHeight = footHeight;
    stepF.leftfootOrient = footOrient;
    stepF.rightfootXtraj = stablefootXtraj;
    stepF.rightfootYtraj = stablefootYtraj;
    stepF.rightfootHeight = stablefootHeight;
    stepF.rightfootOrient = stablefootOrient;
  }

  stepF.waistOrient =  waistOrient;
  stepF.incrTime = incrTime;
  stepF.zc = zc;
  stepF.size = waistOrient.size();

}


void CnewPGstepStudy::produceOneDOWNHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable)
{

  vector<double> comTrajX;
  vector<double> zmpTrajX;
  genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[2]/2, t1/2, t1*3/4, t1, t2, t3);

  vector<double> comTrajY;
  vector<double> zmpTrajY;
  genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[3]/2, t1/2, t1*3/4, t1, t2, t3);

  vector<double> footXtraj;
  vector<double> footYtraj;
  int leftRightCoef = 0;
  if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
  genFOOTposition(footXtraj, footYtraj, incrTime, 0, leftRightCoef*vectDOWNHalfStep_input[0], vectDOWNHalfStep_input[2], vectDOWNHalfStep_input[3], DELAY_2, 0, 0, t1, t2, t3, 'd');

  vector<double> footHeight;
  genFOOTupDOWNheight(footHeight, incrTime, vectDOWNHalfStep_input[1], DELAY_1, t1, t2, t3);

  vector<double> footOrient;
  genFOOTorientation(footOrient, incrTime, 0, vectDOWNHalfStep_input[4], DELAY_2, 0, 0, t1, t2, t3, 'd');

  vector<double> waistOrient;
  genWAISTorientation(waistOrient, incrTime, 0, vectDOWNHalfStep_input[4], DELAY_1, 0, 0, t1, t2, t3, 'd');

  vector<double> stablefootXtraj;
  vector<double> stablefootYtraj;
  vector<double> stablefootHeight;
  vector<double> stablefootOrient;

  for(double i = 0.0 ; i < t3 ; i += incrTime)
    {
      stablefootXtraj.push_back(0);
      stablefootYtraj.push_back(0);
      stablefootHeight.push_back(0);
      stablefootOrient.push_back(0);
    }

  stepF.comTrajX = comTrajX;
  stepF.zmpTrajX = zmpTrajX;
  stepF.comTrajY = comTrajY;
  stepF.zmpTrajY = zmpTrajY;

  if(leftOrRightFootStable == 'L') {
    stepF.leftfootXtraj = stablefootXtraj;
    stepF.leftfootYtraj = stablefootYtraj;
    stepF.leftfootHeight = stablefootHeight;
    stepF.leftfootOrient = stablefootOrient;
    stepF.rightfootXtraj = footXtraj;
    stepF.rightfootYtraj = footYtraj;
    stepF.rightfootHeight = footHeight;
    stepF.rightfootOrient = footOrient;
  } else {
    stepF.leftfootXtraj = footXtraj;
    stepF.leftfootYtraj = footYtraj;
    stepF.leftfootHeight = footHeight;
    stepF.leftfootOrient = footOrient;
    stepF.rightfootXtraj = stablefootXtraj;
    stepF.rightfootYtraj = stablefootYtraj;
    stepF.rightfootHeight = stablefootHeight;
    stepF.rightfootOrient = stablefootOrient;
  }

  stepF.waistOrient =  waistOrient;
  stepF.incrTime = incrTime;
  stepF.zc = zc;
  stepF.size = waistOrient.size();

}


void CnewPGstepStudy::produceSeqStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable)
{

  char alternate = leftOrRightFootStable;

  vector<StepFeatures> vectSFeat;

  vector<double> tmpNine;
  tmpNine.resize(10);

  tmpNine[7] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
  tmpNine[8] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
  tmpNine[9] = -vectSteps_input[5];

  for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/4; i++) {

    tmpNine[0] = (tmpNine[7]/2)*cos(-tmpNine[9]*PI/180) - (tmpNine[8]/2)*sin(-tmpNine[9]*PI/180);
    tmpNine[1] = (tmpNine[7]/2)*sin(-tmpNine[9]*PI/180) + (tmpNine[8]/2)*cos(-tmpNine[9]*PI/180);
    tmpNine[2] = 0;
    tmpNine[3] = -tmpNine[0];
    tmpNine[4] = -tmpNine[1];
    tmpNine[5] = -tmpNine[9];
    tmpNine[6] = vectSteps_input[4*i+2];
    tmpNine[7] = vectSteps_input[4*i+3];
    tmpNine[8] = vectSteps_input[4*i+4];
    tmpNine[9] = vectSteps_input[4*i+5];

    vector<vector<double> > tmp_fb;
    vector<vector<double> > tmp_fbZMP;

    StepFeatures tmp_step;

    produceOneStepFeatures(tmp_step, incrTime, zc, g, t1, t2, t3, t4, t5, tmpNine, alternate);

    vectSFeat.push_back(tmp_step);

    if(alternate == 'L') alternate = 'R'; else alternate = 'L';

  }

  for(unsigned int i = 1; i < vectSFeat.size(); i++) {
    addStepFeaturesWithSlide(vectSFeat[0],vectSFeat[i],0); //no slide
  }

  stepF = vectSFeat[0];

}


void CnewPGstepStudy::produceSeqHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{

  char alternate = leftOrRightFootStable;

  vector<StepFeatures> vectSFeat;

  vector<double> tmpEleven;
  tmpEleven.resize(11);

  vector<double> tmpPart1;
  tmpPart1.resize(8);

  vector<double> tmpPart2;
  tmpPart2.resize(5);

  tmpEleven[8] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
  tmpEleven[9] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
  tmpEleven[10] = -vectSteps_input[5];

  for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/5; i++) {

    tmpEleven[0] = (tmpEleven[8]/2)*cos(-tmpEleven[10]*PI/180) - (tmpEleven[9]/2)*sin(-tmpEleven[10]*PI/180);
    tmpEleven[1] = (tmpEleven[8]/2)*sin(-tmpEleven[10]*PI/180) + (tmpEleven[9]/2)*cos(-tmpEleven[10]*PI/180);
    tmpEleven[2] = 0;
    tmpEleven[3] = -tmpEleven[0];
    tmpEleven[4] = -tmpEleven[1];
    tmpEleven[5] = -tmpEleven[10];

    tmpEleven[6] = vectSteps_input[5*i+1];
    tmpEleven[7] = vectSteps_input[5*i+2];

    tmpEleven[8] = vectSteps_input[5*i+3];
    tmpEleven[9] = vectSteps_input[5*i+4];
    tmpEleven[10] = vectSteps_input[5*i+5];

    StepFeatures tmp_hstepUp;
    StepFeatures tmp_hstepDown;

    tmpPart1[0] = tmpEleven[0];
    tmpPart1[1] = tmpEleven[1];
    tmpPart1[2] = tmpEleven[2];
    tmpPart1[3] = tmpEleven[3];
    tmpPart1[4] = tmpEleven[4];
    tmpPart1[5] = tmpEleven[5];
    tmpPart1[6] = tmpEleven[6];
    tmpPart1[7] = tmpEleven[7];

    tmpPart2[0] = tmpEleven[6];
    tmpPart2[1] = tmpEleven[7];
    tmpPart2[2] = tmpEleven[8];
    tmpPart2[3] = tmpEleven[9];
    tmpPart2[4] = tmpEleven[10];

    produceOneUPHalfStepFeatures(tmp_hstepUp, incrTime, zc, g, t1, t2, t3, tmpPart1, alternate);

    vectSFeat.push_back(tmp_hstepUp);

    produceOneDOWNHalfStepFeatures(tmp_hstepDown, incrTime, zc, g, t1, t2, t3, tmpPart2, alternate);

    vectSFeat.push_back(tmp_hstepDown);

    if(alternate == 'L') alternate = 'R'; else alternate = 'L';

  }

  for(unsigned int i = 1; i < vectSFeat.size(); i++) {
    addStepFeaturesWithSlide(vectSFeat[0],vectSFeat[i],0); //no slide
  }

  stepF = vectSFeat[0];

}


void CnewPGstepStudy::produceSeqSlidedHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable)
{

  char alternate = leftOrRightFootStable;

  vector<StepFeatures> vectSFeat;
  vector<double> slideProfile;

  vector<double> tmpEleven;
  tmpEleven.resize(11);

  vector<double> tmpPart1;
  tmpPart1.resize(8);

  vector<double> tmpPart2;
  tmpPart2.resize(5);

  tmpEleven[8] = (vectSteps_input[0]*2)*cos(-vectSteps_input[5]*PI/180)-(vectSteps_input[1]*2)*sin(-vectSteps_input[5]*PI/180);
  tmpEleven[9] = (vectSteps_input[0]*2)*sin(-vectSteps_input[5]*PI/180)+(vectSteps_input[1]*2)*cos(-vectSteps_input[5]*PI/180) ;
  tmpEleven[10] = -vectSteps_input[5];

  for(unsigned int i = 1; i <= (vectSteps_input.size()-6)/7; i++) {

    tmpEleven[0] = (tmpEleven[8]/2)*cos(-tmpEleven[10]*PI/180) - (tmpEleven[9]/2)*sin(-tmpEleven[10]*PI/180);
    tmpEleven[1] = (tmpEleven[8]/2)*sin(-tmpEleven[10]*PI/180) + (tmpEleven[9]/2)*cos(-tmpEleven[10]*PI/180);
    tmpEleven[2] = 0;
    tmpEleven[3] = -tmpEleven[0];
    tmpEleven[4] = -tmpEleven[1];
    tmpEleven[5] = -tmpEleven[10];

    slideProfile.push_back(vectSteps_input[7*i-1]);
    tmpEleven[6] = vectSteps_input[7*i];
    tmpEleven[7] = vectSteps_input[7*i+1];

    slideProfile.push_back(vectSteps_input[7*i+2]);
    tmpEleven[8] = vectSteps_input[7*i+3];
    tmpEleven[9] = vectSteps_input[7*i+4];
    tmpEleven[10] = vectSteps_input[7*i+5];

    StepFeatures tmp_hstepUp;
    StepFeatures tmp_hstepDown;

    tmpPart1[0] = tmpEleven[0];
    tmpPart1[1] = tmpEleven[1];
    tmpPart1[2] = tmpEleven[2];
    tmpPart1[3] = tmpEleven[3];
    tmpPart1[4] = tmpEleven[4];
    tmpPart1[5] = tmpEleven[5];
    tmpPart1[6] = tmpEleven[6];
    tmpPart1[7] = tmpEleven[7];

    tmpPart2[0] = tmpEleven[6];
    tmpPart2[1] = tmpEleven[7];
    tmpPart2[2] = tmpEleven[8];
    tmpPart2[3] = tmpEleven[9];
    tmpPart2[4] = tmpEleven[10];

    produceOneUPHalfStepFeatures(tmp_hstepUp, incrTime, zc, g, t1, t2, t3, tmpPart1, alternate);

    vectSFeat.push_back(tmp_hstepUp);

    produceOneDOWNHalfStepFeatures(tmp_hstepDown, incrTime, zc, g, t1, t2, t3, tmpPart2, alternate);

    vectSFeat.push_back(tmp_hstepDown);

    if(alternate == 'L') alternate = 'R'; else alternate = 'L';

  }

  for(unsigned int i = 1; i < vectSFeat.size(); i++) {
    addStepFeaturesWithSlide(vectSFeat[0],vectSFeat[i],slideProfile[i]);
  }

  assert(!vectSFeat.empty());
  stepF = vectSFeat[0];

}
