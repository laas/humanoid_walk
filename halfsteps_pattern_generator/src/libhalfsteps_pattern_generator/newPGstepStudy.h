/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 *
 *  Convention: m_ for public members, mp_ for private members.
 */

#ifndef newPGstepStudy_H
#define newPGstepStudy_H

#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

#define PI 3.14159265359

using namespace std;

struct StepFeatures
{
  explicit StepFeatures()
  {}

  ~StepFeatures()
  {}

  StepFeatures& operator= (const StepFeatures& sf)
  {
    if (&sf == this)
      return *this;

    comTrajX = sf.comTrajX;
    zmpTrajX = sf.zmpTrajX;
    comTrajY = sf.comTrajY;
    zmpTrajY = sf.zmpTrajY;
    leftfootXtraj = sf.leftfootXtraj;
    leftfootYtraj = sf.leftfootYtraj;
    leftfootHeight = sf.leftfootHeight;
    leftfootOrient = sf.leftfootOrient;
    rightfootXtraj = sf.rightfootXtraj;
    rightfootYtraj = sf.rightfootYtraj;
    rightfootHeight = sf.rightfootHeight;
    rightfootOrient = sf.rightfootOrient;
    waistOrient = sf.waistOrient;
    incrTime = sf.incrTime;
    zc = sf.zc;
    size = sf.size;

    return *this;
  }

  std::vector<double> comTrajX;
  std::vector<double> zmpTrajX;
  std::vector<double> comTrajY;
  std::vector<double> zmpTrajY;
  std::vector<double> leftfootXtraj;
  std::vector<double> leftfootYtraj;
  std::vector<double> leftfootHeight;
  std::vector<double> leftfootOrient;
  std::vector<double> rightfootXtraj;
  std::vector<double> rightfootYtraj;
  std::vector<double> rightfootHeight;
  std::vector<double> rightfootOrient;
  std::vector<double> waistOrient;
  double incrTime;
  double zc;
  unsigned int size;
};

class CnewPGstepStudy
{

	public:

		/*!
		 * Constructor.
		 */
		CnewPGstepStudy();

		/*!
		 * Destructor.
		 */
		~CnewPGstepStudy();

		/*! drawSteps outputs a file intended for gnuplot.
		 * Let's say that the result of drawSteps is in the
		 * file "data.gnuplot".
		 * Please write the following script file "toto.txt":
		 * set size ratio 1
		 * unset key
		 * set term postscript eps enhanced
		 * set output "totoOutput.eps"
		 * plot "data.gnuplot" index 0 w l lt 1 lw 0.5 lc 1,
		 * "data.gnuplot" index 1 w l lt 2 lw 0.5 lc 2, "data.gnuplot"
		 *  index 2 w l lt 2 lw 0.5 lc 2, "data.gnuplot" index 3 w l
		 * lt 1 lw 0.5 lc 3, "totoOutput.dat" index 4 w l lt 1 lw 0.5 lc 4
		 * Then, gnuplot> load 'toto.txt' will produce the eps file
		 * "totoOutput.eps".
		 * @param fb The ofstream corresponding to the file where the data
		 *           will be written.
		 * @param vect_input Describes the sequence of steps.\n
		 * @param leftOrRightFirstStable
		 * @param halfStepsON
		 * @param slideON
		 *  HERE IS A PRECISE DESCRIPTION OF THE FORMAT USED FOR vect_input:\n
		 *           First remark: all angles are in degrees.\n
		 *           Let's denote by "FStF" the foot which will be
		 *           stable during first step: the first stable foot.\n
		 *           The first 6 parameters are the following:\n
		 *                         We consider a plan (the floor) with a x
		 *                       axis and a y axis. The orientation of the x
		 *                       axis in this plan is 0.\n
		 *                         The (0,0) origin of this plan corresponds to
		 *                       the position of the waist and the CoM of
		 *                       the robot (they are supposed identical).\n
		 *                         The 3 first parameters correspond to the
		 *                       FStF: \n
		 *                       they are respectively the x position, y
		 *                       position and (absolute) orientation of the
		 *                       FStF. \n
		 *                         The orientation of the FStF will always be
		 *                       supposed equal to ZERO ! Hence, the third
		 *                       parameter (vect_input[2]) is superfluous:
		 *                       it should always be 0 !!  \n
		 *                         The 3 next parameters correspond to the
		 *                       first swinging foot (FSwF): x position, y
		 *                       orientation, (absolute) orientation.\n
		 *                         Let's denote by x, y, t, x', y', t', the
		 *                       first 6 parameters.\n
		 *                         We suppose also that the position of the
		 *                       CoM is at the BARYCENTER OF THE INITIAL
		 *                       POSITIONS OF THE FEET; therefore we have:
		 *                       -x' = x , and -y' = y. HENCE, the 6 first
		 *                       parameters can in fact be generated with
		 *                       only 3 parameters, and :\n
		 *                         x,y,t,x',y',t' = -x',-y',0,x',y',t'\n
		 *                           So, we need only the initial position
		 *                         and orientation of the FSwF to generate
		 *                         the 6 parameters.\n
		 *           The first 6 parameters are all ABSOLUTE coordinates,
		 *           but the following parameters are all RELATIVE
		 *           coordinates ! What follows is a list of groups of
		 *           parameters, each group corresponding to one step, or one
		 *           half step realized. The groups depends on the options
		 *           chosen. Let's describe all the cases.\n
		 *           CASE 1: halfStepsON=false, slidedStepsON=false.\n
		 *              In that case, what follows the 6 first parameters
	         *           are groups of 3 parameters, each group describing the
		 *           RELATIVE position of the last foot that was put on the
		 *           ground. For example, if G_i = (x_i, y_i, t_i) is the
		 *           ith such group, then (x_i, y_i) is the couple of
		 *           coordinates (in meters) of the vector between the
		 *           center of the stable foot and the swinging foot
		 *           just after the ith step is finished (i.e. when the
		 *           swinging foot touches the ground). Finally, t_i is the
		 *           RELATIVE orientation (in degrees) of the final position
		 *           of the swinging foot compared to the orientation of stable
		 *           foot. For example, since the orientation of the first
		 *           stable foot is 0, the parameter t_1 is the absolute
		 *           orientation of the FSwF when the first step is just finished.\n
		 *           CASE 2: halfStepsON=false, slidedStepsON=true.\n
		 *              !!WARNING!! This is for the moment a forbidden choice.
		 *           So far, the "sliding" has been implemented only with
		 *           sequences of half steps.
		 *           CASE 3: halfStepsON=true, slidedStepsON=false.\n
		 *              In that case, what follows the 6 first parameters
		 *           are groups of 5 parameters. Each group corresponds
		 *           to 2 half steps: one up, and one down. The two first
		 *           parameters correspond to the RELATIVE position of the
		 *           swing foot after the half step going up, i.e. when the
		 *           swing foot is at his highest point. At this moment,
		 *           the two feet are parallel. The first parameter is the
		 *           lateral distance (in meters) between the two feet, and
		 *           the second parameters is the height of the swing foot.
		 *           Then, since after the next half step a full step has
		 *           been realized, the three next parameters are defined
		 *           just as in the case 1.
		 *           CASE 4: halfStepsON=true, slidedStepsON=true.\n
		 *              In that case, what follows the first 6 parameters
	         *           are groups of 7 parameters. The 2nd and 3rd, 5th, 6th
		 *           and 7th parameters represent two half steps, and are
		 *           defined just as in the case 3. The 1st and 4th parameters
		 *           are "sliding negative times" (in s.), for respectively
		 *           the first and second half step. For example, a slide
		 *           parameter of -0.3s. means that the next step, instead
		 *           of starting after the previous half step has been completed,
		 *           will start 0.3s. before the end of the previous half
		 *           step. The last 0.3s. of the previous half step will thus
		 *           become a mixture between a finishing and a starting half
		 *           step. This technique is used to speed up the walk and make
		 *           it smoother.
		 *              !!IMPORTANT!!: since there is nothing before the first
		 *           half step, the first slide parameter must always be 0.\n
		 */
		void drawSeqStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable, double coefFeet);

		void drawSeqHalfStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable, double coefFeet);

		void drawSeqSlidedHalfStepFeatures(ofstream & fb, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable, double coefFeet);

		void plotOneDimensionCOMZMPSeqStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable);

		void plotOneDimensionCOMZMPSeqHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable);

		void plotOneDimensionCOMZMPSeqSlidedHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable);

		void plotFootHeightSeqStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vect_input, char leftOrRightFootStable);

		void plotFootHeightSeqHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable);

		void plotFootHeightSeqSlidedHalfStep(ofstream & fb, char whichDimension, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vect_input, char leftOrRightFootStable);




		void produceOneStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceOneUPHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceOneDOWNHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectStep_input, char leftOrRightFootStable);

		void produceSeqStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, double t4, double t5, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable);

		void produceSeqSlidedHalfStepFeatures(StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectSteps_input, char leftOrRightFootStable);

		void addStepFeaturesWithSlide(StepFeatures & stepF1, StepFeatures & stepF2, double negativeSlideTime);


	private:

		void genCOMZMPtrajectory(vector<double>& outputCOM, vector<double>& outputZMP, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5);

		void genFOOTposition(vector<double>& outputX, vector<double>& outputY, double incrTime, double xinit, double yinit, double xend, double yend, double delay, double t1, double t2, double t3, double t4, double t5, char dh);

		void genFOOTheight(vector<double>& output, double incrTime, double heightMax, double delay, double t1, double t2, double t3, double t4, double t5);

		void genFOOTupDOWNheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3);

		void genFOOTdownUPheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3);

		void genFOOTorientation(vector<double>& output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char dh);

		void genWAISTorientation(vector<double>& output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char dh);


};
#endif
