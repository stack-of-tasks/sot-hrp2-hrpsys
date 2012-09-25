// Copyright 2010, 2011 François Bleibel, Florent Lamiraux, Thomas
// Moulard, Olivier Stasse, JRL, CNRS/AIST
//
// This file is part of sot-hrp2-hrpsys.
// sot-hrp2-hrpsys is not a free software. You cannot
// redistribute it without making sure that confidential
// information regarding the humanoid robot HRP-2 
// is provided to people having purchased the proper license,
// or signed the non-disclosure agreement.

#include "plugin.hh"
#include <iostream>
#include <fstream>

#include <sot/core/abstract-sot-external-interface.hh>
#include <sot/core/debug.hh>
#include <sot/core/exception-abstract.hh>

typedef void (command_receiver::*method_t) (std::istringstream&);
const std::string DT_DAT_OUTPUT_FILE ("/tmp/dt.dat");
plugin* create_plugin (istringstream &iss)
{
  std::string libname;
  iss >> libname;
  return new dynamicgraph::sot::openhrp::Plugin (libname);
}

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      Plugin::Plugin(std::string &libname)
	:	  
	timeArray_ (),
	timeIndex_ (0),
	t0_ (),
	t1_ (),
	started_ (false),
	libname_(libname),
	sensorsIn_ (),
	controlValues_ (),
	angleEncoder_ (),
	angleControl_ (),
	forces_ (),
	torques_ ()
      {
	// Set to zero C structures.
	bzero (timeArray_, TIME_ARRAY_SIZE * sizeof (double));
	bzero (&t0_, sizeof (timeval));
	bzero (&t1_, sizeof (timeval));

	// load the library where the robot controller is stored.
	
	register_method (":initialize", (method_t) &Plugin::start);
	register_method (":finalize", (method_t) &Plugin::stop);

	initSotController();

	assigned_time = 0.005;
      }

      Plugin::~Plugin ()
      {
	// Do not delete entity_
      }

      void Plugin::initSotController()
      {
	// Load the SotDLRBipedController library.
	void * SotHRP2ControllerLibrary = dlopen(libname_.c_str(),
						  RTLD_GLOBAL | RTLD_NOW);
	 if (!SotHRP2ControllerLibrary) {
	   std::cerr << "Cannot load library: " << dlerror() << '\n';
	   return ;
	 }

	 // reset errors
	 dlerror();

	 // Load the symbols.
	 createSotExternalInterface_t * createHRP2Controller =
	   (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
						  "createSotExternalInterface");
	 const char* dlsym_error = dlerror();
	 if (dlsym_error) {
	   std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
	   return ;
	 }

	 destroySotExternalInterface_t * destroyHRP2Controller =
	   (destroySotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
						   "destroySotExternalInterface");
	 dlsym_error = dlerror();
	 if (dlsym_error) {
	   std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
	   return ;
	 }

	 // Create hrp2-controller
	 sotController_ = createHRP2Controller();
       }

       void 
       Plugin::fillSensors(RobotState *rs,
			   map<string,SensorValues> & sensorsIn)
       {
	 // Update joint values.w
	 sensorsIn["joints"].setName("angle");
	 for(unsigned int i=0;i<rs->angle.length();i++)
	   angleEncoder_[i] = rs->angle[i];
	 sensorsIn["joints"].setValues(angleEncoder_);

	 // Update forces
	 sensorsIn["forces"].setName("force");
	 for (unsigned int i = 0; i < rs->force.length(); ++i)
	   for (unsigned int j = 0; j < rs->force[i].length(); ++j)
	     forces_[i*6+j] = rs->force[i][j];
	 sensorsIn["forces"].setValues(forces_);

	 // Update torque
	 sensorsIn["torques"].setName("torque");
	 for (unsigned int j = 0; j < rs->torque.length(); ++j)
	   torques_[j] = rs->torque[j];
	 sensorsIn["torques"].setValues(torques_);

	 // Update attitude
	 sensorsIn["attitude"].setName ("attitude");
	 for (unsigned int j = 0; j < rs->attitude [0].length (); ++j)
	   baseAtt_ [j] = rs->attitude [0][j];
	 sensorsIn["attitude"].setValues (baseAtt_);
       }

       void 
       Plugin::readControl(RobotState *mc,
			   map<string,ControlValues> &controlValues)
       {
	 static unsigned int nbit=0;
	 
	 // Update joint values.
	 angleControl_ = controlValues["joints"].getValues();
	 
	 /*
	 if (nbit%100==0)
	   std::cout << "Size of angles: " << angleControl_.size() 
		     << " Size of mc->angle: " << mc->angle.length() 
		     << std::endl;
	 */
	 for(unsigned int i=0;i<angleControl_.size();i++)
	   {
	     mc->angle[i] = angleControl_[i];
	     //if (nbit%100==0)
	     //	       std::cout << mc->angle[i] << " ";
	   }

	 /* 
	    if (nbit%100==0)
	   std::cout << std::endl;
	   nbit++;
	 */

	 // Update forces
	 const std::vector<double>& zmp (controlValues["zmp"].getValues());
	 for(unsigned int i=0;i<3;i++)
	   mc->zmp[i] = zmp[i];

	 // Update torque
	 const std::vector<double>& baseff =
	   controlValues["baseff"].getValues();
	 for (int j = 0; j < 3; ++j)
	   mc->basePos[j] = baseff[j*4+3];

	 for(unsigned int i=0;i<3;++i)
	   for (int j = 0; j < 3; ++j)
	     mc->baseAtt[i*3+j] = baseff[i*4+j];
       }

       void
       Plugin::stop (std::istringstream&)
       {
	 // Write log data to file.
	 writeLog ();
       }

       void
       Plugin::writeLog ()
       {
	 std::ofstream of (DT_DAT_OUTPUT_FILE.c_str ());
	 of << "# size = " << timeIndex_ << std::endl;
	 for (unsigned i = 0; i < timeIndex_; ++i)
	   of << i << "\t" << timeArray_[i] << std::endl;
       }

       void
       Plugin::captureTime (timeval& t)
       {
	 gettimeofday (&t, NULL);
       }

       void
       Plugin::logTime (const timeval& t0, const timeval& t1)
       {
	 double dt =
	   (t1.tv_sec - t0.tv_sec) * 1000.
	   + (t1.tv_usec - t0.tv_usec + 0.) / 1000.;

	 if (timeIndex_ < TIME_ARRAY_SIZE)
	   timeArray_[timeIndex_++] = dt;
       }

       void
       Plugin::start (std::istringstream&)
       {
	 started_ = true;
       }

       void
       Plugin::displayRobotState(RobotState * ars)
       {
	 cout << "Angles:";
	 for(unsigned int i=0;i<ars->angle.length();i++)
	   cout << ars->angle[i] << " " ;
	 cout << endl;

	 cout << "ZMP:";
	 for(unsigned int i=0;i<3;i++)
	   cout << ars->zmp[i] << " ";
	 cout << endl;

	 cout << "basePos: ";
	 for (int j = 0; j < 3; ++j)
	   cout << ars->basePos[j] << " ";
	 cout << endl;

	 cout << "baseAtt: ";
	 for(unsigned int i=0;i<3;++i)
	   {
	     for (int j = 0; j < 3; ++j)
	       cout << ars->baseAtt[i*3+j]<< " ";
	     cout << endl;
	   }
       }

       bool
       Plugin::setup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
       {
	 // Resize vector of angles
	 angleEncoder_.resize (rs->angle.length());
	 angleControl_.resize (rs->angle.length());
	 forces_.resize (6*rs->force.length ());
	 torques_.resize(rs->torque.length());
	 baseAtt_.resize (rs->attitude [0].length ());

	 if (!started_)
	   {
	     std::cout
	       << "Please call ':initialize' before starting the plug-in."
	       << std::endl;
	     return false;
	   }

	 // Log control loop start time.
	 captureTime (t0_);

	 // Initialize client to seqplay.
	 fillSensors(rs,sensorsIn_);
	 try
	   {
	     sotController_->setupSetSensors(sensorsIn_);
	     sotController_->getControl(controlValues_);
	   } 
	 catch (std::exception &e) {  std::cout << e.what() <<endl;throw e; }
	 readControl(mc,controlValues_);

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);
	return true;
      }
      
      void
      Plugin::control (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);
	fillSensors(rs,sensorsIn_);

	try 
	  {
	    sotController_->setupSetSensors(sensorsIn_);
	    sotController_->getControl(controlValues_);
	  } 
	catch(std::exception &e) { throw e;} 

	readControl(mc,controlValues_);

	//displayRobotState(mc);
	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);
      }

      bool
      Plugin::cleanup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);
	fillSensors(rs,sensorsIn_);

	bool res = false;
	try
	  {
	    sotController_->setupSetSensors(sensorsIn_);
	    sotController_->getControl(controlValues_);
	  } catch(std::exception &e) { throw e;}
	readControl(mc,controlValues_);

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);
	res = true;
	return res;
      }

    } // namespace openhrp
  } // namespace sot
} // namespace dynamicgraph
