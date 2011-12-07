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
plugin* create_plugin (istringstream &)
{
  return new dynamicgraph::sot::openhrp::Plugin ();
}

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      Plugin::Plugin()
	:	  
	timeArray_ (),
	timeIndex_ (0),
	t0_ (),
	t1_ (),
	started_ (false)
      {
	// Set to zero C structures.
	bzero (timeArray_, TIME_ARRAY_SIZE * sizeof (double));
	bzero (&t0_, sizeof (timeval));
	bzero (&t1_, sizeof (timeval));

	// load the library where the robot controller is stored.
	
	register_method (":initialize", (method_t) &Plugin::start);
	register_method (":finalize", (method_t) &Plugin::stop);

	assigned_time = 0.005;
      }

      Plugin::~Plugin ()
      {
	// Do not delete entity_
      }

      void Plugin::initSotController()
      {
	// Load the SotDLRBipedController library.
	void * SotHRP2ControllerLibrary = dlopen("libsot-hrp2-controller.so",
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
			  map<string,SensorValues> & SensorsIn)
      {
	std::vector<double> angles(30);
	std::vector<double> forces(24);
	std::vector<double> torques(30);
	std::vector<double> zmp(3);
	std::vector<double> basePos(3);
	std::vector<double> baseAtt(9);

	// Update joint values.
	SensorsIn["joints"].setName("angle");
	for(unsigned int i=0;i<rs->angle.length();i++)
	  angles[i] = rs->angle[i];
	SensorsIn["joints"].setValues(angles);
	
	// Update forces
	SensorsIn["forces"].setName("force");
	for (int i = 0; i < 4; ++i)
	  for (int j = 0; j < 6; ++j)
	    forces[i*6+j] = rs->force[i][j];
	SensorsIn["forces"].setValues(forces);

	// Update torque
	SensorsIn["torques"].setName("torque");
	for (int j = 0; j < rs->torque.length(); ++j)
	  torques[j] = rs->torque[j];
	SensorsIn["torques"].setValues(torques);
      }

      void 
      Plugin::readControl(RobotState *mc,
			  map<string,ControlValues> &controlValues)
      {
	std::vector<double> angles(30);
	std::vector<double> forces(24);
	std::vector<double> torques(30);
	std::vector<double> zmp(3);
	std::vector<double> baseff(12);

	// Update joint values.
	angles = controlValues["joints"].getValues();
	for(unsigned int i=0;i<angles.size();i++)
	  mc->angle[i] = angles[i];
	
	// Update forces
	zmp =controlValues["zmp"].getValues();
	for(unsigned int i=0;i<3;i++)
	  mc->zmp[i] = zmp[i];

	// Update torque
	baseff =controlValues["baseff"].getValues();
	for (int j = 0; j < 3; ++j)
	  mc->basePos[j] = baseff[j*4+3];
	
	for(unsigned int i=0;i<3;++i)
	  for (int j = 0; j < 3; ++j)
	    mc->baseAtt[i*3+j] = baseff[i*3+3];
	
	
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

      bool
      Plugin::setup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
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
	map<string,SensorValues> SensorsIn;
	fillSensors(rs,SensorsIn);

	std::map<string,ControlValues> controlValues;
	bool res=false;
	try
	  {
	    sotController_->setupSetSensors(SensorsIn);
	    sotController_->getControl(controlValues);
	  } 
	catch SOT_RETHROW;

	readControl(mc,controlValues);

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);

	return res;
      }

      void
      Plugin::control (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);
	map<string,SensorValues> SensorsIn;
	fillSensors(rs,SensorsIn);

	std::map<string,ControlValues> controlValues;

	try 
	  {
	    sotController_->setupSetSensors(SensorsIn);
	    sotController_->getControl(controlValues);
	  } catch SOT_RETHROW;
	
	readControl(mc,controlValues);

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);
      }

      bool
      Plugin::cleanup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc)
      {
	// Log control loop start time.
	captureTime (t0_);
	map<string,SensorValues> SensorsIn;
	fillSensors(rs,SensorsIn);

	std::map<string,ControlValues> controlValues;

	bool res = false;
	try
	  {
	    sotController_->setupSetSensors(SensorsIn);
	    sotController_->getControl(controlValues);
	  } catch SOT_RETHROW;

	readControl(mc,controlValues);

	// Log control loop end time and compute time spent.
	captureTime (t1_);
	logTime (t0_, t1_);

	return res;
      }

    } // namespace openhrp
  } // namespace sot
} // namespace dynamicgraph
