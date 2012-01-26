// Copyright 2010, 2011 François Bleibel, Florent Lamiraux, Thomas
// Moulard, Olivier Stasse, JRL, CNRS/AIST
//
// This file is part of sot-hrp2-hrpsys.
// sot-hrp2-hrpsys is not a free software. You cannot
// redistribute it without making sure that confidential
// information regarding the humanoid robot HRP-2 
// is provided to people having purchased the proper license,
// or signed the non-disclosure agreement.


#ifndef DYNAMIC_GRAPH_SOT_HRP2_HRPSYS_PLUGIN_HH
# define DYNAMIC_GRAPH_SOT_HRP2_HRPSYS_PLUGIN_HH
# include <sys/time.h>

# include <sot/core/abstract-sot-external-interface.hh>

# include "Plugin_impl.h"

namespace dynamicgraph
{
  namespace sot
  {
    namespace openhrp
    {
      /// \brief OpenHRP plugin to control HRP2
      ///
      /// This plug-in plugs dynamic-graph into the OpenHRP
      /// architecture. This class implements a custom OpenHRP
      /// plug-in.
      ///
      /// An OpenHRP plug-in provides both CORBA requests for
      /// asynchronous/non real-time changes and a control loop
      /// executed in a real-time context.
      ///
      /// The real-time part gathers the setup, control and cleanup
      /// methods respectively called during initialization, every
      /// control loop and when the plug-in is destroyed. This plug-in
      /// is controlled by a Python script running in OpenHRP
      /// (GrxUi.sh). In Python, setup and destroy are mapped to the
      /// start and stop commands.
      ///
      /// As setup and cleanup are executed in the real time loop,
      /// additional start/stop custom procedures are provided to
      /// allow long initialization (i.e. parsing the robot model,
      /// etc). These are the start and stop methods of this class.
      /// They can be accessed in the Python script through the
      /// generic sendMsg command (commands are respectively
      /// initialize and finalize).
      ///
      /// To finish, this controller creates an  entity 
      /// which attribute creates the device that
      /// will be used in dynamic-graph. A device provides the robot
      /// state to the graph and receives the control at the end of
      /// the control loop. See stack-of-tasks.hh for more
      /// information.
      class Plugin : public ::plugin
      {
      public:

	explicit Plugin (std::string &libname);
	virtual ~Plugin ();

	/// \name Inherited control methods.
	/// \{

	/// \brief Called when plug-in is started.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool setup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called at each control loop.
	/// \param rs robot state
	/// \param mc motor control
	virtual void control (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \brief Called when plug-in is stopped.
	/// \param rs robot state
	/// \param mc motor control
	virtual bool cleanup (OpenHRP::RobotState* rs, OpenHRP::RobotState* mc);

	/// \}

      private:
	/// \name Custom OpenHRP commands.
	/// \{

	/// \brief Non real-time initialization method.
	///
	/// This triggers the prologue interpretation and launch the
	/// CORBA server (i.e. the one receiving Python code).
	void start (std::istringstream&);

	/// \brief Non real-time stopping method.
	///
	/// This dumps the logged information.
	void stop (std::istringstream&);
	/// \}

	/// \brief Capture the current time in the argument.
	/// \param t store current time
	void captureTime (timeval& t);

	/// \brief Log time spent between t0 and t1 in the next
	/// free case of timeArray.
	///
	/// \param t0 begin time
	/// \param t1 end time
	void logTime (const timeval& t0, const timeval& t1);

	/// \brief Write logged times in a file.
	void writeLog ();

	/// \brief Load the library which contains the hrp2-sot-controller.
	void initSotController();

	/// \brief Fill the structure Sensors in with current robot state.
	void fillSensors(RobotState *rs,
			 map<string,SensorValues> & SensorsIn);

	/// \brief Read the control value and fill the mc data structure.
	void readControl(RobotState *mc,
			 map<string,ControlValues> & controlValues);

	/// \brief Display the data structure robot state.
	void displayRobotState(RobotState *rs);

	/// \brief Size of the array logging time spent in control loop.
	static const unsigned int TIME_ARRAY_SIZE = 100000;


	/// \brief Log time spend during control loops.
	double timeArray_[TIME_ARRAY_SIZE];
	/// \brief First unfilled item in timeArray.
	unsigned int timeIndex_;

	/// \brief Timestamp matching the beginning of the control
	/// loop.
	timeval t0_;
	/// \brief Timestamp matching the end of the control loop.
	timeval t1_;

	/// \brief Did the non real-time start function been called?
	bool started_;

	/// \brief the sot-hrp2 controller
	AbstractSotExternalInterface * sotController_;

	/// \brief Name of the controller to load
	std::string libname_;
      };

    } // end of namespace openhrp.
  } // end of namespace sot.
} // end of namespace dynamicgraph.

#endif // DYNAMIC_GRAPH_SOT_OPENHRP_PLUGIN_HH
