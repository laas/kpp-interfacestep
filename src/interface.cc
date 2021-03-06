# include <fstream>

# include <KineoGUI/kppMainWindowUICommandFactory.h>
# include <KineoController/kppUICommandList.h>
# include <KineoController/kppUICommand.h>

# include <jrl/mal/matrixabstractlayer.hh>

# include <hpp/util/debug.hh>
# include <hpp/wholebody-step-planner/planner.hh>
# include <hpp/corbaserver/wholebody-step/server.hh>

# include <kpp/interfacestep/interface.hh>
# include <kpp/interfacestep/command-grabobject.hh>
# include <kpp/interfacestep/command-setinitgoal.hh>
# include <kpp/interfacestep/command-dynamicpath.hh>
# include <kpp/interfacestep/command-removeconstraints.hh>

# include "config.h"

namespace kpp
{
  namespace interfaceStep
  {
    Interface::Interface (Planner * planner)
      : CkppInterface (planner),
	wholeBodyStepPlanner_ (planner)
    {
      char** argv = (char**) malloc (sizeof (char*));
      argv[0] = strdup ("Kite");
      server_ = new Server (1, argv, false, "child");
      server_->setPlanner (wholeBodyStepPlanner_);
      try {
	startCorbaServer ();
	hppDout (info, "Successfully started hpp-corbaserver.");
      } catch (const std::exception& exc) {
	hppDout (error, "Failed to start hpp-corbaserver");
      }
      try {
	server_->startCorbaServer("hpp", "plannerContext",
				  "hpp", "wholeBodyStep");
	hppDout (info, "Successfully started wholebody-step corba server.");
      } catch (const std::exception& exc) {
	hppDout (error, "Failed to start wholebody-step corba server.");
      }
    }

    Interface::~Interface ()
    {}

    InterfaceShPtr
    Interface::create ()
    {
      double samplingPeriod = 0.05;
      Planner * planner = new Planner(samplingPeriod);

      KIT_ASSERT(planner);
      planner->setFootPrintLimits(-0.2,0.2,-0.25,-0.13,-M_PI /4,0.1);

      InterfaceShPtr ptr (new Interface (planner));
      InterfaceWkPtr wkPtr (ptr);

      if (ptr->init (wkPtr) != KD_OK)
	ptr.reset ();
      return ptr;
    }

    ktStatus
    Interface::init (const InterfaceWkPtr& weakPtr)
    {
      if (CkppInterface::init (weakPtr) != KD_OK)
	return KD_ERROR;
      weakPtr_ = weakPtr;
      return KD_OK;
    }

    std::string
    Interface::name () const
    {
      return PACKAGE_STRING;
    }

    void
    Interface::getMenuUICommandLists(const CkppMainWindowUICommandFactoryConstShPtr & inCommandFactory,
				     std::vector< CkppUICommandListShPtr > & outMenuCommandListVector)
    {
      CkppInterface::getMenuUICommandLists(inCommandFactory, outMenuCommandListVector);

      CkppUICommandListShPtr hppUICommandList = CkppUICommandList::create ( "Step Planner" );

      commandGrabObject_ = CkppUICommand::create ( CommandGrabObject::create ( this ),
						    inCommandFactory->environment(),
						    "Generate goal configs",
						    "Generate goal configs");

      commandSetInitGoal_ = CkppUICommand::create ( CommandSetInitGoal::create ( this ),
						    inCommandFactory->environment(),
						    "Set Init/Goal configs",
						    "Set Init/Goal configs");
      commandRemoveConstraints_ =  CkppUICommand::create ( CommandRemoveConstraints::create ( this ),
						     inCommandFactory->environment(),
						     "Remove Foot Constraints",
						     "Remove Foot Constraints");
      commandDynamicPath_ =  CkppUICommand::create ( CommandDynamicPath::create ( this ),
						     inCommandFactory->environment(),
						     "Animate Path",
						     "Animate Path");

      hppUICommandList->appendCommand(commandGrabObject_);
      hppUICommandList->appendCommand(commandSetInitGoal_);
      hppUICommandList->appendCommand(commandRemoveConstraints_);
      hppUICommandList->appendCommand(commandDynamicPath_);

      outMenuCommandListVector.push_back ( hppUICommandList );
    }


  }
}

extern "C"
int
initializeModule (CkppModuleInterfaceShPtr& o_moduleInterface)
{
  using namespace kpp::interfaceStep;

  o_moduleInterface = Interface::create ();

  if (o_moduleInterface)
    return 0;

  //FIXME: for some reason, a segv happens if the module initialization
  // fails. As the user is doomed anyway, let's exit with a nice error
  // message.

  return 1;
}
