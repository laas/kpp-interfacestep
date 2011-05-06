# include <fstream>

# include <KineoGUI/kppMainWindowUICommandFactory.h>
# include <KineoController/kppUICommandList.h>
# include <KineoController/kppUICommand.h>

# include <kpp/interfacestep/interface.hh>
# include <kpp/interfacestep/command-setinitgoal.hh>
# include <kpp/interfacestep/command-dynamicpath.hh>


# include "config.h"

namespace kpp
{
  namespace interfaceStep
  {
   Interface::Interface (Planner * planner)
      : CkppInterface (planner),
	attHppStepPlanner (planner)
    {}

    Interface::~Interface ()
    {}

    InterfaceShPtr
    Interface::create ()
    {
      double samplingPeriod = 0.005;

      std::string pathToHrp2 (OPENHRP_PREFIX);
      pathToHrp2 += "/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl";

   
   
      std::ifstream hrp2Wrl;
      hrp2Wrl.open (pathToHrp2.c_str (), std::ios_base::in);
      hrp2Wrl.close ();
      if (hrp2Wrl.fail ())
	{
	  return InterfaceShPtr ();
	}

      Planner * planner = new Planner(samplingPeriod);

      if (!planner)
	{
	  return InterfaceShPtr ();
	}

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
      attWeakPtr = weakPtr;
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

      commandSetInitGoal_ = CkppUICommand::create ( CommandSetInitGoal::create ( this ),
						    inCommandFactory->environment(),
						    "Set Init/Goal configs",
						    "Set Init/Goal configs");
      commandDynamicPath_ =  CkppUICommand::create ( CommandDynamicPath::create ( this ),
						     inCommandFactory->environment(),
						     "Animate Path",
						     "Animate Path");

      hppUICommandList->appendCommand(commandSetInitGoal_);
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
