#include <vector>

#include "KineoModel/kppRotationJointComponent.h"
#include "KineoController/kppSetPropertyCommand.h"
#include "KineoController/kppDocument.h"
#include "KineoModel/kppComponentParameter.h"
#include "KineoModel/kppNumberParameter.h"
#include "KineoModel/kppValue.h"
#include "KineoModel/kppComponentClassFilter.h"
#include "KineoModel/kppComponent.h"
#include "KineoModel/kppSolidComponent.h"
#include "KineoModel/kppPathNode.h"
#include "KineoModel/kppPathComponent.h"
#include "KineoModel/kppModelTree.h"
#include "KineoGUI/kppMainWindowController.h" 
#include "KineoController/kppDocument.h"
#include "KineoGUI/kppMainWindowUICommandFactory.h"

# include <hpp/util/debug.hh>
# include <hpp/wholebody-step-planner/planner.hh>

#include "kpp/interfacestep/command-grabobject.hh"
#include "kpp/interfacestep/interface.hh"

namespace kpp
{
  namespace interfaceStep
  {
    // ==========================================================================

    CommandGrabObject::CommandGrabObject(Interface *kpp)
    {

      attKpp = kpp;

    }


    // ==========================================================================

    CommandGrabObject::CommandGrabObject(const CommandGrabObject& inCommand) :
      CkppCommand(inCommand)
    {
      attKpp = inCommand.attKpp;

    }


    // ==========================================================================

    CommandGrabObject::~CommandGrabObject()
    {
      weakPtr_.reset();
    }


    // ==========================================================================

    CommandGrabObjectShPtr CommandGrabObject::create(Interface *kpp)
    {
      CommandGrabObject*  ptr = new CommandGrabObject(kpp);
      CommandGrabObjectShPtr shPtr(ptr);
  
      if(KD_OK != ptr->init(shPtr))
	{
	  shPtr.reset();
	}

      return shPtr;
    }

    // ==========================================================================

    CommandGrabObjectShPtr CommandGrabObject::createCopy(const CommandGrabObjectConstShPtr& inCommand)
    {
      CommandGrabObject*  ptr = new CommandGrabObject(*inCommand);
      CommandGrabObjectShPtr shPtr(ptr);
  
      if(KD_OK != ptr->init(shPtr))
	{
	  shPtr.reset();
	}
  
      return shPtr;
    }


    // ==========================================================================

    ktStatus CommandGrabObject::init(const CommandGrabObjectWkPtr& inWeakPtr)
    {
      ktStatus success = CkppCommand::init(inWeakPtr);
  
      if(KD_OK == success)
	{
	  weakPtr_ = inWeakPtr;
	}


      return success;
    }


    // ==========================================================================


    CkppCommandShPtr CommandGrabObject::clone() const
    {
      return CommandGrabObject::createCopy(weakPtr_.lock());
    }


    // ==========================================================================

    bool CommandGrabObject::isUndoable() const
    {
      return true;
    }


    // ==========================================================================

    unsigned int CommandGrabObject::countParameters() const
    {
      return PARAMETER_COUNT;
    }


    // ==========================================================================

    CkppParameterConstShPtr CommandGrabObject::parameter(unsigned int inRank) const
    {
 
      CkppParameterShPtr result;
      CkppComponentShPtr nullComponent;

      switch(inRank) {   
      default:
      case SOLID_COMPONENT : result = CkppComponentParameter::create("Solid Component", CkppComponentClassFilter<CkppSolidComponent>());    break;
	KIT_ASSERT( false );
      }
  
      return result;
    }


    // ==========================================================================

    ktStatus CommandGrabObject::doExecute()
    {
      CkppComponentShPtr component(paramValue(parameter(SOLID_COMPONENT)).componentValue());

      CkppSolidComponentShPtr solidComponent = KIT_DYNAMIC_PTR_CAST(CkppSolidComponent, component);

      using namespace hpp::wholeBodyStepPlanner;
	
      Planner *planner = dynamic_cast<Planner*> ( attKpp->hppPlanner() );

      CkitMat4 absPos;
      solidComponent->getAbsolutePosition ( absPos );
      vectorN target ( 3 );
      target[0] = absPos ( 0,3 );
      target[1] = absPos ( 1,3 );
      target[2] = absPos ( 2,3 );

      planner->generateGoalConfig (target[0], target[1], target[2], 1);

      return KD_OK;
    }

    // ==========================================================================
  }
}
