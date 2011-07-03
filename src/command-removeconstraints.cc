# include <KineoModel/kppPathComponent.h>
# include <KineoModel/kppComponentClassFilter.h>
# include <KineoModel/kppComponentParameter.h>

# include <hpp/wholebody-step-planner/wholebody-constraint.hh>

# include "kpp/interfacestep/command-removeconstraints.hh"
# include "kpp/interfacestep/interface.hh"


namespace kpp
{
  namespace interfaceStep
  {
    CommandRemoveConstraints::CommandRemoveConstraints ( Interface * kpp)
    {
      kpp_ = kpp;
    }

    CommandRemoveConstraints::CommandRemoveConstraints(const CommandRemoveConstraints& inCommand):
      CkppCommand ( inCommand )
    {
      kpp_ = inCommand.kpp_;
    }

    CommandRemoveConstraints::~CommandRemoveConstraints()
    {
      wkPtr_.reset();
    }

    CommandRemoveConstraintsShPtr 
    CommandRemoveConstraints::create(Interface *kpp)
    {
      CommandRemoveConstraints * ptr = new CommandRemoveConstraints (kpp);
      CommandRemoveConstraintsShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    CommandRemoveConstraintsShPtr
    CommandRemoveConstraints::createCopy( const  CommandRemoveConstraintsConstShPtr& inCommand)
    {
      CommandRemoveConstraints * ptr = new CommandRemoveConstraints( *inCommand);
      CommandRemoveConstraintsShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    ktStatus
    CommandRemoveConstraints::init( const CommandRemoveConstraintsWkPtr &inWkPtr)
    {
      ktStatus res = CkppCommand::init (inWkPtr);
      if ( res == KD_OK)
	wkPtr_ = inWkPtr;
      return res;
    }

    CkppCommandShPtr
    CommandRemoveConstraints::clone() const
    {
      return CommandRemoveConstraints::createCopy ( wkPtr_.lock() );
    }

    bool 
    CommandRemoveConstraints::isUndoable() const
    {
      return true;
    }

    unsigned int
    CommandRemoveConstraints::countParameters() const
    {
      return PARAMETER_COUNT;
    }

    CkppParameterConstShPtr
    CommandRemoveConstraints::parameter ( unsigned int inRank ) const
    {
      CkppParameterShPtr result;
      CkppComponentShPtr nullComponent;
      switch ( inRank )
	{
	default:
	case PATH_COMPONENT : result = CkppComponentParameter::create ( "Path Component", CkppComponentClassFilter<CkppPathComponent>() );    break;
	  KIT_ASSERT ( false );
	}
      return result;
    }

    ktStatus
    CommandRemoveConstraints::doExecute()
    {
      using namespace hpp::wholeBodyStepPlanner;

      Planner * planner = 
	dynamic_cast<Planner*> (kpp_->hppPlanner());
      
      if (planner)
	  planner->humanoidRobot ()->userConstraints ()->remove (planner->wholeBodyConstraint ());
      else
	return KD_ERROR;

      return KD_OK;
    }
  }
}
