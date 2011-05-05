# include <KineoModel/kppPathComponent.h>
# include <KineoModel/kppComponentClassFilter.h>
# include <KineoModel/kppComponentParameter.h>

# include <kpp/interfacestep/command-setinitgoal.hh>
# include <kpp/interfacestep/interface.hh>


namespace kpp
{
  namespace interfaceStep
  {
    CommandSetInitGoal::CommandSetInitGoal ( Interface * kpp)
    {
      kpp_ = kpp;
    }

    CommandSetInitGoal::CommandSetInitGoal(const CommandSetInitGoal& inCommand):
      CkppCommand ( inCommand )
    {
      kpp_ = inCommand.kpp_;
    }

    CommandSetInitGoal::~CommandSetInitGoal()
    {
      wkPtr_.reset();
    }

    CommandSetInitGoalShPtr 
    CommandSetInitGoal::create(Interface *kpp)
    {
      CommandSetInitGoal * ptr = new CommandSetInitGoal (kpp);
      CommandSetInitGoalShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    CommandSetInitGoalShPtr
    CommandSetInitGoal::createCopy( const  CommandSetInitGoalConstShPtr& inCommand)
    {
      CommandSetInitGoal * ptr = new CommandSetInitGoal( *inCommand);
      CommandSetInitGoalShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    ktStatus
    CommandSetInitGoal::init( const CommandSetInitGoalWkPtr &inWkPtr)
    {
      ktStatus res = CkppCommand::init (inWkPtr);
      if ( res == KD_OK)
	wkPtr_ = inWkPtr;
      return res;
    }

    CkppCommandShPtr
    CommandSetInitGoal::clone() const
    {
      return CommandSetInitGoal::createCopy ( wkPtr_.lock() );
    }

    bool 
    CommandSetInitGoal::isUndoable() const
    {
      return true;
    }

    unsigned int
    CommandSetInitGoal::countParameters() const
    {
      return PARAMETER_COUNT;
    }

    CkppParameterConstShPtr
    CommandSetInitGoal::parameter ( unsigned int inRank ) const
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
    CommandSetInitGoal::doExecute()
    {
      using namespace hpp::wholeBodyStepPlanner;

      CkppComponentShPtr component ( paramValue ( parameter ( PATH_COMPONENT ) ).componentValue() ); 
      
      CkppPathComponentShPtr pathComponent = 
	KIT_DYNAMIC_PTR_CAST ( CkppPathComponent, component );

      Planner * planner = 
	dynamic_cast<Planner*> (kpp_->hppPlanner());
      
      if (planner)
	planner->initAndGoalConfig (pathComponent->kwsPath());
      else
	return KD_ERROR;

      return KD_OK;
    }
  }
}
