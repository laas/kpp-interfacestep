# include <KineoModel/kppPathComponent.h>
# include <KineoModel/kppComponentClassFilter.h>
# include <KineoModel/kppComponentParameter.h>

# include <kpp/interfacestep/command-dynamicpath.hh>
# include <kpp/interfacestep/interface.hh>


namespace kpp
{
  namespace interfaceStep
  {
    CommandDynamicPath::CommandDynamicPath ( Interface * kpp)
    {
      kpp_ = kpp;
    }

    CommandDynamicPath::CommandDynamicPath(const CommandDynamicPath& inCommand):
      CkppCommand ( inCommand )
    {
      kpp_ = inCommand.kpp_;
    }

    CommandDynamicPath::~CommandDynamicPath()
    {
      wkPtr_.reset();
    }

    CommandDynamicPathShPtr 
    CommandDynamicPath::create(Interface *kpp)
    {
      CommandDynamicPath * ptr = new CommandDynamicPath (kpp);
      CommandDynamicPathShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    CommandDynamicPathShPtr
    CommandDynamicPath::createCopy( const  CommandDynamicPathConstShPtr& inCommand)
    {
      CommandDynamicPath * ptr = new CommandDynamicPath( *inCommand);
      CommandDynamicPathShPtr shPtr (ptr);
      if (ptr->init (shPtr) != KD_OK)
	shPtr.reset();
      return shPtr;
    }

    ktStatus
    CommandDynamicPath::init( const CommandDynamicPathWkPtr &inWkPtr)
    {
      ktStatus res = CkppCommand::init (inWkPtr);
      if ( res == KD_OK)
	wkPtr_ = inWkPtr;
      return res;
    }

    CkppCommandShPtr
    CommandDynamicPath::clone() const
    {
      return CommandDynamicPath::createCopy ( wkPtr_.lock() );
    }

    bool 
    CommandDynamicPath::isUndoable() const
    {
      return true;
    }

    unsigned int
    CommandDynamicPath::countParameters() const
    {
      return PARAMETER_COUNT;
    }

    CkppParameterConstShPtr
    CommandDynamicPath::parameter ( unsigned int inRank ) const
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
    CommandDynamicPath::doExecute()
    {
      using namespace hpp::wholeBodyStepPlanner;

      CkppComponentShPtr component ( paramValue ( parameter ( PATH_COMPONENT ) ).componentValue() ); 
      
      CkppPathComponentShPtr pathComponent = 
	KIT_DYNAMIC_PTR_CAST ( CkppPathComponent, component );

      Planner * planner = 
	dynamic_cast<Planner*> (kpp_->hppPlanner());
      
      if (planner)
	{
	  CkwsPathShPtr dynamicPath = planner-> findDynamicPath(pathComponent->kwsPath());
	  if (dynamicPath)
	    planner->hppProblem(0)->addPath(dynamicPath);
	}
      else
	return KD_ERROR;

      return KD_OK;
    }
  }
}
