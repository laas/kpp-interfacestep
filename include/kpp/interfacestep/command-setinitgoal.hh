// Copyright (C) 2011 by Sebastien Dalibard
#ifndef KPP_INTERFACE_STEP_COMMAND_SETINITGOAL_HH
#define  KPP_INTERFACE_STEP_COMMAND_SETINITGOAL_HH

# include <kpp/interfacestep/fwd.hh>

# include <KineoController/kppCommand.h>

namespace kpp
{
  namespace interfaceStep
  {
    class CommandSetInitGoal : public CkppCommand
    {
    public:
	enum EParameters
	{ 
	  PATH_COMPONENT = 0,
	  PARAMETER_COUNT
	};
      
      static CommandSetInitGoalShPtr create(Interface *kpp);
      
      static CommandSetInitGoalShPtr
      createCopy( const  CommandSetInitGoalConstShPtr& inCommand);

      virtual ~CommandSetInitGoal();

      virtual ktStatus doExecute();

      virtual bool isUndoable() const;

     virtual CkppCommandShPtr clone() const;

      virtual unsigned int countParameters() const;

      virtual CkppParameterConstShPtr parameter(unsigned int inRank) const;

    protected:

      CommandSetInitGoal( Interface *kpp);

      CommandSetInitGoal(const CommandSetInitGoal& inCommand);

      ktStatus init(const CommandSetInitGoalWkPtr &inWkPtr);

    private:
      
      CommandSetInitGoalWkPtr wkPtr_;

      Interface * kpp_;

    };
  } // end of namespace interfacestep
} // end of namespace kpp.


#endif
