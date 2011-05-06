// Copyright (C) 2011 by Sebastien Dalibard
#ifndef KPP_INTERFACE_REMOVE_CONSTRAINTS_HH
#define  KPP_INTERFACE_REMOVE_CONSTRAINTS_HH

# include <kpp/interfacestep/fwd.hh>

# include <KineoController/kppCommand.h>

namespace kpp
{
  namespace interfaceStep
  {
    class CommandRemoveConstraints : public CkppCommand
    {
    public:
	enum EParameters
	{ 
	  PATH_COMPONENT = 0,
	  PARAMETER_COUNT
	};
      
      static CommandRemoveConstraintsShPtr create(Interface *kpp);
      
      static CommandRemoveConstraintsShPtr
      createCopy( const  CommandRemoveConstraintsConstShPtr& inCommand);

      virtual ~CommandRemoveConstraints();

      virtual ktStatus doExecute();

      virtual bool isUndoable() const;

     virtual CkppCommandShPtr clone() const;

      virtual unsigned int countParameters() const;

      virtual CkppParameterConstShPtr parameter(unsigned int inRank) const;

    protected:

      CommandRemoveConstraints( Interface *kpp);

      CommandRemoveConstraints(const CommandRemoveConstraints& inCommand);

      ktStatus init(const CommandRemoveConstraintsWkPtr &inWkPtr);

    private:
      
      CommandRemoveConstraintsWkPtr wkPtr_;

      Interface * kpp_;

    };
  } // end of namespace interfacestep
} // end of namespace kpp.


#endif
