// Copyright (C) 2011 by Sebastien Dalibard
#ifndef KPP_INTERFACE_DYNAMIC_PATH_HH
#define  KPP_INTERFACE_DYNAMIC_PATH_HH

# include <kpp/interfacestep/fwd.hh>

# include <KineoController/kppCommand.h>

namespace kpp
{
  namespace interfaceStep
  {
    class CommandDynamicPath : public CkppCommand
    {
    public:
	enum EParameters
	{ 
	  PATH_COMPONENT = 0,
	  PARAMETER_COUNT
	};
      
      static CommandDynamicPathShPtr create(Interface *kpp);
      
      static CommandDynamicPathShPtr
      createCopy( const  CommandDynamicPathConstShPtr& inCommand);

      virtual ~CommandDynamicPath();

      virtual ktStatus doExecute();

      virtual bool isUndoable() const;

     virtual CkppCommandShPtr clone() const;

      virtual unsigned int countParameters() const;

      virtual CkppParameterConstShPtr parameter(unsigned int inRank) const;

    protected:

      CommandDynamicPath( Interface *kpp);

      CommandDynamicPath(const CommandDynamicPath& inCommand);

      ktStatus init(const CommandDynamicPathWkPtr &inWkPtr);

    private:
      
      CommandDynamicPathWkPtr wkPtr_;

      Interface * kpp_;

    };
  } // end of namespace interfacestep
} // end of namespace kpp.


#endif
