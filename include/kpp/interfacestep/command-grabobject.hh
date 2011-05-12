/*
  Research carried out within the scope of the TME/LAAS-CNRS Contract

  Developed by Sebastien Dalibard


*/


#ifndef KPP_INTERFACE_GRAB_OBJECT_H
#define KPP_INTERFACE_GRAB_OBJECT_H

#include <kpp/interfacestep/fwd.hh>

#include <KineoController/kppCommand.h>

#include <iostream>

/**
   class CommandGrabObject
  \brief Command for initializing and solving a grasping problem
 */

namespace kpp
{
  namespace interfaceStep
  {
    class CommandGrabObject : public CkppCommand
    {

    public:

      enum EParameters
	{ 
	  SOLID_COMPONENT = 0,
	  PARAMETER_COUNT
	};
	  

      static CommandGrabObjectShPtr   create(Interface *kpp);

      static CommandGrabObjectShPtr   createCopy(const CommandGrabObjectConstShPtr& inCommand);

      virtual ~CommandGrabObject();

      virtual ktStatus doExecute();

      virtual bool	isUndoable() const;

      virtual CkppCommandShPtr clone() const;

      virtual unsigned int	countParameters() const;

      virtual CkppParameterConstShPtr	parameter(unsigned int inRank) const;

    protected:

      CommandGrabObject(Interface *kpp);

      CommandGrabObject(const CommandGrabObject& inCommand);

      ktStatus init(const CommandGrabObjectWkPtr& inWeakPtr);

    private:

      CommandGrabObjectWkPtr attWeakPtr;

      Interface *attKpp;

    };
  } // end of namespace interfacestep
} // end of namespace kpp.

#endif
