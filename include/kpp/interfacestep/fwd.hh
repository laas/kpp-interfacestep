#ifndef KPP_INTERFACE_STEP_FWD_HH
# define KPP_INTERFACE_STEP_FWD_HH
# include <KineoUtility/kitDefine.h>
# include <hpp/walkplanner/fwd.hh>

KIT_PREDEF_CLASS (CkppUICommand);

namespace kpp
{
  namespace interfaceStep
  {
    KIT_PREDEF_CLASS (Interface);
    KIT_PREDEF_CLASS (CommandSetInitGoal);
    KIT_PREDEF_CLASS (CommandDynamicPath);
    KIT_PREDEF_CLASS (CommandRemoveConstraints);
    KIT_PREDEF_CLASS (CommandGrabObject);
  } 
} // end of namespace kpp.

#endif
