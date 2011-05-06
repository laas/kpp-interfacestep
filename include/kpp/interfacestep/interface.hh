// Copyright (C) 2011 by Sebastien Dalibard
#ifndef KPP_INTERFACE_STEP_INTERFACE_HH
# define KPP_INTERFACE_STEP_INTERFACE_HH

# include <KineoController/kppUICommand.h>

# include <hpp/wholebody-step-planner/planner.hh>

# include <kpp/interfacestep/fwd.hh>
# include <kpp/interfacestep/kppinterface-nowarning.hh>



namespace kpp
{
  namespace interfaceStep
  {

    class Interface : public CkppInterface
    {
    public:

      typedef hpp::wholeBodyStepPlanner::Planner Planner;


      virtual ~Interface ();

      static InterfaceShPtr create ();

      virtual std::string name () const;

      virtual void 
      getMenuUICommandLists( const CkppMainWindowUICommandFactoryConstShPtr & inCommandFactory,
			     std::vector< CkppUICommandListShPtr > & outMenuCommandListVector	 
			     );

    protected:
      Interface (Planner * inHppPlanner);

      /// \brief Initialization of the object
      ///
      /// Each object stores a weak pointer to itself, instatiated by
      /// calling this function.
      ktStatus init (const InterfaceWkPtr& inWeakPtr);


      /// Planner associed to the interface
      Planner * attHppStepPlanner;

      
      

      /// \}
    private:
      /// \brief Weak pointer to itself
      InterfaceWkPtr attWeakPtr;
      CkppUICommandShPtr commandSetInitGoal_;
      CkppUICommandShPtr commandDynamicPath_;
      CkppUICommandShPtr commandRemoveConstraints_;
   
    };

  } // end of namespace interfaceWalk.
} // end of namespace kpp.


/// \brief KPP initialization function.
///
/// First command ran by Kineo when loading the module and checking
/// for the license.
extern "C" KPP_ADDON_API int
initializeModule (CkppModuleInterfaceShPtr& o_moduleInterface);

#endif //! KPP_INTERFACE_WALK_INTERFACE_HH
