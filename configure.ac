# Don't expend macros beginning by JRL_.
m4_pattern_forbid([^JRL_])
AC_PREREQ([2.63])

## --------------------- ##
## Package configuration ##
## --------------------- ##
AC_INIT([kpp-interfacestep],
	[0.1],
	[sebastien.dalibard@laas.fr],
	[kpp-interfacestep])

# Auxiliary files.
AC_CONFIG_AUX_DIR([build-aux])

# Macros are stored in build-aux.
AC_CONFIG_MACRO_DIR([build-aux])

# Automake.
AM_INIT_AUTOMAKE([1.9.6 dist-bzip2 -Wall nostdinc])

## ------------- ##
## C++ compiler  ##
## ------------- ##

# Search for a C++ compiler.
AC_LANG([C++])
AC_PROG_CXX

# Speed-up compilation.
if test "$GXX" = yes; then
  CXXFLAGS="$CXXFLAGS -pipe"
fi

# Customize warnings.
JRL_CXX_WARNINGS([[-Wall],
                   [-Werror],
                   [-Wcast-align],
                   [-Wcast-qual],
                   [-Wformat],
                   [-Wmissing-prototypes],
                   [-Wstrict-prototypes],
                   [-Wwrite-strings],
                   [-Wbad-function-cast],
                   [-Wmissing-declarations],
                   [-Wnested-externs]])

# Search for uname.
AC_ARG_VAR([UNAME], [the uname system utility])
AC_CHECK_PROGS([UNAME], [uname])

if test x"$UNAME" = x; then
 AC_MSG_ERROR([cannot find uname binary])
fi

# Detect hostname for Kineo.
AC_SUBST([KINEO_LICENSE_HOST], [`$UNAME -n`])

# Search for pkg-config dependencies.
JRL_PKGCONFIGCHECK_ADDDEP([HPPWHOLEBODYTEPPLANNER], [hpp-wholebody-step-planner])
JRL_PKGCONFIGCHECK_ADDDEP([KPPINTERFACE], [kppInterface >= 1.6])
JRL_PKGCONFIGCHECK_ADDDEP([HPPCORBASERVER], [hpp-corbaserver]) dnl >= 1.7.4
JRL_PKGCONFIGCHECK_ADDDEP([KINEOPATHPLANNER], [KineoPathPlanner])

JRL_PKGCONFIGCHECK([HPPOPENHRP], [hppOpenHRP >= 1.6])
OPENHRP_PREFIX=`$PKG_CONFIG hppOpenHRP --variable=OpenHRP_prefix`
AC_DEFINE_UNQUOTED([OPENHRP_PREFIX], ["$OPENHRP_PREFIX"],
		   [OpenHRP installation prefix.])

JRL_PKGCONFIGCHECK([KINEOCONTROLLER], [KineoController])
AC_SUBST([KINEO_LIBDIR],[${KINEOCONTROLLER_PREFIX}/lib])
AC_SUBST(KINEO_PREFIX, [${KINEOPATHPLANNER_PREFIX}])
AC_SUBST([KINEO_ADDON_BUILDER],
	[${KINEOCONTROLLER_PREFIX}/bin/KineoAddonBuilder])


## ------------------- ##
## Additionnal tools.  ##
## ------------------- ##

# Search for misc. common tools.
AC_PROG_MKDIR_P
AC_PROG_SED

# Search for Doxygen.
JRL_PROG_DOXYGEN

# Search for Libtool.
LT_INIT


## ------------ ##
## Pkg-config.  ##
## ------------ ##

AC_SUBST([PKGCONFIG_DESCRIPTION], [kpp-interfacestep])
AC_SUBST([PKGCONFIG_CONFLICTS], [])
AC_SUBST([PKGCONFIG_LIBS], [-lkpp-interfacestep])
AC_SUBST([PKGCONFIG_CFLAGS], [])


## ------------ ##
## Test-suite.  ##
## ------------ ##

AC_CONFIG_TESTDIR([tests])
AM_MISSING_PROG([AUTOM4TE], [autom4te])


## -------------- ##
## Write output.  ##
## -------------- ##

# Create config.h
AC_CONFIG_HEADERS([config.h])

# Create Makefiles.
AC_CONFIG_FILES([
  Makefile
   include/Makefile
   src/Makefile
   doc/Makefile
   tests/Makefile
    tests/atlocal
    tests/package.m4
])

# Write files.
AC_OUTPUT
