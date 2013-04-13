#pragma once

// Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define BULLETSIM_HELPER_DLL_IMPORT __declspec(dllimport)
  #define BULLETSIM_HELPER_DLL_EXPORT __declspec(dllexport)
  #define BULLETSIM_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define BULLETSIM_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define BULLETSIM_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define BULLETSIM_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define BULLETSIM_HELPER_DLL_IMPORT
    #define BULLETSIM_HELPER_DLL_EXPORT
    #define BULLETSIM_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define BULLETSIM_API and BULLETSIM_LOCAL.
// BULLETSIM_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// BULLETSIM_LOCAL is used for non-api symbols.

#define BULLETSIM_DLL

#ifdef BULLETSIM_DLL // defined if BULLETSIM is compiled as a DLL
  #ifdef BULLETSIM_DLL_EXPORTS // defined if we are building the BULLETSIM DLL (instead of using it)
    #define BULLETSIM_API BULLETSIM_HELPER_DLL_EXPORT
  #else
    #define BULLETSIM_API BULLETSIM_HELPER_DLL_IMPORT
  #endif // BULLETSIM_DLL_EXPORTS
  #define BULLETSIM_LOCAL BULLETSIM_HELPER_DLL_LOCAL
#else // BULLETSIM_DLL is not defined: this means BULLETSIM is a static lib.
  #define BULLETSIM_API
  #define BULLETSIM_LOCAL
#endif // BULLETSIM_DLL





#define PRINT_AND_THROW(s) do {\
  std::cerr << "\033[1;31mERROR " << s << "\033[0m\n";\
  std::cerr << "at " << __FILE__ << ":" << __LINE__ << std::endl;\
  std::stringstream ss;\
  ss << s;\
  throw std::runtime_error(ss.str());\
} while (0)
#define FAIL_IF_FALSE(expr) if (!expr) {\
    PRINT_AND_THROW( "expected true: " #expr);\
  }

#ifdef __CDT_PARSER__
#define BOOST_FOREACH(a,b) for(;;)
#endif

#define ALWAYS_ASSERT(exp) if (!(exp)) {printf("%s failed in file %s at line %i\n", #exp, __FILE__, __LINE__ ); abort();}
