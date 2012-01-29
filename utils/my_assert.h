#define ASSERT(exp)  if (exp) ;						\
  else {								\
    printf("%s failed in file %s at line %i\n", #exp, __FILE__, __LINE__ ); \
    exit(0);\
  }

