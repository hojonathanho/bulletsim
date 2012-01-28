#define ASSERT(exp)  if (exp) ;						\
  else {								\
  printf("%s failed in file %s at line %i" #exp, __FILE__, __BASE_FILE__, __LINE__ ); \
  exit(0);}

