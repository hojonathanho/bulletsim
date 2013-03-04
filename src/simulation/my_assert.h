#pragma once
#define ENSURE(exp) if (!(exp)) {printf("%s failed in file %s at line %i\n", #exp, __FILE__, __LINE__ ); exit(1);}
#define ASSERT_FAIL() {printf("failure occurred in file %s at line %i\n", __FILE__, __LINE__ ); exit(1);}
