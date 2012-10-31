#include <map>
#include <cstdio>
using namespace std;

#define MEMO_CALC(lhs, rhs, key) do {\
	static map<typeof(key), typeof(rhs)> cache;\
	map<typeof(key), typeof(rhs)>::iterator it = cache.find(key);\
	if (it != cache.end()) lhs = it->second;\
	else cache[key] = lhs = rhs;\
} while (0)	
	
	


double sq(double x) {
	
	printf("calling sq on %.2f\n", x);
	return x*x;
}

double memo_sq(double x) {
	double out;
	MEMO_CALC(out,sq(x), x);		
	return out;
}



int main() {
	memo_sq(1);
	memo_sq(2);
	memo_sq(1);

}		
	
		