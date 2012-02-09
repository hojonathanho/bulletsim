#include <utility>
#include <vector>
using namespace std;

typedef pair< vector<double>, vector<int> > ValuesInds;

ValuesInds getValuesInds(const vector<double>& joints); 
// gives the joint values and joint indices in the format that
// openrave requires
