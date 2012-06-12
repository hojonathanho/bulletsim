#include <iostream>
#include <algorithm>
#include <vector>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
	for (int i=0; i<v.size(); i++) {
		os << v[i] << " ";
	}
	os.flush();
}

template <typename T>
T median(const std::vector<T>& v) {
	if (v.size() == 0) return 0;
	std::vector<T> v_sort(v);
	sort(v_sort.begin(), v_sort.end());
	return v_sort[(v_sort.size()-1)/2];
}

template <typename T>
int argMedian(const std::vector<T>& v) {
	T m = median(v);
	for (int i=0; i<v.size(); i++)
		if (v[i] == m) return i;
	return -1;
}
