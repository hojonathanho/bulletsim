#pragma once
#include <iostream>
#include <string>
#include <vector>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
	os << "[ ";
	for (int i=0; i<v.size(); i++) {
		os << v[i];
		if (i != (v.size()-1))
			os << ", ";
	}
	os << " ]";
	os.flush();
}

template <typename T>
T min(const std::vector<T>& v) {
	assert(v.size() > 0);
	T result = v[0];
	for (int i=1; i<v.size(); i++)
		result = min(result, v[i]);
	return result;
}

template <typename T>
T max(const std::vector<T>& v) {
	assert(v.size() > 0);
	T result = v[0];
	for (int i=1; i<v.size(); i++)
		result = max(result, v[i]);
	return result;
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

template <typename T>
T mean(const std::vector<T>& v) {
	if (v.size() == 0) return 0;
	T result = 0;
	for (int i=0; i<v.size(); i++)
		result += v[i];
	result /= v.size();
	return result;
}

template <typename T>
std::vector<T> append(const std::vector<std::vector<T> >& v, int start, int end) {
	std::vector<T> res;
	for (int i=start; i<v.size() && i<end; i++)
		for (int j=0; j<v[i].size(); j++)
			res.push_back(v[i][j]);
	return res;
}

bool cwiseOr(const std::vector<bool>& v);

bool cwiseAnd(const std::vector<bool>& v);

std::string itoa(int n, int padding=0);
