#include "numpy_boost.hpp"

namespace detail {


template<>
const int numpy_type_map<float>::typenum = NPY_FLOAT;

template<>
const int numpy_type_map<std::complex<float> >::typenum = NPY_CFLOAT;

template<>
const int numpy_type_map<double>::typenum = NPY_DOUBLE;

template<>
const int numpy_type_map<std::complex<double> >::typenum = NPY_CDOUBLE;

template<>
const int numpy_type_map<long double>::typenum = NPY_LONGDOUBLE;

template<>
const int numpy_type_map<std::complex<long double> >::typenum = NPY_CLONGDOUBLE;

template<>
const int numpy_type_map<boost::int8_t>::typenum = NPY_INT8;

template<>
const int numpy_type_map<boost::uint8_t>::typenum = NPY_UINT8;

template<>
const int numpy_type_map<boost::int16_t>::typenum = NPY_INT16;

template<>
const int numpy_type_map<boost::uint16_t>::typenum = NPY_UINT16;

template<>
const int numpy_type_map<boost::int32_t>::typenum = NPY_INT32;

template<>
const int numpy_type_map<boost::uint32_t>::typenum = NPY_UINT32;

template<>
const int numpy_type_map<boost::int64_t>::typenum = NPY_INT64;

template<>
const int numpy_type_map<boost::uint64_t>::typenum = NPY_UINT64;

}