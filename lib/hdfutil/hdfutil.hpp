/* HdfUtil HDF5 utility functions     *
 * Developed by Fredrik Orderud, 2009 */
#pragma once
#include <vector>
#include <string>
#include <stdexcept>
#ifdef HDFUTIL_USE_BOOST
#  pragma warning(push)
#  pragma warning(disable: 4127) // conditional expression is constant
#  pragma warning(disable: 4996) // 'std::copy': Function call with parameters that may be unsafe
#  include <boost/numeric/ublas/vector.hpp>
#  include <boost/numeric/ublas/matrix.hpp>
#  pragma warning(pop) // re-enable warning 4127
   using namespace boost::numeric;
#endif

#include <H5Cpp.h>

/** Common utility functions for accessing HDF5 data. */
namespace hdfutil {

// workaround for visual-studio/gcc differences in handling template specialization
#ifdef _WIN32
#  define TEMPLATE_STORAGE static
#else
#  define TEMPLATE_STORAGE
#endif

/** Internal type conversion structure for mapping to HDF5 datatypes. */
template<typename T>
static H5::DataType Type () {
#ifdef _WIN32
    BOOST_STATIC_ASSERT(false); // unsupported type
#else
    throw std::logic_error("Invalid H5 type");
#endif
}
template<>
TEMPLATE_STORAGE H5::DataType Type<unsigned char> () {
    return H5::PredType::NATIVE_UCHAR;
}
template<>
TEMPLATE_STORAGE H5::DataType Type<unsigned short> () {
    return H5::PredType::NATIVE_USHORT;
}
template<>
TEMPLATE_STORAGE H5::DataType Type<short> () {
    return H5::PredType::NATIVE_SHORT;
}
template<>
TEMPLATE_STORAGE H5::DataType Type<int> () {
    return H5::PredType::NATIVE_INT;
}
template<>
TEMPLATE_STORAGE H5::DataType Type<float> () {
    return H5::PredType::NATIVE_FLOAT;
}
template<>
TEMPLATE_STORAGE H5::DataType Type<double> () {
    return H5::PredType::NATIVE_DOUBLE;
}

/** Internal function. */
int GetDsDims (const H5::DataSet & dataset, int dims[3]);

/** Property list that disables time-stamping. */
H5::DSetCreatPropList CreatePropList ();


/** Read/write scalar values. */
template<typename T>
T ReadValue (const H5::CommonFG& group, const std::string & dsname) {
	T value = 0;
    const H5::DataSet & dataset = group.openDataSet(dsname);
	try {
        dataset.read(&value, Type<T>() );
	} catch (H5::Exception e) {
        throw std::runtime_error("Unable to ReadValue.");
	}
	return value;
}
template<typename T>
void WriteValue (H5::CommonFG& group, const std::string & dsname, const T & value) {
    hsize_t dims[] = {1};
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dset = group.createDataSet(dsname, Type<T>(), dataspace, CreatePropList());
    dset.write(&value, Type<T>() );
}


/** Read/write std::vector<T> objects. */
template<typename T>
std::vector<T> ReadArray (const H5::CommonFG& group, const std::string & dsname) {
	std::vector<T> array;
    const H5::DataSet & dataset = group.openDataSet(dsname);
	try {
        int dims[3];
        int rank = GetDsDims(dataset, dims);
		if (rank != 1)
			return array;

        array.resize((size_t)dims[0]);
        dataset.read(&array[0], Type<T>());
	} catch (H5::Exception e) {
        throw std::runtime_error("Unable to ReadArray.");
	}
	return array;
}
template<typename T>
void WriteArray (H5::CommonFG& group, const std::string & dsname, const std::vector<T> & array) {
    hsize_t dims[] = {array.size()};
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dset = group.createDataSet(dsname, Type<T>(), dataspace, CreatePropList());
    dset.write(&array[0], Type<T>() );
}

#ifdef HDFUTIL_USE_BOOST
/** Read ublas::vector<T> objects. */
template<typename T>
ublas::vector<T> ReadVector (const H5::CommonFG& group, const std::string & dsname) {
	ublas::vector<T> array;
    const H5::DataSet & dataset = group.openDataSet(dsname);
	try {
        int dims[3];
        int rank = GetDsDims(dataset, dims);
		if (rank != 1)
			return array;

        array.resize((size_t)dims[0]);
        dataset.read(&array[0], Type<T>());
	} catch (H5::Exception e) {
        throw std::runtime_error("Unable to ReadArray.");
	}
	return array;
}
template<typename T>
void WriteVector(const H5::CommonFG& group, const std::string & dsname, const ublas::vector<T> & array) {
    hsize_t dims[] = {array.size()};
    H5::DataSpace dataspace(1, dims);
    H5::DataSet dset = group.createDataSet(dsname, Type<T>(), dataspace, CreatePropList());
    dset.write(&array(0), Type<T>() );
}


/** Read/write ublas::matrix<T> objects. */
template<typename T>
ublas::matrix<T> ReadMatrix (const H5::CommonFG& group, const std::string & dsname) {
    ublas::matrix<T,ublas::row_major> matrix;
    const H5::DataSet & dataset = group.openDataSet(dsname);
	try {
        int dims[3];
        int rank = GetDsDims(dataset, dims);
		if (rank > 2 || rank < 1)
			return ublas::matrix<T>(0,0);
        if (rank == 2)
            matrix.resize((size_t)dims[0], (size_t)dims[1], false);
        else
            matrix.resize(1, (size_t)dims[0], false); // row-vector for 1D matrices

        // NOTICE: Assumes row-major matrices
		dataset.read(&matrix(0,0), Type<T>() );
	} catch (H5::Exception e) {
        throw std::runtime_error("Unable to ReadMatrix.");
	}
	return matrix;
}
template<typename T>
void WriteMatrix(const H5::CommonFG& group, const std::string & dsname, const ublas::matrix<T> & matrix) {
    hsize_t dims[] = {matrix.size1(), matrix.size2()};
    H5::DataSpace dataspace(2, dims);
    H5::DataSet dset = group.createDataSet(dsname, Type<T>(), dataspace, CreatePropList());
    dset.write(&matrix(0,0), Type<T>() );
}
#endif // HDFUTIL_USE_BOOST

/** Read/write text strings. */
std::string ReadString (const H5::CommonFG& group, const std::string & dsname);
void        WriteString(const H5::CommonFG& group, const std::string & dsname, const std::string & str);


/** Write N-dimensional table. */
template<typename T>
void ReadTable (const H5::CommonFG& group, const std::string & dsname, std::vector<size_t> & dims, std::vector<T> & data) {
    const H5::DataSet & dataset = group.openDataSet(dsname);
    {
        // parse dimensions
	    H5::DataSpace dataspace = dataset.getSpace();
        bool simple = dataspace.isSimple();
        if (!simple)
            throw std::runtime_error("complex HDF5 dataspace");
	    int rank = (int)dataspace.getSimpleExtentNdims();

        std::vector<hsize_t> long_dims(rank);
	    dataspace.getSimpleExtentDims(&long_dims[0]);
        dims.resize(rank);
        for (int i = 0; i < rank; i++)
            dims[i] = (int)long_dims[rank-1-i]; // flip dimensions
    }
    try {
        // compute size
        size_t N = (dims.empty() ? 0 : 1);
        for (size_t i = 0; i < dims.size(); i++)
            N *= dims[i];

        data.resize(N);
		dataset.read(&data[0], Type<T>() );
	} catch (H5::Exception e) {
        throw std::runtime_error("Unable to ReadTable.");
	}
}
/** Write N-dimensional table. */
template<typename T>
void WriteTable (const H5::CommonFG& group, const std::string & dsname, const std::vector<size_t> & dims, const std::vector<T> & data) {
    {
        // size check
        size_t N = (dims.empty() ? 0 : 1);
        for (size_t i = 0; i < dims.size(); i++)
            N *= dims[i];
        if (data.size() != N)
            throw std::runtime_error("WriteTable data size mismatch.");
        if (N == 0)
            return; // discard empty tables
    }
    // convert dimension array to long-long (64 bit)
    std::vector<hsize_t> long_dims(dims.size());
    for (size_t i = 0; i < dims.size(); i++)
        long_dims[i] = dims[dims.size()-1-i]; // flip dimensions

    H5::DataSpace dataspace(long_dims.size(), &long_dims[0]);
    H5::DataSet dset = group.createDataSet(dsname, Type<T>(), dataspace, CreatePropList());
    dset.write(&data[0], Type<T>());
}

/** Check whether a file has a given dataset. */
bool HasDataSet (const H5::H5File & h5file, const std::string & name);


/** Checks whether a file exists, and can be opened. */
bool FileExists (const std::string & filename);

} // namespace hdfutil
