/* HdfUtil HDF5 utility functions     *
 * Developed by Fredrik Orderud, 2009 */
#include "hdfutil.hpp"
#include <math.h>
#include <fstream>

// auto-linking of HDF5 libraries
#ifdef _WIN32
#  ifdef _DEBUG
#    ifdef HDF5CPP_USEDLL
#      pragma comment(lib,"hdf5ddll.lib")
#      pragma comment(lib,"hdf5_cppddll.lib")
#    else
#      pragma comment(lib,"hdf5d.lib")
#      pragma comment(lib,"hdf5_cppd.lib")
#    endif
#  else
#    ifdef HDF5CPP_USEDLL
#      pragma comment(lib,"hdf5dll.lib")
#      pragma comment(lib,"hdf5_cppdll.lib")
#    else
#      pragma comment(lib,"hdf5.lib")
#      pragma comment(lib,"hdf5_cpp.lib")
#    endif
#  endif
#endif


/** Retrieves the dimensions of the dataset */
int hdfutil::GetDsDims (const H5::DataSet & dataset, int dims[3]) {
	H5::DataSpace dataspace = dataset.getSpace();
    bool simple = dataspace.isSimple();
    if (!simple)
        throw std::runtime_error("complex HDF5 dataspace");
	int rank = (int)dataspace.getSimpleExtentNdims();
	if (rank > 3)
        throw std::runtime_error("unsupported dimensionality");

    hsize_t h5_dims[3];
	dataspace.getSimpleExtentDims(h5_dims, NULL);
    for (int i = 0; i < rank; i++)
        dims[i] = (int)h5_dims[i];
    for (int i = rank; i < 3; i++)
        dims[i] =      -1;

    return rank;
}


std::string hdfutil::ReadString (const H5::CommonFG& group, const std::string & dsname) {
    const H5::DataSet & dataset = group.openDataSet(dsname);
	try {
        H5::DataType type = dataset.getDataType();
        std::string retval;
        retval.resize(type.getSize());
		dataset.read(&retval[0], dataset.getStrType());
        return retval;
	} catch (H5::Exception e) {
        std::string error = "Unable to ReadString.";// + dsname;
        throw std::runtime_error(error.c_str());
	}
}
void hdfutil::WriteString(const H5::CommonFG& group, const std::string & dsname, const std::string & str) {
    hsize_t dims[] = {1};
    H5::DataSpace dataspace(1, dims);       // 1 string
    H5::StrType   strtype  (0, str.size()); // string length
    H5::DataSet dset = group.createDataSet(dsname, strtype, dataspace, CreatePropList());
    dset.write(&str[0], strtype);
}


bool hdfutil::HasDataSet (const H5::H5File & h5file, const std::string & name) {
    hid_t loc_id = h5file.getLocId();
#if H5_VERS_MINOR >= 8
    hid_t dataset_id = H5Dopen1( loc_id, name.c_str());
#else
    hid_t dataset_id = H5Dopen( loc_id, name.c_str());
#endif
   if(dataset_id < 0)
      return false;

   H5Dclose(dataset_id);
   return true;
}


H5::DSetCreatPropList hdfutil::CreatePropList () {
	H5::DSetCreatPropList plist;
	hid_t dset_cplist = plist.getId();
	// disable time-stamping of datasets
	/*herr_t ret_val =*/ H5Pset_obj_track_times(dset_cplist, false);
    return plist;
}


bool hdfutil::FileExists (const std::string & filename) {
    std::ifstream testfile(filename.c_str());
    return testfile.is_open();
}
