from comm.mapper import Mapper
import scipy.ndimage as ndi
from image_processing.ransac import ransac, ConstantModel

def remove_bad_rows(x):
    good_rows = np.isfinite(x).all(axis=1)
    return x[good_rows]

class GetRegionCenters(Mapper):
    def func(self, label_image, xyz_rgb):
        xyz_image = xyz_rgb[0]

        mask = (label_image == args.label).copy()
        labels, max_label = ndi.label(mask)
        counts = bincount(labels.flatten())
        good_labels = np.flatnonzero(counts > args.min_pix)[1:]
        good_labels_sorted = good_labels[counts[good_labels].argsort()[::-1]]
        good_labels_sorted = good_labels_sorted[:args.max_blobs]

        centers = []

        for i in good_labels_sorted:
            mask = labels == i
            xyzs_blob = remove_bad_rows(xyz_image[mask])
            model,_,_ = ransac(xyzs_blob, ConstantModel, minDataPts=1, nIter=10, threshold=.04, nCloseRequired=args.min_pix)
            if model is not None:
                centers.push_back(model.mean)
                            
        return centers

    def add_extra_arguments(self, parser):
        parser.add_argument('--label',type=int)
        parser.add_argument('--min_pix',type=int,default=10,nargs='?')
        parser.add_argument('--max_blobs',type=int,default=10,nargs='?')
        return parser


    

M = Mapper()
M.run()
