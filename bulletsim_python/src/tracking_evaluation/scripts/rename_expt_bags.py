from glob import glob
import os, shutil

os.chdir("/home/joschu/Data/tracking_results")
infiles=glob("*/track*/*.bag")
outfiles = [infile.replace("/","__") for infile in infiles]
for (inf, outf) in zip(infiles, outfiles):
    shutil.copy(inf,os.path.join('/home/joschu/Data/Experiments/',outf))
for (inf, outf) in zip(infiles, outfiles):
    shutil.copy(inf,os.path.join('/home/joschu/Data/Experiments/',outf))
