import argparse
parser = argparse.ArgumentParser()
parser.add_argument("headers", nargs="*")
args = parser.parse_args()



import source_analysis as sa

classes = []
for header in args.headers:
    
    with open(header, "r") as fh:
        classes.extend(sa.find_classes(fh.read()))

    
print "#include <boost/shared_ptr.hpp>"
for name in classes:
    d = dict(name=name)
    print "class %(name)s;"%d
    print "typedef boost::shared_ptr<%(name)s> %(name)sPtr;"%d