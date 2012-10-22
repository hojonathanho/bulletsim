from pylab import *
import os

for pdf in ["robotrope.pdf", "humantowel.pdf", "humanrope.pdf"]:
    if os.path.exists(pdf): os.remove(pdf)

filename="robotrope.pdf"
problems = ["overhand", "figure-eight I", "figure-eight II"]
methods = ["no-color","color"]

#nprob x nmeth
performance = array([
    # overhand, f8a, f8b
    [.024,.034,.031], #nocolor
    [.021,.140,.114] #color
]).T
execfile("make_bar_plot.py")


filename="humantowel.pdf"
problems = ["diag-left", "diag-right", "double", "long", "short", "thirds"]
methods = ["one-cam/color","two-cam/no-color", "two-cam/color"]

#nprob x nmeth
performance = array([
    #diag-left #diag-right double long short triple
    [.014, .007, .014, .011, .012, .014], #one-cam/color
    [.007, .009, .013, .009, .007, .013], #two-cam/no-color
    [.007, .009, .007, .009, .007, .013] #two-cam/color
]).T
execfile("make_bar_plot.py")

filename="humanrope.pdf"
problems = ["overhand I","overhand II", "double-overhand", "figure8", "overhand+untie"]
methods = ["one-cam/no-color","two-cam/no-color", "two-cam/color"]

#nprob x nmeth
performance = array([
    #overhand, overhand-nat, double-voerhand, figure8, overhand+untie
    [.021, .028, .019, .021, .041], #one-cam/color
    [.063, .016, .064, .026, .016], #two-cam/no-color
    [.022, .013, .015, .022, .016], #two-cam/color
]).T
execfile("make_bar_plot.py")



