from pylab import *
import pylab

print "hi"

# from http://www.scipy.org/Cookbook/Matplotlib/LaTeX_Examples
fig_width_pt = 250  # Get this from LaTeX using \showthe\columnwidth
inches_per_pt = 1.0/72.27               # Convert pt to inch
ht_over_width=.4     # Aesthetic ratio
fig_width = fig_width_pt*inches_per_pt  # width in inches
fig_height = fig_width*ht_over_width      # height in inches
fig_size =  [fig_width,fig_height]
params = {'backend': 'ps',
          'axes.labelsize': 10,
          'text.fontsize': 10,
          'legend.fontsize': 8,
          'xtick.labelsize': 8,
          'ytick.labelsize': 8,
          'text.usetex': True,
          'figure.figsize': fig_size}
pylab.rcParams.update(params)

clf()
close('all')

colors = 'rgbcmyk'

nprob = len(problems)
nmeth = len(methods)


width = 1./(nprob*2)
fig = figure()
ax = fig.add_subplot(111)
for imeth in xrange(nmeth):
    ax.bar(arange(nprob)+imeth*width, performance[:,imeth]*100, width,color=colors[imeth], label=methods[imeth])
    
ax.set_xticks(arange(nprob)+width*nmeth/2.)
ax.set_xticklabels(problems)
#ax.set_yticks(arange(0,1.2,.2))
#ax.set_yticks(arange(0,10,1))
ax.set_ylabel("error (cm)")
xlo,xhi = ax.get_xlim()
ylo, yhi = ax.get_ylim()
ax.set_xlim(xlo, xhi)
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles, labels,loc=1,frameon=False, ncol=2)

pylab.savefig('bar_plots.pdf')
