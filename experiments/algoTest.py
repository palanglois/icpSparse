#!/usr/bin/python

import os

#Please add here the path to your icpSparse executable
execPath = '/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/icpSparse/build/icpSparse'

#Please add here yout path to the media directory
mediaPath = '/home/thefroggy/Documents/MVA/NuageDePointModelisation/projet/icpSparse/media/'

#Point to point
pc1s_po = ['bunny_side1.obj','bunny_noised_250_1.obj','bunny_noised_500_1.obj']
pc2s_po = ['bunny_side2.obj','bunny_noised_250_2.obj','bunny_noised_500_2.obj']
titles_po = ['0_outlier','250_outliers','500_outliers']
p_po = [0.2,0.5,0.8,1.5,2]

method = "-po"
for p in p_po:
  for i in range(len(titles_po)):
    name = 'p_'+str(p)+'_method'+method+'_'+titles_po[i]
    exec_line = execPath + ' -i1 ' + mediaPath + pc1s_po[i] + ' -i2 ' + mediaPath + pc2s_po[i] + ' -o . -n ' + name + ' ' + method + ' -p ' + str(p) + ' -n1 60'
    os.system(exec_line)
    print 'Obtained for point-to-point with p='+str(p)+' for the set of file with '+str(titles_po[i])+'\n'


#Point to plane
pc1s_pl = ['bunny_side1.obj','bunny_noised1.obj','bunny_noised_3000_1.obj','bunny_noised_4000_1.obj','bunny_noised_5000_1.obj']
pc2s_pl = ['bunny_side2.obj','bunny_noised2.obj','bunny_noised_3000_2.obj','bunny_noised_4000_2.obj','bunny_noised_5000_2.obj']
titles_pl = ['0_outlier','1000_outliers','3000_outliers','4000_outliers','5000_outliers']
p_pl = [0.2,0.5,0.8,1.5,2]

method = "-pl"
for p in p_pl:
  for i in range(len(titles_pl)):
    name = 'p_'+str(p)+'_method'+method+'_'+titles_pl[i]
    exec_line = execPath + ' -i1 ' + mediaPath + pc1s_pl[i] + ' -i2 ' + mediaPath + pc2s_pl[i] + ' -o . -n ' + name + ' ' + method + ' -p ' + str(p) + ' -n1 60'
    os.system(exec_line)
    print 'Obtained for point-to-plane with p='+str(p)+' for the set of file with '+str(titles_pl[i])+'\n'

