# Sparse Iterative Closest Point Algorithm #

This repository contains an implementation of the [Sparse Iterative Closest Point](http://dl.acm.org/citation.cfm?id=2600305). It was implemented for the course [Nuage de Point et Modélisation](https://perso.telecom-paristech.fr/~boubek/ens/master/mva/npm/index.html) at Master [MVA](http://www.math.ens-cachan.fr/version-francaise/formations/master-mva/).

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.
For the record, here is the list of dependencies :

* [nanoflann](https://github.com/jlblancoc/nanoflann)
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main\_Page)

## Usage ##

When the program has been built thanks to CMake and Make, just run `./icpSparse -h` in order to see the help.

Other useful lines : 

* `./icpSparse -d` runs a demo with the files stored in the media directory
* `./icpSparse -i1 /path/to/first/objectFile.obj -i2 ../path/to/second/objectFile.obj -o /path/to/output/directory/ -pl` would be a minimal line in order to use the point-to-plane variant of the sparse ICP.

Notice that you should always specify if you want to use the point-to-plane variant (`-pl`) or the point-to-point variant (`-po`).

## Experiments ##

There is a python script in the [experiments directory](https://github.com/palanglois/icpSparse/tree/master/experiments) in order to reproduce the results presented in the report.

## Reference ##

Bouaziz, Sofien, Andrea Tagliasacchi, et Mark Pauly. « Sparse Iterative Closest Point ». In Proceedings of the Eleventh Eurographics/ACMSIGGRAPH Symposium on Geometry Processing, 113–123. SGP ’13. Aire-la-Ville, Switzerland, Switzerland: Eurographics Association, 2013. doi:10.1111/cgf.12178.

