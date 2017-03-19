# Experimentation tool

This tool reproduces the experiments carried out in the report.

To use it : do the following steps : 

* Open algoTest.py. Set execPath to the icpSparse executable file. Set mediaPath to be the absolute path of the media folder of this repository.
* Run `python algoTest.py`

The script produces for each step a .ply file which is the moved point cloud. There is also a txt file which summarizes the distance of the computed partial transformation at each step to the identity in order to watch for convergence.
