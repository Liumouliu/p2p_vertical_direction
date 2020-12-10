# A minimal 2-point based absolute pose solver for a monocular or a multi-camera system

Using the vertical direction, the minimal 3D--2D correspondences needed to solve the absolute pose of the generalised camera is 2.

Although Chris Sweeney [1] et al. have solved the problem, by using a different solution procedure, a 20% speed-up has been achieved while maintaining exactly the same precision.

The implementation can be integrated within theia [2] or opengv [3] without much effort since only Eigen lib is used.

The documentation is available at [here](./PhD_thesis_ANU-pages-44-47.pdf).

[1] Sweeney C, Flynn J, Nuernberger B, et al. Efficient Computation of Absolute Pose for Gravity-Aware Augmented Reality[C]//Mixed and Augmented Reality (ISMAR), 2015 IEEE International Symposium on. IEEE, 2015: 19-24.

[2] http://www.theia-sfm.org/

[3] http://laurentkneip.github.io/opengv/

