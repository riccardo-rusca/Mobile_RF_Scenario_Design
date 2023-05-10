# Mobile RF Scenario Design for Massive-Scale Wireless Channel Emulators

If you use the content of this repository, please reference the following paper: 

> R. Rusca, F. Raviglione, C. Casetti, P. Giaccone f. Restuccia, "Mobile RF Scenario Design for Massive-Scale Wireless Channel Emulators," pre-print version. [URL](https://hdl.handle.net/11583/2977750) [BibTeX](/cite.bib)

### Files Overview
* [*raytrace_mobile.m*](/raytrace_mobile.m) &rarr; contains the main code with the dataset import, channel matrix initialization and computation of others variables needed. For the Ray-tracing and Clustering step for vehicle and for each timestamp is called the raytrace_fcn inside the *raytrace_fcn.m* file.
* [*raytrace_fcn.m*](/raytrace_fcn.m) &rarr; contains the code for the ray-tracing and clustering.

### SAMARCANDA Mobility Scenario
Inside **SAMARCANDA_Scenario** folder you can find all the files needed to create a *Mobile RF Scenario* starting from the *SAMARCANDA* dataset available [here](https://github.com/francescoraves483/ms-van3t-CAM2CEM/raw/master/src/gps-tc/examples/GPS-Traces-Sample/SAMARCANDA_dataset.csv).  

More in detail:
* [*SAMARCANDA_area.png*](/SAMARCANDA_Scenario/SAMARCANDA_area.png) &rarr; contains a picture showing the SAMARCANDA traces of 19 vehicles on a map with 1 km^2 area highlighted in blue.
* [*SAMARCANDA_selected_area.png*](/SAMARCANDA_Scenario/SAMARCANDA_selected_area.png) &rarr; contains a zoomed picture showing the selected area for the creation of the RF Scenario with the position (static) of the antenna and some vehicles.
* [*cars_distribution.fig*](/SAMARCANDA_Scenario/cars_distribution.fig) &rarr; is a MATLAB figure that contains the distribution of cars inside the 1km^2 area selected for the scenario. For each timestamp on the y-axis there is the number of vehicles inside the selected area.
* [*raytrace_fcn.m*](/SAMARCANDA_Scenario/raytrace_fcn.m) and [*raytrace_mobile.m*](/SAMARCANDA_Scenario/raytrace_mobile.m) &rarr; contain the code for the creation of the channel matrix tailor made for the [Colosseum emulator](https://www.northeastern.edu/colosseum/) and specific for the SAMARCANDA dataset.

### Contacts
* Riccardo Rusca [riccardo.rusca@polito.it]
* Francesco Raviglione [francesco.raviglione@polito.it]
* Claudio Casetti [claudio.casetti@polito.it]
* Paolo Giaccone [paolo.giaccone@polito.it]
* Francesco Restuccia [f.restuccia@northeastern.edu]
