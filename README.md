# MOSS: Map Optimization for Size and Saliency 

AprilSAM: Real-time Smoothing and Mapping
===================================================
Copyright APRIL Lab.

https://april.eecs.umich.edu/

--------------------
# Build the code
    install LCM: https://lcm-proj.github.io/
    git clone https://github.com/xipengwang/MOSS
    cd MOSS
    make

# Simulator
    ././MOSS-sim-manual -c ../config/simworld.config -f /tmp/sim.lcmlog -p 8888
    ./MOSS-sim -c ../config/simworld.config -g corridor-map.graph -c ../config/simworld-corridor.config -f /tmp/sim.lcmlog -p 8888
![alt text](https://april.eecs.umich.edu/public/users/xipengw/images/MOSS-sim.jpg)

# Mapping 
    ./MOSS-mapping -f /tmp/sim.lcmlog -p 8889
![alt text](https://april.eecs.umich.edu/public/users/xipengw/images/MOSS-mapping.jpg)

# Localization
    ./MOSS-localization -g corridor-map.graph -f /tmp/training-folder -p 8890 [--resume-file][--bootstrapping-file]
![alt text](https://april.eecs.umich.edu/public/users/xipengw/images/MOSS-localization.jpg)
