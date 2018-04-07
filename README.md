# MAVs
Michigan Autonomous Vehicles


ros dependencies:
```
sudo apt-get install ros-kinetic-move-base
```

julia packages:
```
Pkg.add("VehicleModels")
Pkg.add("NLOptControl")
Pkg.checkout("VehicleModels")  # to use master
Pkg.checkout("NLOptControl")
Pkg.clone("https://github.com/JuliaMPC/MAVs.jl")
```
