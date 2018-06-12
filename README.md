# Michigan Autonomous Vehicles (MAVs)

[![Travis](https://travis-ci.org/JuliaMPC/MichiganAutonomousVehicles.jl.svg?branch=master)](https://travis-ci.org/JuliaMPC/MichiganAutonomousVehicles.jl)

julia packages:
```
Pkg.add("VehicleModels")
Pkg.add("NLOptControl")
Pkg.checkout("VehicleModels")  # to use master
Pkg.checkout("NLOptControl")
Pkg.clone("https://github.com/JuliaMPC/MAVs.jl")
```

To test:
```
Pkg.test("MichiganAutonomousVehicles")
```
