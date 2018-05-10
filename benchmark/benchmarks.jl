using MAVs, NLOptControl, VehicleModels

planner_name = "RTPP"
case_names = ["s1","s2","s3","s4","s5","s6"]
vehicle_name = "hmmwv"
models =[:ThreeDOFv2, :KinematicBicycle2]
model = models[1]

@benchgroup "cases" ["case"] begin
    for case_name in case_names
        c = load(open(string(Pkg.dir("MAVs"),"/config/planner/",planner_name,".yaml")))
        c["vehicle"] = load(open(string(Pkg.dir("MAVs"),"/config/vehicle/",vehicle_name,".yaml")))
        c["goal"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["goal"]
        c["X0"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["X0"]
        c["obstacle"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["obstacle"]
        setConfig(c, "misc";(:N=>15),(:model=>model),(:solver=>:KNITRO), (:integrationScheme=>:trapezoidal))
        fixYAML(c)   # fix messed up data types
        n = initializeAutonomousControl(c);
        simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)
        @bench case_name maximum($n.r.ocp.dfsOpt[:tSolve])
    end
end
