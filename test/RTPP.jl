planner_name = "RTPP"
vehicle_name = "hmmwv"
cases = ["s1","s2","s3","s4","s5","s6"]
models =[:ThreeDOFv2, :KinematicBicycle2]

c = load(open(string(Pkg.dir("MAVs"),"/config/planner/",planner_name,".yaml")))
@testset "cases with (case=>$(case_name)) " for case_name in cases, model in models
  c["vehicle"] = load(open(string(Pkg.dir("MAVs"),"/config/vehicle/",vehicle_name,".yaml")))
  c["goal"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["goal"]
  c["X0"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["X0"]
  c["obstacle"] = load(open(string(Pkg.dir("MAVs"),"/config/case/",case_name,".yaml")))["obstacle"]
  setConfig(c,"misc";(:N=>40),(:model=>model),(:solver=>:Ipopt),(:integrationScheme=>:trapezoidal))
  fixYAML(c)   # fix messed up data types
  n = initializeAutonomousControl(c);
  simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)
  @test n.f.mpc.goalReached
end
