# HMMWV tests sets

@testset "HMMWV" begin
    # Test set #1
    planner_name = "RTPP"
    cases = ["s1","s2","s3","s4","s5","s6"]
    #models =[:ThreeDOFv2, :KinematicBicycle2]
    models =[:ThreeDOFv2]
    t = 1
    @testset "Test set #1, test #$(t) \n cases with (case=>$(case_name)) \n models with (model=>$(model))  " for case_name in cases, model in models
      c = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/planner/","RTPP",".yaml")))
      c["vehicle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/vehicle/","hmmwv",".yaml")))
      c["goal"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["goal"]
      c["X0"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["X0"]
      c["obstacle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["obstacle"]
      setConfig(c,"misc";(:N=>40),(:model=>model),(:solver=>:Ipopt),(:integrationScheme=>:trapezoidal))
      fixYAML(c)   # fix messed up data types
      n = initializeAutonomousControl(c);
      simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)
      @test n.f.mpc.goalReached
      t = t + 1
    end

    # Test set #2, constant speed
    planner_name = "RTPP"
    cases = ["s1","s2","s3"]
    models =[:ThreeDOFv2]
    t = 1
    @testset "Test set #2, test #$(t) \n cases with (case=>$(case_name)) \n models with (model=>$(model))" for case_name in cases, model in models
      c = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/planner/","RTPP",".yaml")))
      c["vehicle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/vehicle/","hmmwv",".yaml")))
      c["goal"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["goal"]
      c["X0"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["X0"]
      c["obstacle"] = load(open(string(Pkg.dir("MichiganAutonomousVehicles"),"/config/case/",case_name,".yaml")))["obstacle"]
      setConfig(c,"misc";(:N=>15),(:model=>model),(:solver=>:Ipopt),(:integrationScheme=>:trapezoidal),(:constantSpeed=>true))
      setConfig(c,"X0";(:ux=>7.0))# the type needs to be a float
      # otherwise, this will pop up: WARNING: Ipopt finished with status Invalid_Number_Detected
      fixYAML(c)   # fix messed up data types
      n = initializeAutonomousControl(c);
      simMPC!(n;updateFunction=updateAutoParams!,checkFunction=checkCrash)
      @test n.f.mpc.goalReached
      t = t + 1
    end

end
