module CaseModule

using DataFrames

export
      Case,
      defineCase,
      setWeights!,
      setMisc!,
      setupResults,
      case2dfs,
      dataSet,
      Obs,
      defineObs,
      setSolverSettings!,
      defineTolerances

################################################################################
# Basic Types
################################################################################
############################### track info ########################################
type Track
  name
  func
  dir
  a
  b
  c
  y0
  Y
  X
end

function Track()
Track([],
      [],
      [],
      [],
      [],
      [],
      [],
      [],
      []);
end

function defineTrack(name)
  t=Track();
  if name==:path # test case for testPathFollowing.jl
    t.name=name;
    t.func=:poly;
    t.dir=:posY
    t.a=[0.0,0.0,2.33e-3,-6.43e-6,5.07e-9];
    t.Y=0:0.1:250;
    t.X=0:0.1:65;
  elseif name==:caseStudy  # TODO consider adding an @NLexpression for the path
    t.name=name;
    t.func=:fourier;
    t.dir=:posX
    dfs=dataSet(name,Pkg.dir("MAVs/examples/Cases/Tracks/"));
    t.a=dfs[1][:a][1:8];
    t.b=dfs[1][:b][1:8];
    t.c=dfs[1][:c][1:8];
    t.y0=dfs[1][:y0][1];
    t.Y=dfs[1][:Y];
    t.X=dfs[1][:X];
  end
  return t
end

############################### weights ########################################
type Weights
  goal  # should be zero when vehicle is not within goal distance during prediction horizon
  psi
  time
  haf
  Fz
  ce
  sa
  sr
  jx
  path
  driver
end

function Weights()
  Weights(100.,      # w_goal
          0.01,    # w_psi
          0.05,    # w_time
          1.0e-5,  # w_haf
          0.1,     # w_Fz  0.5 in paper
          1.,      # w_ce
          0.1,     # w_sa
          1.,      # w_sr
          0.01,    # w_jx
          1.0,     # path following
          1.0,     # follow driver steering angle
          );
end

function defineWeights(name)
  if name==:auto
    w=Weights();
  elseif name==:path
    w=Weights();
    w.sr=0.1;
    w.path=1.0;
    w.driver=1.0;
  elseif name==:m3

  elseif name!==:NA
    error("\n Pick a name for weights! \n")
  end
  return w
end

"""
setWeights!(c;sr=0.08,path=10.0,driver=0.5)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 4/12/2017, Last Modified: 4/12/2017 \n
--------------------------------------------------------------------------------------\n
"""
function setWeights!(c;
                      sr=c.w.sr,
                      path=c.w.path,
                      driver=c.w.driver
                      )
    c.w.sr=sr;
    c.w.path=path;
    c.w.driver=driver;
    return nothing
end

############################### tolerances ########################################
type Tolerances
    ix
    iy
    iv
    ir
    ipsi
    isa
    iu
    iax
    fx
    fy
    fv
    fr
    fpsi
    fsa
    fu
    fax

end

function Tolerances()
Tolerances(0.05,
0.05, # ix
0.05, # iy
0.05, # ir
0.01,
0.001,
0.05,
0.05,
1.0,   # 0.05 (m)small margin, if the vehicle is within this margin, then the target is considered to be reached
1.0,   # 0.05 (m)small margin, if the vehicle is within this margin, then the target is considered to be reached
NaN,
NaN,
NaN,
NaN,
NaN,
NaN
          );
end

function defineTolerances(name)
  if name==:auto
    tol = Tolerances();
  elseif name!==:NA
    error("\n Pick a name for tolerances! \n")
  end
  return tol
end

"""
setTolerances!(c;)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/16/2018, Last Modified: 2/16/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setTolerances!(c;
                      sr=c.w.sr,
                      path=c.w.path,
                      driver=c.w.driver
                      )
    c.w.sr=sr;
    c.w.path=path;
    c.w.driver=driver;
    return nothing
end

############################### solver settings ########################################
type SolverSettings
  #####################################
  # settings for both KNITRO and IPOPT
  ####################################
  outlev # (c.m.solver==:Ipopt) ? :print_level : :outlev # print level
  # (KNITRO) absolute stopping tolerance for the feasibility error
  # (Ipopt) Absolute tolerance on the constraint violation.
  feastol_abs # (c.m.solver==:Ipopt) ? :constr_viol_tol : :feastol_abs
  # (KNITRO) maximum number of iterations before termination
  # (Ipopt) Maximum number of iterations.
  maxit # (c.m.solver==:Ipopt) ? :max_iter : :maxit
  # (KNITRO) in seconds, the maximum allowable CPU time before termination
  # (Ipopt) A limit on CPU seconds that Ipopt can use to solve one problem
  maxtime_cpu # (c.m.solver==:Ipopt) ? :max_cpu_time : :maxtime_cpu
  #############################
  # settings for KNITRO
  ##########################
  ftol # relative change in the objective function is less than ftol for ftol_iters consecutive iterations
  feastol # (c.m.solver==:Ipopt) ? :empty  : :feastol  # relative stopping tolerance for the feasibility error
  ftol_iters # (c.m.solver==:Ipopt) ? :empty : :ftol_iters  # number of iters to stop after if change in obj fun is less than
  infeastol # (c.m.solver==:Ipopt) ? :empty : :infeastol # (relative) tolerance used for declaring infeasibility of a model
  maxfevals # (c.m.solver==:Ipopt) ? :empty :maxfevals  # maximum number of function evaluations before termination
  maxtime_real # (c.m.solver==:Ipopt) ? : :empty : :maxtime_real # in seconds, the maximum allowable real time before termination
  opttol # (c.m.solver==:Ipopt) ? :empty : :opttol  # final relative stopping tolerance for the KKT (optimality) error
  opttol_abs # (c.m.solver==:Ipopt) ? :empty : :opttol_abs # final absolute stopping tolerance for the KKT (optimality) error
  xtol # (c.m.solver==:Ipopt) ? :empty : :xtol # optimization process will terminate if the relative change in all components of the solution point estimate is less than xtol for xtol_iters
  ############################
  # settings for Ipopt
  ###########################
  warm_start_init_point  #
  dual_inf_tol   # Absolute tolerance on the dual infeasibility
  compl_inf_tol  # Absolute tolerance on the complementarity.
  acceptable_tol   # Determines which (scaled) overall optimality error is considered to be "acceptable.
  acceptable_constr_viol_tol   # Absolute tolerance on the constraint violation. "Acceptable" termination requires that the max-norm of the (unscaled) constraint violation is less than this threshold
  acceptable_dual_inf_tol   # Acceptable" termination requires that the (max-norm of the unscaled) dual infeasibility is less than this threshold
  acceptable_compl_inf_tol  # "Acceptable" termination requires that the max-norm of the (unscaled) complementarity is less than this threshold
  acceptable_obj_change_tol   # If the relative change of the objective function (scaled by Max(1,|f(x)|)) is less than this value, this part of the acceptable tolerance termination is satisfied
  diverging_iterates_tol   # If any component of the primal iterates exceeded this value (in absolute terms), the optimization is aborted
end

function SolverSettings()
  SolverSettings(:empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty,
  :empty        );
end

function defaultSolverSettings(name)

  s = SolverSettings()
  if typeof(name) ==  Array{Any,1}
    warn("Not defining solver settings in defineCase().
          Make sure that defaultSolverSettings() is called before optimization.\n  ")
  else
    if name==:KNITRO
      s.outlev = 0
      s.feastol_abs = 7e-2
      s.maxit = 500
      s.maxtime_cpu = 30.
      s.ftol = 1e-15
      s.feastol = 1.0e20
      s.ftol_iters = 5
      s.infeastol = 1e-2
      s.maxfevals = -1
      s.maxtime_real = 30.
      s.opttol = 1.0e20
      s.opttol_abs = 5e-1
      s.xtol = 1e-12
    elseif name==:Ipopt
      s.outlev = 0         # :print_level
      s.feastol_abs = 7e-2 # :constr_viol_tol (NOTE was = 1e-1)
      s.maxit = 500        # :max_iter
      s.maxtime_cpu = 30.  # :max_cpu_time
      s.acceptable_obj_change_tol=1e20
      s.warm_start_init_point="yes"
      s.dual_inf_tol = 5.
      s.compl_inf_tol = 1e-1
      s.acceptable_tol = 1e-2
      s.acceptable_constr_viol_tol = 0.01
      s.acceptable_dual_inf_tol = 1e10
      s.acceptable_compl_inf_tol = 0.01
      s.acceptable_obj_change_tol = 1e20
      s.diverging_iterates_tol = 1e20
    elseif name!==:NA
      error("\n Pick a name for solver: ", name, "is not defined. \n")
    end
  end

  return s
end

"""
setSolverSettings!(c;)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/16/2018, Last Modified: 2/16/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setSolverSettings!(c; kwargs... )
  kwargs = Dict(kwargs)
# TODO get ride of CaseModule or use better config methods
  # start with default settings
  c.s = defaultSolverSettings(c.m.solver)

  return nothing
end

#TODO make an Error: check for Float64 in iteration number  warning ERROR: LoadError: IPOPT: Couldn't set option 'max_iter' to value '500.0'.

############################### obstacle info ########################################
type Obs
  name
  A
  B
  s_x
  s_y
  X0
  Y0
  status
end

function Obs()
  Obs([],
            [],
            [],
            [],
            [],
            [],
            [],
            []);
end

"""
o=defineObs(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 7/5/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineObs(name)
  o=Obs();
  if name==:auto
    o.name=name;
    o.A=[5.];
    o.B=[5.];
    o.s_x=[0.0];
    o.s_y=[0.0];
    o.X0=[200.];
    o.Y0=[75.];
    o.status=falses(length(o.X0));
  elseif name==:autoBench
    o.name=name;
    o.A=[10.];
    o.B=[5.];
    o.s_x=[0.];
    o.s_y=[0.];
    o.X0=[200.];
    o.Y0=[43.];
    o.status=falses(length(o.X0));
  elseif name==:s1
    o.name=name;
    o.A=[1.] # 5
    o.B=[1.] # 5
    o.s_x=[0.]
    o.s_y=[0.]
    o.X0=[200.]
    o.Y0=[50.]
    o.status=falses(length(o.X0));
  elseif name==:s2
    o.name=name;
    o.A=[1.]
    o.B=[1.]
    o.s_x=[2.]
    o.s_y=[0.]
    o.X0=[190]
    o.Y0=[50]
    o.status=falses(length(o.X0));
  elseif name==:s3
    o.name=name;
    o.A=[5.,10.,2.]
    o.B=[5.,10.,2.]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[205.,180.,200.]
    o.Y0=[50.,75.,30.]
    o.status=falses(length(o.X0));
  elseif name==:s4
    o.name=name;
    o.A=[5,4,2]
    o.B=[5,4,2]
    o.s_x=[-2,-1,0]
    o.s_y=[0,1,4.5]
    o.X0=[205,180,200]
    o.Y0=[57,75,30]
    o.status=falses(length(o.X0));
  elseif name==:s5
    o.name=name;
    A=[2.5 1.5];      # small car
    B=[3.3 1.9];      # HUMMVEEs
    C=[9.8/2 3.65/2]; # tanks
    random=[0 0.6 1 0.25 0.6 0.5495 0.4852 0.8905 0.7990 1];
    o.X0=[225,225,220,170,220,165,200];
    o.Y0=[25,30,45,60,75,95,50];
    o.A=[B[1],B[1],B[1],C[1],A[1],C[1],C[2]];
    o.B=[B[2],B[2],B[2],C[2],A[2],C[2],C[1]];
    o.s_x=[-5,-5.5,-6,6,-5,4,-2];
    o.s_y=[0,0,0,0,0,0,5];
    o.status=falses(length(o.X0));
  elseif name==:s6
    o.name=name;
    o.A=[10,2,5,3]
    o.B=[10,2,5,3]
    o.s_x=[-3,4,-2,0]
    o.s_y=[-2,4,2.5,0]
    o.X0=[180,180,220,200]
    o.Y0=[75,40,60,100]
    o.status=falses(length(o.X0));
  elseif name==:autoGazebo
    o.name=name;
    o.A=[1.];
    o.B=[1.];
    o.s_x=[0.];
    o.s_y=[0.];
    o.X0=[200.];
    o.Y0=[50.];
    o.status=falses(length(o.X0));
  elseif name==:autoARC
    o.name=name;
    A=[2.5 1.5];      # small car
    B=[3.3 1.9];      # HUMMVEEs
    C=[9.8/2 3.65/2]; # tanks
    random=[0 0.6 1 0.25 0.6 0.5495 0.4852 0.8905 0.7990 1];
    o.X0=[225,225,220,170,220,165,200];
    o.Y0=[25,30,45,60,75,95,50];
    o.A=[B[1],B[1],B[1],C[1],A[1],C[1],C[2]];
    o.B=[B[2],B[2],B[2],C[2],A[2],C[2],C[1]];
    o.s_x=[-5,-5.5,-6,6,-5,4,-2];
    o.s_y=[0,0,0,0,0,0,5];
  elseif name==:path  # test case for testPathFollowing.jl
    o.name=name;
    o.A=[5.,4.,10.]
    o.B=[5.,5.,6.]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[7.0648272,17.37,39.98];
    o.Y0=[60.,130,173]
    o.status=falses(length(o.X0));
  elseif name==:o3  # test case for testSharedControl.jl
    o.name=name;
    o.A=[1.5,2.0,1.5]
    o.B=[1.5,2.0,1.5]
    o.s_x=[0.,0.,0.]
    o.s_y=[0.,0.,0.]
    o.X0=[0.0,-2.5,10.]
    o.Y0=[40.,30.,75.]
    o.status=falses(length(o.X0));
  elseif name==:o4  # test case for testPathFollowing.jl
    o.name=name;
    o.A=[5.,10.]
    o.B=[7.5,3.]
    o.s_x=[0.,0.]
    o.s_y=[0.,0.]
    o.X0=[17.,49.];
    o.Y0=[100.,200.];
    o.status=falses(length(o.X0));
  elseif name==:caseStudyPath # test case for testPathFollowing.jl
    o.name=:caseStudy;
    dfs=dataSet(o.name,Pkg.dir("MAVs/examples/Cases/Obstacles/"));
    o.X0=dfs[1][:X];
    o.Y0=dfs[1][:Y];
    o.A=dfs[1][:A];
    o.B=dfs[1][:B];
    o.status=dfs[1][:status];
    o.s_x=zeros(length(o.X0));
    o.s_y=zeros(length(o.X0));
    o.status=falses(length(o.X0));
  elseif name==:caseStudy # test case for testPathFollowing.jl
    o.name=:caseStudy;
    dfs=dataSet(o.name,Pkg.dir("MAVs/examples/Cases/Obstacles/"));
    o.X0=dfs[1][:X][1];
    o.Y0=dfs[1][:Y][1];
    o.A=dfs[1][:A][1];
    o.B=dfs[1][:B][1];
    o.status=dfs[1][:status][1];
    o.s_x=zeros(length(o.X0));
    o.s_y=zeros(length(o.X0));
    o.status=falses(length(o.X0));
  elseif name!==:NA
    error("\n Pick a name for obstacle data! \n")
  end
  return o
end

############################### goal info ########################################
type Goal
    name
    x_ref
    y_ref
    psi_ref
end

function Goal()
       Goal([],
            [],
            [],
            []);
end

"""
defineGoal(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineGoal(name)
  g=Goal()
  g.name=name
  if name==:auto
    g.x_ref=200.;
    g.y_ref=125.;
    g.psi_ref=pi/2;
  elseif name==:autoBench
    g.x_ref=200.;
    g.y_ref=100.;
    g.psi_ref=pi/2;
  elseif name==:autoGazebo
    g.x_ref=200.;
    g.y_ref=100.;
    g.psi_ref=pi/2;
  elseif name==:RTPP
    g.x_ref=200.;
    g.y_ref=125.;
    g.psi_ref=pi/2;
  elseif name==:path # test case for testPathFollowing.jl
    g.x_ref=65.0;
    g.y_ref=250.0;
    g.psi_ref=pi/2; #NA
  elseif name==:caseStudy
    g.x_ref=702.372760886434;
    g.y_ref=120.986699170193;
    g.psi_ref=pi/2;
  elseif name==:NA # g is already defined
  else
    error("\n Pick a name for goal data! \n")
  end
  return g
end

############################### misc info ########################################
type Misc
  name
  model              # function name
  X0                 # intial state (at the start of the simulation)
  Xlims              # limit for x
  Ylims              # limit for Y
  tp                 # prediction horizon (s)
  tex                # execution horizon (s)
  max_cpu_time       # maximum time the algorithm has
  sm                 # (m) distance to make sure we don't hit obstacle (for optimization)
  sm2                # (m) distance to make sure we don't hit obstacle (for crash checking)
  Lr                 # LiDAR range (m)
  L_rd               # relaxation distance to LiDAR range
  Nck                # number of points per interval
  N                  # number of points in :tm methods
  solver             # either :IPOPT or :KNITRO
  max_iter           # max evaluations in optimization
  mpc_max_iter       # an MPC parameter
  PredictX0          # if the state that the vehicle will be at when the optimization is finished is to be predicted
  FixedTp            # fixed final time for predicting X0
  activeSafety       # a bool if the system is only used to avoid collisions with obstacles
  followDriver       # a bool to try and follow the driver
  followPath         # a bool to follow the path
  NF                 # number to subtract off the constraint check vector-> used for activeSafety mode so driver sees obstacle before autonomy
  integrationScheme
  tfMax # maximum final time in MPC
end

function Misc()
       Misc([],
            [],
            [],
            [],
            [],
            0.0,
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            [],
            []
            );
end

"""
defineMisc(name)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/3/2017 \n
--------------------------------------------------------------------------------------\n
"""
function defineMisc(name)
  m=Misc();
  if name==:auto
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,7.0,0.0];
    m.Xlims=[-1., 400.]
    m.Ylims=[-1., 400.]
    m.tex=0.5;
    m.sm=2.0;
    m.Nck=[12,10,8,6];
    m.solver=:KNITRO;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=50.;
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoBench
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,15.0,0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.sm=5.0;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:Ipopt;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=150. # not in play
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoGazebo
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2, 0.0, 0.0, 0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.sm=5.0;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:Ipopt;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr= 50. # not in play
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:autoARC
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2,0.0,17.0,0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.sm=2.6;
    m.Nck=[10,8,6];#[12,10,8,6];
    m.solver=:KNITRO;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr=50.
    m.L_rd=5.;
    m.integrationScheme=:lgrExplicit
  elseif name==:RTPP
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[200.0, 0.0, 0.0, 0.0, pi/2, 0.0, 17.0, 0.0];
    m.Xlims=[111.,250.]
    m.Ylims=[-1., 140.]
    m.tex=0.5;
    m.max_cpu_time=300.;
    m.sm=4.5;
    m.sm2=3;
    m.mpc_max_iter=60;
    m.PredictX0=true;
    m.FixedTp=true;
    m.Lr= 50.
    m.L_rd=5.;
    m.tfMax = 10.
  elseif name==:path
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[0.0, 0.0, 0.0, 0.0, pi/2,0.0,10,0.0];
    m.Xlims=[-1., 100.]
    m.Ylims=[-1., 300.]
    m.tp=6.0;
    m.tex=0.5;
    m.sm=2.6;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
    m.integrationScheme=:lgrExplicit
  elseif name==:caseStudy
    m.name=name;
    m.model=:ThreeDOFv2;
    m.X0=[0.0, 0.0, 0.0, 0.0, 1.2037,0.0,10,0.0];
    m.Xlims=[-10., 750.]
    m.Ylims=[-10., 200.]
    m.tp=6.0;
    m.tex=0.3;
    m.sm=2.0;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.mpc_max_iter=600;
    m.PredictX0=true;
    m.FixedTp=true;
    m.activeSafety=true;
    m.followPath=false;
    m.followDriver=false;
    m.NF=0;
    m.integrationScheme=:lgrExplicit
  elseif name==:caseStudyPath # test case for testPathFollowing.jl with other model
    m.name=:caseStudy;
    m.model=:ThreeDOFv2;
    m.X0=[0.0,0.0, 0.0, 0.0,1.2037,0.0,10,0.0];
    m.Xlims=[-10., 730.]
    m.Ylims=[-10., 200.]
    m.tp=7.0;
    m.tex=0.3;
    m.sm=2.0;
    m.Nck=[10,8,6];
    m.solver=:KNITRO;
    m.mpc_max_iter=30;
    m.PredictX0=true;
    m.FixedTp=true;
    m.followPath=true;
    m.integrationScheme=:lgrExplicit
  elseif name!==:NA
    error("\n Pick a name for misc data! \n")
  end
  return m
end

"""
setMisc!(c;activeSafety=true,followPath=false,followDriver=false,predictX0=false,fixedTp=false,tp=5.0,tex=0.3,max_cpu_time=0.26,Ni=3,Nck=[10,8,6]);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/28/2017, Last Modified: 4/7/2017 \n
--------------------------------------------------------------------------------------\n
"""
function setMisc!(c;
                Xlims=c.m.Xlims,
                Ylims=c.m.Ylims,
                tp=c.m.tp,
                tex=c.m.tex,
                sm=c.m.sm,
                Lr=c.m.Lr,
                L_rd=c.m.L_rd,
                Nck=c.m.Nck,
                N=c.m.N,
                solver=c.m.solver,
                mpc_max_iter=c.m.mpc_max_iter,
                PredictX0=c.m.PredictX0,
                FixedTp=c.m.FixedTp,
                activeSafety=c.m.activeSafety,
                followPath=c.m.followPath,
                followDriver=c.m.followDriver,
                NF=c.m.NF,
                integrationScheme=c.m.integrationScheme);
    c.m.Xlims=Xlims;
    c.m.Ylims=Ylims;
    c.m.tp=tp;
    c.m.tex=tex;
    c.m.sm=sm;
    c.m.Lr=Lr;
    c.m.L_rd=L_rd;
    c.m.Nck=Nck;
    c.m.N=N;
    c.m.solver=solver;
    c.m.mpc_max_iter=mpc_max_iter;
    c.m.PredictX0=PredictX0;
    c.m.FixedTp=FixedTp;
    c.m.activeSafety=activeSafety;
    c.m.followPath=followPath;
    c.m.followDriver=followDriver;
    c.m.NF=NF;
    c.m.integrationScheme=integrationScheme;
    return nothing
end

################################################################################
# Model Class
################################################################################
abstract type AbstractCase end
type Case <: AbstractCase
 name
 g::Goal        # goal data
 o::Obs          # obstacle data
 w::Weights     # weight data
 tol::Tolerances # tolerance data
 s::SolverSettings  # solver settings
 t::Track       # track data
 m::Misc        # miscelaneous data
end

"""
c = Case();
# default constructor
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function Case()
 Case(Any,
      Goal(),
      Obs(),
      Weights(),
      Tolerances(),
      SolverSettings(),
      Track(),
      Misc()
    );
end

"""
# a basic example of autonomousControl
c=defineCase(;(mode=>:auto));


# a basic example of PathFollowing
c=defineCase(;(mode=>:path));


--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 2/16/2018 \n
--------------------------------------------------------------------------------------\n
"""
function defineCase(; kwargs...)

  kw = Dict(kwargs);
  if !haskey(kw,:mode); kw_ = Dict(:mode => :auto); mode = get(kw_,:mode,0);
  else; mode = get(kw,:mode,0);
  end

   c=Case();
   if mode==:auto
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoBench
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoGazebo
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
   elseif mode==:autoARC
     c.name=:auto;
     c.g=defineGoal(c.name);
     c.o=defineObs(:autoARC);
     c.w=defineWeights(c.name);
     c.t=defineTrack(:NA);
     c.m=defineMisc(:autoARC);
   elseif mode==:RTPP
     c.name=mode
     c.g=defineGoal(:RTPP);
     c.tol=defineTolerances(:auto);
     #c.o=defineObs(:autoGazebo);
     c.w=defineWeights(:auto);
     c.t=defineTrack(:NA);
     c.m=defineMisc(c.name);
     # c.s=defaultSolverSettings(:Ipopt)
   elseif mode==:path
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(c.name);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudy
     c.name=mode;
     c.g=defineGoal(c.name);
     c.o=defineObs(c.name);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(c.name);
   elseif mode==:caseStudyPath
     c.name=:caseStudy;
     c.g=defineGoal(c.name);
     c.o=defineObs(:caseStudyPath);
     c.w=defineWeights(:path);
     c.t=defineTrack(c.name);
     c.m=defineMisc(:caseStudyPath);
   else
     c.name=:user;
     c.g=defineGoal(goal);
     c.o=defineObs(obstacles);
     c.w=defineWeights(weights);
     c.t=defineTrack(:NA);
     c.m=defineMisc(misc);
   end

   return c
end

################################################################################
# data stuff
################################################################################
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 12/7/2016, Last Modified: 3/28/2017 \n
--------------------------------------------------------------------------------------\n
"""
function dataSet(set,path)
  main_dir=pwd();
  dfs=Vector{DataFrame}(1) # create am empty DataFrame
  cd(path)
  dfs[1]=readtable(string(set,".csv"))
  cd(main_dir)
  return dfs
end

"""
case2dfs(c)
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/11/2017, Last Modified: 2/15/2018 \n
--------------------------------------------------------------------------------------\n
"""

function case2dfs(c)
    dfs = DataFrame()

    ##############
    # weights
    dfs[:wgoal] = c.w.goal
    dfs[:wpsi] = c.w.psi # 0
    dfs[:wtime] = c.w.time
    dfs[:whaf] = c.w.haf # 0
    dfs[:wce] = c.w.ce
    dfs[:csa] = c.w.sa
    dfs[:wsr] = c.w.sr
    dfs[:wjx] = c.w.jx
    dfs[:path] = c.w.path
    dfs[:driver] = c.w.driver
    # weights
    ##############

    ##############
    # tolerances
    # initial state tolerances
    dfs[:ixt] = c.tol.ix
    dfs[:iyt] = c.tol.iy
    dfs[:ivt] = c.tol.iv
    dfs[:irt] = c.tol.ir
    dfs[:ipsit] = c.tol.ipsi
    dfs[:isat] = c.tol.isa
    dfs[:iut] = c.tol.iu
    dfs[:iaxt] = c.tol.iax

    # final state tolerances
    dfs[:fxt] = c.tol.fx
    dfs[:fyt] = c.tol.fy
    dfs[:fvt] = c.tol.fv
    dfs[:frt] = c.tol.fr
    dfs[:fpsit] = c.tol.fpsi
    dfs[:fsat] = c.tol.fsa
    dfs[:fut] = c.tol.fu
    dfs[:faxt] = c.tol.fax
    # tolerances
    ##################

    #####################
    # solver settings
    dfs[:outlev] = c.s.outlev
    dfs[:feastolAbs] = c.s.feastol_abs
    dfs[:maxit] = c.s.maxit
    dfs[:maxtimeCpu] = c.s.maxtime_cpu
    dfs[:ftol] = c.s.ftol
    dfs[:feastol] = c.s.feastol
    dfs[:infeastol] = c.s.infeastol
    dfs[:maxfevals] = c.s.maxfevals
    dfs[:maxtimeReal] = c.s.maxtime_real
    dfs[:opttol] = c.s.opttol
    dfs[:opttolAbs] = c.s.opttol_abs
    dfs[:xtol] = c.s.xtol
    dfs[:acceptableObjChangeTol] = c.s.acceptable_obj_change_tol
    dfs[:warmStartInitPoint] = c.s.warm_start_init_point
    dfs[:dualInfTol] = c.s.dual_inf_tol
    dfs[:acceptableTol] = c.s.acceptable_tol
    dfs[:acceptableConstrViolTol] = c.s.acceptable_constr_viol_tol
    dfs[:acceptableDualInfTol] = c.s.acceptable_dual_inf_tol
    dfs[:acceptableComplInfTol] = c.s.acceptable_compl_inf_tol
    dfs[:acceptableObjChangeTol] = c.s.acceptable_obj_change_tol
    dfs[:divergingIteratesTol] = c.s.diverging_iterates_tol
    # solver settings
    #####################

    ####################
    # obstacles
    dfs[:A] = string(c.o.A')
    dfs[:B] = string(c.o.B')
    dfs[:sX] = string(c.o.s_x')
    dfs[:sy] = string(c.o.s_y')
    dfs[:Xi] = string(c.o.X0')
    dfs[:Yi] = string(c.o.Y0')
    dfs[:status] = string(c.o.status')
    # obstacles
    ####################

    ###############
    # goal
    dfs[:xRef] = c.g.x_ref
    dfs[:yRef] = c.g.y_ref
    dfs[:psiRef] = c.g.psi_ref
    # goal
    ###############

    ###################
    # misc. parameters
    dfs[:model] = c.m.model
    dfs[:xi] = c.m.X0[1]
    dfs[:yi] = c.m.X0[2]
    dfs[:vi] = c.m.X0[3]
    dfs[:ri] = c.m.X0[4]
    dfs[:psii] = c.m.X0[5]
    dfs[:sai] = c.m.X0[6]
    dfs[:ui] = c.m.X0[7]
    dfs[:ax] = c.m.X0[8]
    dfs[:Xmax] = c.m.Xlims[1]
    dfs[:Xmin] = c.m.Xlims[2]
    dfs[:Ymin] = c.m.Ylims[1]
    dfs[:Ymax] = c.m.Ylims[2]
    dfs[:tp] = c.m.tp
    dfs[:tex] = c.m.tex
    dfs[:maxCPU] = c.m.max_cpu_time
    dfs[:sms] = c.m.sm
    dfs[:smh] = c.m.sm2
    dfs[:Lr] = c.m.Lr
    dfs[:Lrd] = c.m.L_rd

    if c.m.integrationScheme==:lgrExplicit || c.m.integrationScheme==:lgrImplicit
        dfs[:NI] = length(c.m.Nck)
        dfs[:colPts] = sum(c.m.Nck)
        dfs[:Nck] = string(c.m.Nck')
    else
        dfs[:NI] = NaN
        dfs[:colPts] = c.m.N
        dfs[:Nck] = NaN
    end
    dfs[:solver] = c.m.solver
    #dfs[:maxIter] = c.m.max_iter
    dfs[:MPCmaxIter] = c.m.mpc_max_iter
    dfs[:predictX0] = c.m.PredictX0
    dfs[:fixedTp] = c.m.FixedTp
    #dfs[:activeSafety] = c.m.activeSafety
    #dfs[:followDriver] = c.m.followDriver
    #dfs[:followPath] = c.m.followPath
    #dfs[:NF] = c.m.NF
    dfs[:integrationScheme] = c.m.integrationScheme
    dfs[:tfMax] = c.m.tfMax
    # misc. parameters
    ###################
    return dfs
end


"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/15/2018, Last Modified: 2/15/2018 \n
-------------------------------------------------------------------------------------\n
"""
function obs2dfs2(c)
  Q = length(c.o.A)
  A_data = DataFrame(ID = 1:Q, A = c.o.A);
  B_data = DataFrame(ID = 1:Q, B = c.o.B);
  s_x_data = DataFrame(ID = 1:Q, s_x = c.o.s_x);
  s_y_data = DataFrame(ID = 1:Q, s_y = c.o.s_y);
  X0_data = DataFrame(ID = 1:Q, X0 = c.o.X0);
  Y0_data = DataFrame(ID = 1:Q, Y0 = c.o.Y0);

  obs_data = join(A_data,B_data, on = :ID);
  obs_data = join(obs_data,s_x_data,on =:ID);
  obs_data = join(obs_data,s_y_data,on =:ID);
  obs_data = join(obs_data,X0_data,on =:ID);
  obs_data = join(obs_data,Y0_data,on=:ID);

  return obs_data
end

end # module
