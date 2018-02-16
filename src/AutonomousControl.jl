module AutonomousControl

using NLOptControl
using VehicleModels

include("CaseModule.jl")
using .CaseModule

export
      initializeAutonomousControl,
      updateAutoParams!,
      avMpc


# TODO make sure that the user defined all aspects

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/15/2018, Last Modified: 2/15/2018 \n
-------------------------------------------------------------------------------------\n
"""
function solverConfig(c)
  # settings for both KNITRO and IPOPT
  outlev = (c.m.solver==:Ipopt) ? :print_level : :outlev
  feastol_abs = (c.m.solver==:Ipopt) ? :constr_viol_tol : :feastol_abs
  maxit = (c.m.solver==:Ipopt) ? :max_iter : :maxit
  maxtime_cpu = (c.m.solver==:Ipopt) ? :max_cpu_time : :maxtime_cpu
  #S1 = ((:name=>c.m.solver),(outlev=>c.s.outlev),(feastol_abs=>c.s.feastol_abs),(maxit=>c.s.maxit),(maxtime_cpu=>c.s.maxtime_cpu),(maxit=>c.s.maxit))
 if c.m.solver == :KNITRO
   SS=((:name=>c.m.solver),(outlev=>c.s.outlev),(feastol_abs=>c.s.feastol_abs),(maxit=>c.s.maxit),(maxtime_cpu=>c.s.maxtime_cpu),(maxit=>c.s.maxit),(:ftol=>c.s.ftol),(:feastol=>c.s.feastol),(:ftol_iters=>c.s.ftol_iters),(:infeastol=>c.s.infeastol),(:maxfevals=>c.s.maxfevals),(:opttol=>c.s.opttol),(:opttol_abs=>c.s.opttol_abs),(:xtol=>c.s.xtol))
 elseif c.m.solver == :Ipopt
   #SS=(S1,(:acceptable_obj_change_tol=>c.s.acceptable_obj_change_tol),(:warm_start_init_point=>c.s.warm_start_init_point),(:dual_inf_tol=>c.s.dual_inf_tol),(:compl_inf_tol=>c.s.compl_inf_tol),(:acceptable_tol=>c.s.acceptable_tol),(:acceptable_constr_viol_tol=>c.s.acceptable_constr_viol_tol),(:acceptable_dual_inf_tol=>c.s.acceptable_dual_inf_tol),(:acceptable_compl_inf_tol=>c.s.acceptable_compl_inf_tol),(:acceptable_obj_change_tol=>c.s.acceptable_obj_change_tol))
   SS=((:name=>c.m.solver),(outlev=>c.s.outlev),(feastol_abs=>c.s.feastol_abs),(maxit=>c.s.maxit),(maxtime_cpu=>c.s.maxtime_cpu),(maxit=>c.s.maxit),(:acceptable_obj_change_tol=>c.s.acceptable_obj_change_tol),(:warm_start_init_point=>c.s.warm_start_init_point),(:dual_inf_tol=>c.s.dual_inf_tol),(:compl_inf_tol=>c.s.compl_inf_tol),(:acceptable_tol=>c.s.acceptable_tol),(:acceptable_constr_viol_tol=>c.s.acceptable_constr_viol_tol),(:acceptable_dual_inf_tol=>c.s.acceptable_dual_inf_tol),(:acceptable_compl_inf_tol=>c.s.acceptable_compl_inf_tol),(:acceptable_obj_change_tol=>c.s.acceptable_obj_change_tol))
 end

return SS
end


"""
n=initializeAutonomousControl(c);
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/1/2017, Last Modified: 7/05/2017 \n
--------------------------------------------------------------------------------------\n
"""
function initializeAutonomousControl(c)
 pa=Vpara(x_min=c.m.Xlims[1],x_max=c.m.Xlims[2],y_min=c.m.Ylims[1],y_max=c.m.Ylims[2],sr_min=-0.18,sr_max=0.18);
 @unpack_Vpara pa
 XF=[c.g.x_ref, c.g.y_ref, NaN, NaN, NaN, NaN, NaN, NaN];
 XL=[x_min, y_min, NaN, NaN, psi_min, sa_min, u_min, NaN];
 XU=[x_max, y_max, NaN, NaN, psi_max, sa_max, u_max, NaN];
 CL = [sr_min, jx_min]; CU = [sr_max, jx_max];
 n=define(numStates=8,numControls=2,X0=copy(c.m.X0),XF=XF,XL=XL,XU=XU,CL=CL,CU=CU)
 n.s.tf_max=c.m.tfMax;
 n.params=[pa];   # vehicle parameters

 # set mpc parameters
 initializeMPC!(n;FixedTp=c.m.FixedTp,PredictX0=c.m.PredictX0,tp=c.m.tp,tex=copy(c.m.tex),max_iter=c.m.mpc_max_iter);
 n.mpc.X0=[copy(c.m.X0)];
 n.mpc.plantEquations=ThreeDOFv2;
 n.mpc.modelEquations=ThreeDOFv2;

 # define tolerances
 X0_tol=[c.tol.ix, c.tol.iy, c.tol.iv, c.tol.ir, c.tol.ipsi, c.tol.isa, c.tol.iu, c.tol.iax];
 XF_tol=[c.tol.fx, c.tol.fy, c.tol.fv, c.tol.fr, c.tol.fpsi, c.tol.fsa, c.tol.fu, c.tol.fax];
 defineTolerances!(n;X0_tol=X0_tol,XF_tol=XF_tol);

         # 1  2  3  4  5    6   7   8
 names = [:x,:y,:v,:r,:psi,:sa,:ux,:ax];
 descriptions = ["X (m)","Y (m)","Lateral Velocity (m/s)", "Yaw Rate (rad/s)","Yaw Angle (rad)", "Steering Angle (rad)", "Longitudinal Velocity (m/s)", "Longitudinal Acceleration (m/s^2)"];
 states!(n,names,descriptions=descriptions)

          # 1   2
 names = [:sr,:jx];
 descriptions = ["Steering Rate (rad/s)","Longitudinal Jerk (m/s^3)"];
 controls!(n,names,descriptions=descriptions)

 # dynamic constraints and additional constraints
 dx,con,tire_expr=ThreeDOFv2_expr(n)
 dynamics!(n,dx)
 constraints!(n,con)

 #c.s.maxtime_cpu = 300. # initially giving solver as much time as needed NOTE will not work properly
 # configure problem
 if isempty(c.s.outlev)
   error("Call setSolverSettings!(c) before initializeAutonomousControl(c) \n
          Note in the future setSolverSettings!(c) should be called in initializeAutonomousControl(c).\n
          But doing this in the REPEL creates a constructor issue. ")
end
 if c.m.integrationScheme==:lgrImplicit || c.m.integrationScheme==:lgrExplicit
   configure!(n;(:Nck=>c.m.Nck),(:integrationScheme=>c.m.integrationScheme),(:finalTimeDV=>true),(:solverSettings=>solverConfig(c)))
 else
   configure!(n;(:N=>c.m.N),(:integrationScheme=>c.m.integrationScheme),(:finalTimeDV=>true),(:solverSettings=>solverConfig(c)))
 end

 x=n.r.x[:,1];y=n.r.x[:,2];psi=n.r.x[:,5]; # pointers to JuMP variables

 #################################
 # obstacle aviodance constraints
 ################################
 Q = size(c.o.A)[1]; # number of obstacles
 @NLparameter(n.mdl, a[i=1:Q] == copy(c.o.A[i]));
 @NLparameter(n.mdl, b[i=1:Q] == copy(c.o.B[i]));
 @NLparameter(n.mdl, X_0[i=1:Q] == copy(c.o.X0[i]));
 @NLparameter(n.mdl, Y_0[i=1:Q] == copy(c.o.Y0[i]));
 @NLparameter(n.mdl, speed_x[i=1:Q] == copy(c.o.s_x[i]));
 @NLparameter(n.mdl, speed_y[i=1:Q] == copy(c.o.s_y[i]));
 obs_params=[a,b,X_0,Y_0,speed_x,speed_y,Q];

 # obstacle postion after the initial postion
 X_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], X_0[j] + speed_x[j]*n.tV[i]);
 Y_obs=@NLexpression(n.mdl, [j=1:Q,i=1:n.numStatePoints], Y_0[j] + speed_y[j]*n.tV[i]);

 # constraint on position
 obs_con=@NLconstraint(n.mdl, [j=1:Q,i=1:n.numStatePoints-1], 1 <= ((x[(i+1)]-X_obs[j,i])^2)/((a[j]+c.m.sm)^2) + ((y[(i+1)]-Y_obs[j,i])^2)/((b[j]+c.m.sm)^2));
 newConstraint!(n,obs_con,:obs_con);

 ####################
 # LiDAR constraints
 ###################
 # ensure that the final x and y states are near the LiDAR boundary
 @NLparameter(n.mdl, LiDAR_param_1==(c.m.Lr + c.m.L_rd)^2);
 @NLparameter(n.mdl, LiDAR_param_2==(c.m.Lr - c.m.L_rd)^2);
 LiDAR_edge_high = @NLconstraint(n.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  <= LiDAR_param_1);
 LiDAR_edge_low = @NLconstraint(n.mdl,[j=1], (x[end]-x[1])^2+(y[end]-y[1])^2  >= LiDAR_param_2);
 newConstraint!(n,LiDAR_edge_high,:LiDAR_edge_high);
 newConstraint!(n,LiDAR_edge_low,:LiDAR_edge_low);

# constrain all state points to be within LiDAR boundary
 LiDAR_range = @NLconstraint(n.mdl, [j=1:n.numStatePoints-1], (x[j+1]-x[1])^2+(y[j+1]-y[1])^2 <= (c.m.Lr + c.m.L_rd)^2 );
 newConstraint!(n,LiDAR_range,:LiDAR_range);

 if goalRange!(n,c)   # relax LiDAR boundary constraints
    setvalue(LiDAR_param_1, 1e6)
    setvalue(LiDAR_param_2,-1e6)
else                  # relax constraints on the final x and y position
    for st=1:2;for k in 1:2; setRHS(n.r.xf_con[st,k], 1e6); end; end
end
 LiDAR_params=[LiDAR_param_1,LiDAR_param_2]
 #####################
 # objective function
 ####################
 # parameters
 if goalRange!(n,c)
  println("\n goal in range")
  @NLparameter(n.mdl, w_goal_param == 0.0)
  @NLparameter(n.mdl, w_psi_param == 0.0)
 else
  @NLparameter(n.mdl, w_goal_param == c.w.goal)
  @NLparameter(n.mdl, w_psi_param == c.w.psi)
 end
 obj_params=[w_goal_param,w_psi_param]

 # penalize distance to goal
 goal_obj=@NLexpression(n.mdl,w_goal_param*((x[end] - c.g.x_ref)^2 + (y[end] - c.g.y_ref)^2)/((x[1] - c.g.x_ref)^2 + (y[1] - c.g.y_ref)^2 + EP))

 # penalize difference between final heading angle and angle relative to the goal NOTE currently this is broken becasue atan2() is not available
 #psi_frg=@NLexpression(n.mdl,asin(c.g.y_ref-y[end])/(acos(c.g.x_ref-x[end]) + EP) )
 #psi_obj=@NLexpression(n.mdl,w_psi_param*(asin(sin(psi[end] - psi_frg))/(acos(cos(psi[end] - psi_frg)) + EP) )^2 )
 psi_obj=0;
 # psi_obj=@NLexpression(n.mdl,w_psi_param*(asin(sin(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end])))/(acos(cos(psi[end] - asin(c.g.y_ref-y[end])/acos(c.g.x_ref-x[end]))) + EP) )^2 )

 # soft constraints on vertical tire load
 tire_obj=integrate!(n,tire_expr);

 # minimizing the integral over the entire prediction horizon of the line that passes through the goal
 #haf_obj=integrate!(n,:( $c.w.haf*( sin($c.g.psi_ref)*(x[j]-$c.g.x_ref) - cos($c.g.psi_ref)*(y[j]-$c.g.y_ref) )^2 ) )
 haf_obj=0;
 # penalize control effort
 ce_obj=integrate!(n,:($c.w.ce*($c.w.sa*(sa[j]^2)+$c.w.sr*(sr[j]^2)+$c.w.jx*(jx[j]^2))) )

 @NLobjective(n.mdl, Min, goal_obj + psi_obj + c.w.Fz*tire_obj + haf_obj + c.w.time*n.tf + ce_obj )

 #########################
 # initial optimization (s)
 ########################
 n.s.save=false; n.s.MPC=false; n.s.evalConstraints=false;
 if n.s.save
  warn("saving initial optimization results where functions where cashed!")
 end
 for k in 1:3
  optimize!(n);
  if n.r.status==:Optimal; break; end
 end

# defineSolver!(n,solverConfig(c)) # modifying solver settings NOTE currently not in use

         #  1      2          3          4
 n.params=[pa,obs_params,LiDAR_params,obj_params];

 n.s.save=true;
 # n.s.evalConstraints=true # NOTE can turn back on to investigate infeasibilities
 return n
end

"""

--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 3/20/2017, Last Modified: 7/05/2017 \n
--------------------------------------------------------------------------------------\n
"""
function updateAutoParams!(n,c)

  # obstacle information-> only show if it is in range at the start TODO
  goal_in_range = goalRange!(n,c)
  if goal_in_range # TODO make a flag that indicates this switch has been flipped
    println("goal is in range")

    # enforce final state constraints on x and y position
    for st=1:2;
      setRHS(n.r.xf_con[st,1], +(n.XF[st]+n.XF_tol[st]));
      setRHS(n.r.xf_con[st,2], -(n.XF[st]-n.XF_tol[st]));
    end

    # relax LiDAR constraints
    setvalue(n.params[3][1], 1e6)
    setvalue(n.params[3][2],-1e6)

    # remove terms in cost function
    setvalue(n.params[4][1],0.0)
    setvalue(n.params[4][2],0.0)
  end
  #NOTE assuming it is not going in and out of range

 return goal_in_range
end

"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/06/2018, Last Modified: 2/06/2018 \n
--------------------------------------------------------------------------------------\n
"""
function avMpc(c)
 n = initializeAutonomousControl(c);
 driveStraight!(n)
 for ii = 1:n.mpc.max_iter
     println("Running model for the: ",n.r.eval_num," time")
     updateAutoParams!(n,c)                        # update model parameters
     status = autonomousControl!(n)                # rerun optimization

     # if the vehicle is very close to the goal sometimes the optimization returns with a small final time
     # and it can even be negative (due to tolerances in NLP solver). If this is the case, the goal is slightly
     # expanded from the previous check and one final check is performed otherwise the run is failed
     if getvalue(n.tf) < 0.01 # assuming that the final time is a design variable, could check, but this module uses tf as a DV
       if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 4*n.XF_tol[1]
          println("Expanded Goal Attained! \n"); n.mpc.goal_reached=true;
          break;
      else
          warn("Expanded Goal Not Attained! -> stopping simulation! \n"); break;
      end
    end

     n.mpc.t0_actual = (n.r.eval_num-1)*n.mpc.tex  # external so that it can be updated easily in PathFollowing
     simPlant!(n)  # simulating out here even if it is not :Optimal so that we can look at final solution

     if n.r.status==:Optimal || n.r.status==:Suboptimal || n.r.status==:UserLimit
       println("Passing Optimized Signals to 3DOF Vehicle Model");
     elseif n.r.status==:Infeasible
       println("\n FINISH:Pass PREVIOUSLY Optimized Signals to 3DOF Vehicle Model \n"); break;
     else
       println("\n There status is nor Optimal or Infeaible -> create logic for this case!\n"); break;
     end
   if n.r.eval_num==n.mpc.max_iter
     warn(" \n This is the last itteration! \n i.e. the maximum number of iterations has been reached while closing the loop; consider increasing (max_iteration) \n")
   end
   if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.XF_tol[1]
      println("Goal Attained! \n"); n.mpc.goal_reached=true;
      break;
   end
   if checkCrash(n,c,c.m.sm2;(:plant=>true))
     warn(" \n The vehicle crashed -> stopping simulation! \n"); break;
   end
 end
 return n
end
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 7/04/2017, Last Modified: 7/04/2017 \n
--------------------------------------------------------------------------------------\n
"""
function goalRange!(n,c)
 return ( (n.mpc.X0[end][1] - c.g.x_ref)^2 + (n.mpc.X0[end][2] - c.g.y_ref)^2 )^0.5 < c.m.Lr
end

end # module
