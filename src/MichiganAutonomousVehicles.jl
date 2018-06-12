isdefined(Base, :__precompile__) && __precompile__()

module MichiganAutonomousVehicles
using NLOptControl
using VehicleModels
import YAML.load

include("CaseModule.jl")
using .CaseModule

include("AutonomousControl.jl")
#include("SharedControl.jl")
#include("SimpleModel.jl")
#include("PathFollowing.jl")

using .AutonomousControl
#using .SharedControl
#using .SimpleModel
#using .PathFollowing

export

      # CaseModule.jl
      setConfig,
      case2dfs,

      # AutonomousControl.jl
      initializeAutonomousControl,
      updateAutoParams!,
      avMpc,
      solverConfig,
      fixYAML,

      load # from YAML

      # SharedControl.jl
    #  initializeSharedControl!,
    #  checkFeasibility!,
    #  sharedControl!,
    #  getPlantData!,
    #  sendOptData!,
    #  ExternalModel,

      # SimpleModel.jl
  #    initializeSimpleModel,
    #  updateSimpleModel,
  #    runSimpleModel,
  #    compareSimpleModels,
#      resetDesignVariables,

      # PathFollowing.jl
    #  initializePathFollowing,
    #  updatePathParams!
end
