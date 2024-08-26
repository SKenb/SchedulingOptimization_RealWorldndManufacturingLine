using JuMP, HiGHS, GLPK, Gurobi, Juniper, Ipopt, Cbc
using CSV, Tables, TypedTables
using FileIO, JLD2

include("../00_Common/common.jl")

# -------------------------------------------------------------------
# 
tD_minutes = 5;  # in min / time instance
tD_seconds = 60*tD_minutes; # in s / time instance
tD_hours = tD_minutes / 60;  # in h / time instance
N = Int(3*60 / tD_minutes); # 6 hours = 6*60 min / min / time instace = number of time instances


# -------------------------------------------------------------------
# If not stated otherwise massHoldUp reffers zo massHoldUpOut
#
taskNames = [:synthesis, :cryst, :filtration, :hme, :vacuum, :dcLine, :tabletPress]
eventNames = [:triggerFiltration, :triggerVacuumTransport]

synthesisFields = [:massFlowOut, :massHoldUp]
synthesisConst = Dict([
    (:massFlowOutMin, 3.33 * tD_seconds), #l / s <-- NOT sure about that maybe ml/s
    (:massFlowOutMax, 8.33 * tD_seconds), #l / s <-- NOT sure about that maybe ml/s
    (:startUpTime, 1),
    (:shutdownTime, 2),
])

# massFlowIntern - API containing material
crystFields = [:massFlowIntern, :massFlowInternPolymer, :massHoldUp]
crystConst = Dict([
    (:massFlowInternMin, 2.49*tD_minutes), # 0.3 ml/min -> 1,5 ml / n
    (:massFlowInternMax, 2.5*tD_minutes*1.25), # 26.56*tD_minutes), # 26.56 ml/min -> 132 ml / n
    (:massFlowInternPolymerMin, 0.38*tD_minutes), # TODO not used at the moment
    (:massFlowInternPolymerMax, 44.29*tD_minutes), # TODO not used at the moment
    (:massFlowInternTarget, 2.5*tD_minutes), # 2,5 ml/min
    (:startUpTime, Int(ceil(0 / tD_minutes))), # 15 min
    (:shutdownTime, Int(ceil(5 / tD_minutes))), # 5 min
])

filtrationFields = [:massInFiltration]
filtrationConst = Dict([
    (:massInFiltrationMax, 2500); # ml
    (:ratioGrammOverMilliLitre, 8/3600); # 3600 ml -> 8g [g/ml]
    (:filtTime, Int(ceil(20 / tD_minutes))); # min RT about 20 min
])

hmeFields = [:massHoldUpIn, :massFlowAPI, :massFlowPolymer, :massFlowIntern, :massHoldUp]
hmeConst = Dict([
    (:startUpTime, Int(ceil(5 / tD_minutes))), # 5 min
    (:shutdownTime, Int(ceil(5 / tD_minutes))), # 5 min
    (:massFlowAPIMin, 0), # Not rel.
    (:massFlowAPIMax, 80000), # Not rel.
    (:massFlowPolymerMin, 1.77*tD_hours*1000), # Recommended nominal poly. feed rate = 1.778 kg / h
    (:massFlowPolymerMax, 1.78*tD_hours*1000), # Recommended nominal poly. feed rate = 1.778 kg / h
    (:massFlowInternMin, 1.4*tD_hours*1000), # 1.4 kg/h * 1000 * h/n = g/n
    (:massFlowInternMax, 2.6*tD_hours*1000), # 2.6 kg/h
])

vacuumTransportFields = [:massInTransport]
vacuumTransportConst = Dict([
    (:massInVacuumTransportMax, 10000*tD_hours); # TODO at moment ref from hme 
    (:vacuumTransportTime, 1); # n
])

dcLineFields = [:massHoldUpIn, :massFlowAPI, :massFlowPolymer, :massFlowIntern, :massHoldUp]
dcLineConst = Dict([
    (:startUpTime, 0),
    (:shutdownTime, 0),
    (:massFlowAPIMin, 0.6*tD_hours*1000), # 0.6 kg/h
    (:massFlowAPIMax, 1.4*tD_hours*1000), # 1.4 kg/h
    (:massFlowPolymerMin, 2.4*tD_hours*1000), # 2.4 kg/h
    (:massFlowPolymerMax, 5.6*tD_hours*1000), # 5.6 kg/h
    (:massFlowInternMin, 0),
    (:massFlowInternMax, 8*tD_hours*1000), # Not rel (> 1.4+5.6 kg/h)
])

tabletPressFields = [:massFlowIntern, :massHoldUp]
tabletPressConst = Dict([
    (:startUpTime, 0),
    (:shutdownTime, 0),
    (:massFlowInternMin, 3*tD_hours*1000), # 3 kg/h related to DC Line
    (:massFlowInternMax, 7*tD_hours*1000), # 7 kg/h related to DC Line
    (:targetMassHoldUp, 1.8*1000), # 1.8 kg Target to keep the hopper level constant --> related to dcLine[massHoldUp]
    (:massOfTablet, .2*1000), # 200 mg - average mass of tablet
])

# -------------------------------------------------------------------
# 
model = Model(Gurobi.Optimizer);

@variables(model, begin
    tasksSwitchedOn[taskNames, 1:N], Bin
    tasksInOperation[taskNames, 1:N], Bin
    taskInOperationChanged[taskNames, 1:N]

    events[eventNames, 1:N], Bin

    synthesis[synthesisFields, 1:N]
    cryst[crystFields, 1:N]
    filtration[filtrationFields, 1:N]
    hme[hmeFields, 1:N]
    vacuum[vacuumTransportFields, 1:N]
    dcLine[dcLineFields, 1:N]
    tabletPress[tabletPressFields, 1:N]
end)

# -------------------------------------------------------------------
# Switch On/Off
#for taskName in taskNames
#    
#    start = timeConst[taskName][:startUpTime];
#    stop = timeConst[taskName][:shutdownTime];
#
#    @constraints(model, begin
#        [j=(start+1):(N-stop), k=(j-start):(j+stop)], tasksInOperation[taskName, j] => { tasksSwitchedOn[taskName, k] == 1 };
#        #[j=1:start], tasksInOperation[taskName, j] == 0;
#    end)
#end

# -------------------------------------------------------------------
# Synthesis

# synthesis - on/off
@constraints(model, begin
    [j=(synthesisConst[:startUpTime]+1):(N-synthesisConst[:shutdownTime]), k=(j-synthesisConst[:startUpTime]):(j+synthesisConst[:shutdownTime])], tasksInOperation[:synthesis, j] => { tasksSwitchedOn[:synthesis, k] == 1 };
    [j=1:synthesisConst[:startUpTime]], tasksInOperation[:synthesis, j] == 0;
    [j=(N-synthesisConst[:shutdownTime]+1):N], tasksInOperation[:synthesis, j] == 0;

    [j=1:(N-1)], taskInOperationChanged[:synthesis, j] == tasksInOperation[:synthesis, j] - tasksInOperation[:synthesis, (j+1)];
end)

# synthesis - constraints
@constraints(model, begin
    [j=1:N], tasksInOperation[:synthesis, j] => { synthesisConst[:massFlowOutMin] <= synthesis[:massFlowOut, j] <= synthesisConst[:massFlowOutMax] };
    [j=1:N], !tasksInOperation[:synthesis, j] => { synthesis[:massFlowOut, j] == 0 };
    [j=1:N], 0 <= synthesis[:massHoldUp, j] 
end)

# synthesis - mass flows/hold ups
@constraints(model, begin
    [j=1:N], synthesis[:massHoldUp, j] == 599 + sum(synthesis[:massFlowOut, 1:j] .- cryst[:massFlowIntern, 1:j])     
end)

# synthesis - start/end conditions
#fix(synthesis[:massHoldUp, 1], 0)

# -------------------------------------------------------------------
# Crystallization
# Antisolvent is added with fixed ratio
# TODO change to seperate mass flow in  +  add bounds accordingly

# Crystallization - on/off
@constraints(model, begin
    [j=(crystConst[:startUpTime]+1):(N-crystConst[:shutdownTime]), k=(j-crystConst[:startUpTime]):(j+crystConst[:shutdownTime])], tasksInOperation[:cryst, j] => { tasksSwitchedOn[:cryst, k] == 1 };
    [j=1:crystConst[:startUpTime]], tasksInOperation[:cryst, j] == 0;
    [j=(N-crystConst[:shutdownTime]+1):N], tasksInOperation[:cryst, j] == 0;

    [j=1:(N-1)], taskInOperationChanged[:cryst, j] == tasksInOperation[:cryst, j] - tasksInOperation[:cryst, (j+1)];
end)

# Crystallization - constraints
@constraints(model, begin
    [j=1:N], tasksInOperation[:cryst, j] => { crystConst[:massFlowInternMin] <= cryst[:massFlowIntern, j] <= crystConst[:massFlowInternMax] };
    [j=1:N], !tasksInOperation[:cryst, j] => { cryst[:massFlowIntern, j] == 0 };
    #[j=1:N], tasksInOperation[:cryst, j] => { crystConst[:massFlowInternPolymerMin] <= cryst[:massFlowInternPolymer, j] <= crystConst[:massFlowInternPolymerMin] };
    #[j=1:N], !tasksInOperation[:cryst, j] => { cryst[:massFlowInternPolymer, j] == 0 };
    [j=1:N], tasksInOperation[:cryst, j] => { cryst[:massFlowInternPolymer, j] == 9*cryst[:massFlowIntern, j] };
    [j=1:N], 0 <= cryst[:massHoldUp, j] 
end)

# Crystallization - mass flows/hold ups - fixed ratio 1 to 9 api to solvent (1 massFloeIntern = API + 9 Anti-solvent = 10*API)
@constraints(model, begin
    [j=1:N], cryst[:massHoldUp, j] == 1500 + 10*sum(cryst[:massFlowIntern, 1:j]) - sum(filtration[:massInFiltration, 1:j]) # + sum(cryst[:massFlowInternPolymer, 1:j])    
end)


# -------------------------------------------------------------------
# trigger Filtration
@constraints(model, begin    
    [j=1:(N-(filtrationConst[:filtTime])), k=1:filtrationConst[:filtTime]], events[:triggerFiltration, j] => { events[:triggerFiltration, j+k] == 0 };
    [j=(N-(filtrationConst[:filtTime])+1):N], events[:triggerFiltration, j] == 0 ;
end)

@constraints(model, begin    
    [j=1:N], events[:triggerFiltration, j] => { 0 <= filtration[:massInFiltration, j] <= filtrationConst[:massInFiltrationMax] };
    [j=1:N], !events[:triggerFiltration, j] => { filtration[:massInFiltration, j] == 0 };
end)


# -------------------------------------------------------------------
# HME

# HME - on/off
@constraints(model, begin
    [j=(hmeConst[:startUpTime]+1):(N-hmeConst[:shutdownTime]), k=(j-hmeConst[:startUpTime]):(j+hmeConst[:shutdownTime])], tasksInOperation[:hme, j] => { tasksSwitchedOn[:hme, k] == 1 };
    [j=1:hmeConst[:startUpTime]], tasksInOperation[:hme, j] == 0;
    [j=(N-hmeConst[:shutdownTime]+1):N], tasksInOperation[:hme, j] == 0;

    [j=1:(N-1)], taskInOperationChanged[:hme, j] == tasksInOperation[:hme, j] - tasksInOperation[:hme, (j+1)];
end)

# HME - constraints
@constraints(model, begin
    [j=1:N], 0 <= hme[:massHoldUpIn, j] 
    [j=1:N], 0 <= hme[:massHoldUp, j] 
    [j=1:N], tasksInOperation[:hme, j] => { hmeConst[:massFlowAPIMin] <= hme[:massFlowAPI, j] <= hmeConst[:massFlowAPIMax] };
    [j=1:N], tasksInOperation[:hme, j] => { hmeConst[:massFlowPolymerMin] <= hme[:massFlowPolymer, j] <= hmeConst[:massFlowPolymerMax] };
    [j=1:N], tasksInOperation[:hme, j] => { hmeConst[:massFlowInternMin] <= hme[:massFlowIntern, j] <= hmeConst[:massFlowInternMax] };
    [j=1:N], !tasksInOperation[:hme, j] => { hme[:massFlowAPI, j] == 0 };
    [j=1:N], !tasksInOperation[:hme, j] => { hme[:massFlowPolymer, j] == 0 };
    [j=1:N], !tasksInOperation[:hme, j] => { hme[:massFlowIntern, j] == 0 }; 
end)

# HME - mass flows/hold ups - fixed ratio of polymer over API 
@constraints(model, begin
    [j=(filtrationConst[:filtTime]+1):N], hme[:massHoldUpIn, j] == 5.56 + sum(filtrationConst[:ratioGrammOverMilliLitre] .* filtration[:massInFiltration, 1:(j-filtrationConst[:filtTime])]) - sum(hme[:massFlowAPI, 1:j])
    [j=1:N], hme[:massFlowPolymer, j] == 19*hme[:massFlowAPI, j] # API+MOISTURE+WATER (30% API) + 19:1 Poly:API Might change when the target/producing tablet changes
    [j=1:N], hme[:massFlowIntern, j] == hme[:massFlowAPI, j] + hme[:massFlowPolymer, j]
    [j=1:N], hme[:massHoldUp, j] == sum(hme[:massFlowIntern, 1:j] .- vacuum[:massInTransport, 1:j])
end)


# -------------------------------------------------------------------
# trigger vacuum transport
@constraints(model, begin    
    [j=1:(N-vacuumTransportConst[:vacuumTransportTime]), k=1:vacuumTransportConst[:vacuumTransportTime]], events[:triggerVacuumTransport, j] => { events[:triggerVacuumTransport, j+k] == 0 };
end)

@constraints(model, begin    
    [j=1:N], events[:triggerVacuumTransport, j] => { vacuum[:massInTransport, j] <= vacuumTransportConst[:massInVacuumTransportMax] };
    [j=1:N], !events[:triggerVacuumTransport, j] => { vacuum[:massInTransport, j] == 0 };
end)

# -------------------------------------------------------------------
# DC Line

# DCLine - on/off
@constraints(model, begin
    [j=(dcLineConst[:startUpTime]+1):(N-dcLineConst[:shutdownTime]), k=(j-dcLineConst[:startUpTime]):(j+dcLineConst[:shutdownTime])], tasksInOperation[:dcLine, j] => { tasksSwitchedOn[:dcLine, k] == 1 };
    [j=1:dcLineConst[:startUpTime]], tasksInOperation[:dcLine, j] == 0;
    [j=(N-dcLineConst[:shutdownTime]+1):N], tasksInOperation[:dcLine, j] == 0;

    [j=1:(N-1)], taskInOperationChanged[:dcLine, j] == tasksInOperation[:dcLine, j] - tasksInOperation[:dcLine, (j+1)];
end)

# DCLine - constraints
@constraints(model, begin
    [j=1:N], 0 <= dcLine[:massHoldUpIn, j] 
    [j=1:N], 0 <= dcLine[:massHoldUp, j] 
    [j=1:N], tasksInOperation[:dcLine, j] => { dcLineConst[:massFlowAPIMin] <= dcLine[:massFlowAPI, j] <= dcLineConst[:massFlowAPIMax] };
    [j=1:N], tasksInOperation[:dcLine, j] => { dcLineConst[:massFlowPolymerMin] <= dcLine[:massFlowPolymer, j] <= dcLineConst[:massFlowPolymerMax] };
    [j=1:N], tasksInOperation[:dcLine, j] => { dcLineConst[:massFlowInternMin] <= dcLine[:massFlowIntern, j] <= dcLineConst[:massFlowInternMax] };
    [j=1:N], !tasksInOperation[:dcLine, j] => { dcLine[:massFlowAPI, j] == 0 };
    [j=1:N], !tasksInOperation[:dcLine, j] => { dcLine[:massFlowPolymer, j] == 0 };
    [j=1:N], !tasksInOperation[:dcLine, j] => { dcLine[:massFlowIntern, j] == 0 }; 
end)

# DCLine - mass flows/hold ups - fixed ratio of excipient over API containing material = 4
@constraints(model, begin
    #[j=1:N], dcLine[:massHoldUpIn, j] == sum(vacuum[:massInFiltration, 1:j])  
    [j=(vacuumTransportConst[:vacuumTransportTime]+1):N], dcLine[:massHoldUpIn, j] == sum(vacuum[:massInTransport, 1:(j-vacuumTransportConst[:vacuumTransportTime])]) - sum(dcLine[:massFlowAPI, 1:j])
    [j=1:N], dcLine[:massFlowPolymer, j] == 4*dcLine[:massFlowAPI, j] # Might change when the target/producing tablet changes
    [j=1:N], dcLine[:massFlowIntern, j] == dcLine[:massFlowAPI, j] + dcLine[:massFlowPolymer, j]
    [j=1:N], dcLine[:massHoldUp, j] == tabletPressConst[:targetMassHoldUp] + sum(dcLine[:massFlowIntern, 1:j] .- tabletPress[:massFlowIntern, 1:j]) # hopper level at start already filled to target
end)


# -------------------------------------------------------------------
# Tablet press

# Tablet press - on/off
@constraints(model, begin
    [j=(tabletPressConst[:startUpTime]+1):(N-tabletPressConst[:shutdownTime]), k=(j-tabletPressConst[:startUpTime]):(j+tabletPressConst[:shutdownTime])], tasksInOperation[:tabletPress, j] => { tasksSwitchedOn[:tabletPress, k] == 1 };
    [j=1:tabletPressConst[:startUpTime]], tasksInOperation[:tabletPress, j] == 0;
    [j=(N-tabletPressConst[:shutdownTime]+1):N], tasksInOperation[:tabletPress, j] == 0;

    [j=1:(N-1)], taskInOperationChanged[:tabletPress, j] == tasksInOperation[:tabletPress, j] - tasksInOperation[:tabletPress, (j+1)];
end)

# Tablet press - constraints
@constraints(model, begin
    [j=1:N], 0 <= tabletPress[:massHoldUp, j] 
    [j=1:N], tasksInOperation[:tabletPress, j] => { tabletPressConst[:massFlowInternMin] <= tabletPress[:massFlowIntern, j] <= tabletPressConst[:massFlowInternMax] };
    [j=1:N], !tasksInOperation[:tabletPress, j] => { tabletPress[:massFlowIntern, j] == 0 };
end)

# Tablet press - mass flows/hold ups
@constraints(model, begin
    [j=1:N], tabletPress[:massHoldUp, j] == sum(tabletPress[:massFlowIntern, 1:j])
end)

# -------------------------------------------------------------------
# Inital vaulues and additional constraints
for taskName in taskNames 
    #fix(tasksInOperation[taskName, 1], 0)
    #fix(tasksSwitchedOn[taskName, 1], 0)
    fix(tasksInOperation[taskName, end], 0)
end

for eventName in eventNames
    fix(events[eventName, end], 0)
end


# -------------------------------------------------------------------
#

@objective(model, Min,
    + 1e-2*synthesis[:massHoldUp, N]
    + 1e-2*cryst[:massHoldUp, N]
    + 1e2*hme[:massHoldUpIn, N]
    + 1e-2*hme[:massHoldUp, N]
    + 1e-2*dcLine[:massHoldUpIn, N]
    + 1e-2*dcLine[:massHoldUp, N]
    - 1e3*tabletPress[:massHoldUp, N]
    + 1e1*sum(tasksSwitchedOn[:, :]) / N
    + 1e-2*sum(events[:triggerFiltration, :]) / N
    + 1e-2*sum(events[:triggerVacuumTransport, :]) / N
    + 1e3*sum((taskInOperationChanged[:, :]).^2) / N
    + 1e5*sum((taskInOperationChanged[:hme, :]).^2)
    #- 1e2*tabletPress[:massHoldUp, N]
    #+ 1*(synthesis[:massHoldUp, N])^2
    #+ 1*(cryst[:massHoldUp, N])^2
    #+ 1*(hme[:massHoldUpIn, N])^2
    #+ 1e1*(hme[:massHoldUp, N])^2
    #+ 1*(dcLine[:massHoldUp, N])^2
    #+ 1*(dcLine[:massHoldUpIn, N])^2
    + 1e2*sum((dcLine[:massHoldUp, :] .- tabletPressConst[:targetMassHoldUp]).^2) # optimize for keeping the hopper level of the tablet press const
    #+ 1*sum((cryst[:massFlowIntern, :] .- crystConst[:massFlowInternTarget]).^2) # optimize for keeping trying to keep the flow rate to the target
)

# -------------------------------------------------------------------
#
set_optimizer_attribute(model, "MIPGap", .01)
set_optimizer_attribute(model, "TimeLimit", 1*60*60)
#set_optimizer_attribute(model, "NonConvex", 2)

optimize!(model)
solution_summary(model)


# -------------------------------------------------------------------
# Plots
timeInstanceToHours = tD_hours;
timeOffsetInHours = 3;

plotOperationSynthesis = addPlot(1:N, [
    wrapPlotDetails(tasksSwitchedOn[:synthesis, :], "Syn.: switch"),
    wrapPlotDetails(tasksInOperation[:synthesis, :], "Syn.: operation")
], timeInstanceToHours, "OFF/ON", true, true, :topright)

plotOperationCryst = addPlot(1:N, [
    wrapPlotDetails(tasksSwitchedOn[:cryst, :], "Cryst.: switch"),
    wrapPlotDetails(tasksInOperation[:cryst, :], "Cryst.: operation")
], timeInstanceToHours, "OFF/ON", true, true, :topright)

plotOperationHME = addPlot(1:N, [
    wrapPlotDetails(tasksSwitchedOn[:hme, :], "HME: switch"),
    wrapPlotDetails(tasksInOperation[:hme, :], "HME: operation")
], timeInstanceToHours, "OFF/ON", true, true, :topright)

plotOperationDCLine = addPlot(1:N, [
    wrapPlotDetails(tasksSwitchedOn[:dcLine, :], "DCL: switch"),
    wrapPlotDetails(tasksInOperation[:dcLine, :], "DCL: operation")
], timeInstanceToHours, "OFF/ON", true, true)

plotOperationTabletPress = addPlot(1:N, [
    wrapPlotDetails(tasksSwitchedOn[:tabletPress, :], "TP: switch"),
    wrapPlotDetails(tasksInOperation[:tabletPress, :], "TP: operation")
], timeInstanceToHours, "OFF/ON", true, true)

plotMassHoldUps = addPlot(1:N, [
    wrapPlotDetails(synthesis[:massHoldUp, :], "Syn. in l"),
    wrapPlotDetails(cryst[:massHoldUp, :], "Cryst. in l")
], timeInstanceToHours, "mass hold up", false, false, :topright, 1e-3)

plotMassHoldUps2 = addPlot(1:N, [
    wrapPlotDetails(hme[:massHoldUpIn, :], "HME - In in kg"),
    wrapPlotDetails(hme[:massHoldUp, :], "HME in kg"),
    wrapPlotDetails(dcLine[:massHoldUpIn, :], "dcLine - In in kg"),
    wrapPlotDetails(dcLine[:massHoldUp, :], "dcLine in kg"),
    wrapPlotDetails(tabletPress[:massHoldUp, :], "TP in kg")
], timeInstanceToHours, "mass hold up", false, false, :topleft, 1e-3)

plotMassHoldUps2Detailed = addPlot(1:N, [
    wrapPlotDetails(hme[:massHoldUpIn, :], "HME - In in kg"),
    wrapPlotDetails(hme[:massHoldUp, :], "HME in kg"),
    wrapPlotDetails(dcLine[:massHoldUpIn, :], "dcLine - In in kg")
], timeInstanceToHours, "mass hold up", false, false, :topleft, 1e-3)

plotMassFlows = addPlot(1:N, [
    wrapPlotDetails(synthesis[:massFlowOut, :]./tD_hours, "Syn. in l/s"),
    wrapPlotDetails(cryst[:massFlowIntern, :]./tD_hours, "Cryst. - intern in l/min"),
    wrapPlotDetails(filtration[:massInFiltration, :], "Filt. in l"),
    wrapPlotDetails(vacuum[:massInTransport, :], "Vac.T. in kg")
], timeInstanceToHours, "mass flows", false, false, :topright, 1e-3)

plotMassFlowsHME = addPlot(1:N, [
    wrapPlotDetails(hme[:massFlowAPI, :]./tD_hours, "HME API in kg/h"),
    wrapPlotDetails(hme[:massFlowPolymer, :]./tD_hours, "HME Polymer in kg/h"),
    wrapPlotDetails(hme[:massFlowIntern, :]./tD_hours, "HME Intern in kg/h"),
], timeInstanceToHours, "mass flows", false, false, :topleft, 1e-3)

plotMassFlowsDCLine = addPlot(1:N, [
    wrapPlotDetails(dcLine[:massFlowAPI, :]./tD_hours, "DC Line API in kg/h"),
    wrapPlotDetails(dcLine[:massFlowPolymer, :]./tD_hours, "DC Line Polymer in kg/h"),
    wrapPlotDetails(dcLine[:massFlowIntern, :]./tD_hours, "DC Line Intern in kg/h")
], timeInstanceToHours, "mass flows", false, false, :topleft, 1e-3)

plotEvents = addPlot(1:N, [
    wrapPlotDetails(events[:triggerFiltration, :], "trigger Filt."),
    wrapPlotDetails(events[:triggerVacuumTransport, :], "trigger VT")
], timeInstanceToHours, "OFF/ON", true, true, :topright)

plot(plotOperationSynthesis, plotOperationCryst, plotEvents, plotOperationHME, plotOperationDCLine, plotMassFlows, plotMassHoldUps, plotMassHoldUps2, layout=(8, 1), size=(600, 600))
savefig("./02_Images/AutoGenerated/all.pdf")
savefig("./02_Images/AutoGenerated/all.png")

plot(plotOperationSynthesis, plotOperationCryst, plotEvents, plotOperationHME, plotOperationDCLine, plotOperationTabletPress, layout=(6, 1), size=(800, 1200))
savefig("./02_Images/AutoGenerated/operations.pdf")
savefig("./02_Images/AutoGenerated/operations.png")

plot(plotMassHoldUps, plotMassHoldUps2, plotMassHoldUps2Detailed, plotMassFlows, plotMassFlowsHME, plotMassFlowsDCLine, layout=(6, 1), size=(800, 1200))
savefig("./02_Images/AutoGenerated/massHoldUps.pdf")
savefig("./02_Images/AutoGenerated/massHoldUps.png")

plot(plotMassHoldUps, plotMassHoldUps2, plotMassHoldUps2Detailed, plotMassFlows, plotMassFlowsHME, plotMassFlowsDCLine, layout=(6, 1), size=(800, 1200))


# Extract results an important data

CSV.write("output.csv", Table(
    time_minutes = (1:N)*tD_minutes,
    triggerFiltration = value.(events[:triggerFiltration, :]),
    triggerVacuumTransport = value.(events[:triggerVacuumTransport, :]),
    synthesisSwitchedOn = value.(tasksSwitchedOn[:synthesis, :]),
    crystSwitchedOn = value.(tasksSwitchedOn[:cryst, :]),
    hmeSwitchedOn = value.(tasksSwitchedOn[:hme, :]),
    dcLineSwitchedOn = value.(tasksSwitchedOn[:dcLine, :]),
    tabletPressSwitchedOn = value.(tasksSwitchedOn[:tabletPress, :]),
    synthesisInOperation = value.(tasksInOperation[:synthesis, :]),
    crystInOperation = value.(tasksInOperation[:cryst, :]),
    hmeInOperation = value.(tasksInOperation[:hme, :]),
    dcLineInOperation = value.(tasksInOperation[:dcLine, :]),
    tabletPressInOperation = value.(tasksInOperation[:tabletPress, :]),
    synthesisMassFlowOut = value.(synthesis[:massFlowOut, :])./tD_hours,
    synthesisMassHoldUp = value.(synthesis[:massHoldUp, :]),
    crystMassFlowIntern = value.(cryst[:massFlowIntern, :])./tD_minutes,
    crystMassFlowPolymer = value.(cryst[:massFlowInternPolymer, :])./tD_minutes,
    crystMassHoldUp = value.(cryst[:massHoldUp, :]),
    filtrationMassInFiltration = value.(filtration[:massInFiltration, :]),
    hmeMassHoldUpIn = value.(hme[:massHoldUpIn, :]),
    hmeMassFlowAPI = value.(hme[:massFlowAPI, :])./tD_hours/1000,
    hmeMassFlowPolymer = value.(hme[:massFlowPolymer, :])./tD_hours/1000,
    hmeMassFlowIntern = value.(hme[:massFlowIntern, :])./tD_hours/1000,
    hmeMassHoldUp = value.(hme[:massHoldUp, :]),
    vacuumMassInTransport = value.(vacuum[:massInTransport, :]),
    dcLineMassHoldUpIn = value.(dcLine[:massHoldUpIn, :]),
    dcLineMassFlowAPI = value.(dcLine[:massFlowAPI, :])./tD_hours/1000,
    dcLineMassFlowPolymer = value.(dcLine[:massFlowPolymer, :])./tD_hours/1000,
    dcLineMassFlowIntern = value.(dcLine[:massFlowIntern, :])./tD_hours/1000,
    dcLineMassHoldUp = value.(dcLine[:massHoldUp, :]),
    tabletPressMassFlowIntern = value.(tabletPress[:massFlowIntern, :])./tD_hours/1000,
    tabletPressTabletMassFlowIntern = value.(tabletPress[:massFlowIntern, :])./tD_hours/1000/0.0002, # kg/h over average mass of tablet
    tabletPressBlenderSpeed = value.(tabletPress[:massFlowIntern, :])./tD_hours/1000/0.0002*50/22000, # % ref 50% for 22000 tablets / h
    tabletPressMassHoldUp = value.(tabletPress[:massHoldUp, :]),
))