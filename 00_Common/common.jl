using Plots
#using LaTeXStrings

#p0 = addPlot(1:N, [
#    wrapPlotDetails(tanks[:tankAPI, :massHoldUp, :], "API-Tank: massHoldUp"),
#    wrapPlotDetails(tanks[:tankPolymer, :massHoldUp, :], "Polymer-Tank: massHoldUp")
#])
#
#p1 = addPlot(1:N, [
#    wrapPlotDetails(extruder[:omega, :], "Extruder: omega"),
#    wrapPlotDetails(extruder[:omega_dt, :], "Extruder: dOmega_dt")
#])
#
#p = plot(p0,p1,p2,p3,layout = (4,1))
#display(p)


function wrapPlotDetails(y, label)
    return Dict([(:y, y), (:label, label)])
end

function addPlot(x, plotDetailsList, timeInstanceToHours, ylabel, roundFlag=false, labelOffOn=false, legendPos=:topleft, yScale=1)
    ## Plots.resetfontsizes()
    #plot_font = "Computer Modern"
    #default(fontfamily=plot_font, linewidth=10, grid=true)
    #scalefontsizes(1.2)

    p = undef
    for plotDetails in plotDetailsList

        y = plotDetails[:y]
        if !isa(y, Array) y = Array(value.(y)) end

        time = x * timeInstanceToHours 

        #plot(rand(40),xtickfontsize=18,ytickfontsize=18,xlabel="wavelength",xguidefontsize=18,yscale=:log10,ylabel="flux",yguidefontsize=18,legendfontsize=18)


        # https://discourse.julialang.org/t/jump-gurobi-binary-variables-end-up-with-values-no-equal-1-or-0/65610/5
        if roundFlag==true y = round.(y) end

        if p != undef
            # https://www.matecdev.com/posts/julia-plotting-legend-position.html
            #plot!(p, time, yScale*y, linewidth=2, label=plotDetails[:label], legend=legendPos, xlabel="Time in h", ylabel=ylabel, legendfontsize=12, yguidefontsize=16)
            plot!(p, time, yScale*y, linewidth=2, label=plotDetails[:label], legend=legendPos, xlabel="Time in h", ylabel=ylabel)
        else
            #p = plot(time, yScale*y, linewidth=2, label=plotDetails[:label], legend=legendPos, xlabel="Time in h", ylabel=ylabel, legendfontsize=12, yguidefontsize=16)
            p = plot(time, yScale*y, linewidth=2, label=plotDetails[:label], legend=legendPos, xlabel="Time in h", ylabel=ylabel)
        end

        if labelOffOn==true
            yticks!([0, 1], ["OFF", "ON"])
        end
    end
    return p
end
