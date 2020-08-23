using Makie
using AbstractPlotting: px

scene, layout = layoutscene(resolution = (1400, 900))

ax = layout[1, 1] = LAxis(scene)

xs = 0:0.5:10
ys = sin.(xs)
lin = lines!(ax, xs, ys, color = :blue)
sca = scatter!(ax, xs, ys, color = :red, markersize = 15)

leg = LLegend(scene, [lin, sca, [lin, sca]], ["a line", "some dots", "both together"])
layout[1, 2] = leg
display(scene)
