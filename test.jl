using Makie
x = 1:10
a = 1:10
scene = lines(x, a)

axis = scene[Axis]
axis[:ticks][
