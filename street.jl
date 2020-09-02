include("car.jl")

import Base.Threads.@spawn
using Makie
using DataStructures
using AbstractPlotting.MakieLayout


# building scene and layout
ax, layout = layoutscene(resolution = (1500, 500))
scene = layout[1, 1] = LAxis(ax)

# building street
street_km = 0:30
lineA(x) = 0
lineB(x) = 5
lineC(x) = 10

lin = lines!(scene, street_km, lineA, color = :black, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4, linestyle = :dash)
lines!(scene, street_km, lineC, color = :black, linewidth = 4)


function paint_car(obs_car::Array{Observable{Float64},1}, col::String)

	obs1 = lift(x -> [x], obs_car[1])
	obs2 = lift(x -> [x], obs_car[2])
	obs3 = lift(x -> [x], obs_car[3])
	obs4 = lift(x -> [x], obs_car[4])
	arrows!(scene, obs1, obs2, obs3, obs4, linewidth = 60, arrowsize = 0, linecolor= col)

end


# build car_1
ll_drivepath_c1 = CarModule.LL_State_Vec()
car_1 = CarModule.Car()
CarModule.calc_drive(car_1, 0.1, (5.0, 6.0, 1.0, 0.0, 0.001, 0.001, 60.0), (0.5, CarModule.ω), ll_drivepath_c1, false, true, 3.0)
car_1.drive_path = ll_drivepath_c1
obs_car1 = [Node(5.0), Node(6.0), Node(1.0), Node(0.0)]
paint_car(obs_car1, "gray53")

# build car_2
ll_drivepath_c2 = CarModule.LL_State_Vec()
car_2 = CarModule.Car()
CarModule.calc_drive(car_2, 6.0, (0.0, 6.0, 1.0, 0.0, 0.0, 0.0, 50.0), (0.0, 0.0), ll_drivepath_c2, false, false, -1.0)
car_2.drive_path = ll_drivepath_c2
obs_car2 = [Node(0.0), Node(6.0), Node(1.0), Node(0.0)]
paint_car(obs_car2, "black")


# create legend
black_car = [MarkerElement(color = :black, marker = :rect, strokecolor = :transparent)]
gray_car = [MarkerElement(color = :gray53, marker = :rect, strokecolor = :transparent)]
line_break = [MarkerElement(color = :transparent, marker = :circle, strokecolor = :transparent)]
orange_line = [LineElement(color = :orange, linestyle = nothing, linewidth = 4)]
blue_line = [LineElement(color = :blue, linestyle = :dot, linewidth = 4)]
green_dot = [MarkerElement(color = :green, marker = :circle, strokecolor = :transparent)]
red_dot = [MarkerElement(color = :red, marker = :circle, strokecolor = :transparent)]
line_break = [MarkerElement(color = :transparent, marker = :circle, strokecolor = :transparent)]
yellow_dot = [MarkerElement(color = :yellow, marker = :circle, strokecolor = :transparent)]
purple_dot = [MarkerElement(color = :purple, marker = :circle, strokecolor = :transparent)]

leg = LLegend(ax, [black_car, gray_car, line_break, orange_line, blue_line, green_dot, red_dot, line_break, yellow_dot, purple_dot], 
			  ["Main-Car", "Threat-Car", " ", "Predicted-Trajectory", "Braking-Phase", "Safety-Position", "Predicted-Crash-Location", 
				 " ", "Optimal Evation Trajectory", "Best Possible Evation Trajectory"])
layout[1, 2] = leg


display(ax)


CarModule.init(car_1, scene)
CarModule.init(car_2, scene)

#@spawn CarModule.observe_new_car(car_1, car_2, scene)
@spawn CarModule.observe_new_car(car_2, car_1, scene)

@spawn CarModule.drive_car(car_1, obs_car1)
@spawn CarModule.drive_car(car_2, obs_car2)



