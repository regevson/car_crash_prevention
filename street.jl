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


# build car_1 (threat-car)
ll_drivepath_c1 = CarModule.LL_State_Vec()
car1_color = Node("gray53")
car_1 = CarModule.Car(car1_color)

# stop and go
x_start = 4.0
CarModule.calc_drive(car_1, 0.1, (x_start, 6.0, 1.0, 0.0, 0.001, 0.001, 25.0), (0.5, CarModule.ω), ll_drivepath_c1, false, true, 6.0)

# cut in
#x_start = 11.0
#CarModule.calc_drive(car_1, 0.0, (x_start, 2.5, 1.0, 0.0, 0.001, 0.001, 22.0), (0.05, CarModule.ω), ll_drivepath_c1, true, false, 3.0)

# vel-test
# x_start = 0.0
#CarModule.calc_drive(car_1, 0.0, (x_start, 2.5, 1.0, 0.0, 0.001, 0.001, 30.0), (0.05, CarModule.ω), ll_drivepath_c1, true, false, 2.0)

car_1.drive_path = ll_drivepath_c1
obs_car1 = [Node(x_start), Node(6.0), Node(1.0), Node(0.0)]
CarModule.paint_car(obs_car1, car1_color, scene)

# build car_2 (ego-car)
ll_drivepath_c2 = CarModule.LL_State_Vec()
car2_color = Node("black")
car_2 = CarModule.Car(car2_color)

# vel-test
#CarModule.calc_drive(car_2, 6.0, (0.0, 6.0, 1.0, 0.0, 0.0, 0.0, 30.0), (0.0, 0.0), ll_drivepath_c2, false, true, -1.0)

# stop and go
CarModule.calc_drive(car_2, 8.0, (0.0, 6.0, 1.0, 0.0, 0.0, 0.0, 40.0), (0.0, 0.0), ll_drivepath_c2, false, false, -1.0)

# cut in
#CarModule.calc_drive(car_2, 8.0, (0.0, 6.0, 1.0, 0.0, 0.0, 0.0, 40.0), (0.0, 0.0), ll_drivepath_c2, false, false, -1.0)

car_2.drive_path = ll_drivepath_c2
obs_car2 = [Node(0.0), Node(6.0), Node(1.0), Node(0.0)]
CarModule.paint_car(obs_car2, car2_color, scene)


# create legend
black_car = [MarkerElement(color = :black, marker = :rect, strokecolor = :transparent)]
gray_car = [MarkerElement(color = :gray53, marker = :rect, strokecolor = :transparent)]
line_break = [MarkerElement(color = :transparent, marker = :circle, strokecolor = :transparent)]
gold_line = [LineElement(color = :gold3, linestyle = :solid, linewidth = 4)]
yellow_line = [LineElement(color = :yellow2, linestyle = :dot, linewidth = 6)]
orange_line = [LineElement(color = :orange, linestyle = :dot, linewidth = 6)]
red_line = [LineElement(color = :red3, linestyle = :dot, linewidth = 6)]
blue_line = [LineElement(color = :blue, linestyle = :dot, linewidth = 6)]
green_dot = [MarkerElement(color = :green, marker = :circle, strokecolor = :transparent)]
red_dot = [MarkerElement(color = :red, marker = :circle, strokecolor = :transparent)]
line_break = [MarkerElement(color = :transparent, marker = :circle, strokecolor = :transparent)]
yellow_dot = [MarkerElement(color = :yellow, marker = :circle, strokecolor = :transparent)]
purple_dot = [MarkerElement(color = :purple, marker = :circle, strokecolor = :transparent)]

leg = LLegend(ax, [black_car, gray_car, line_break, 
									 gold_line, yellow_line, orange_line, red_line, blue_line, green_dot, red_dot, 
									 line_break], 
			  ["Main-Car", "Threat-Car", " ", "Predicted-Trajectory", "Light Braking-Phase", "Medium Braking-Phase", 
				 "Hard Braking-Phase", "Acceleration-Phase", "Safety-Position", "Predicted-Crash-Location", 
				 " "])
layout[1, 2] = leg


display(ax)


CarModule.init(car_1, scene)
CarModule.init(car_2, scene)

#@spawn CarModule.observe_new_car(car_1, car_2, scene)
@spawn CarModule.observe_new_car(car_2, car_1, scene)

@spawn CarModule.drive_car(car_1, car_2, obs_car1, true, scene)
@spawn CarModule.drive_car(car_2, car_1, obs_car2, false, scene)



