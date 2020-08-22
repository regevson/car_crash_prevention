import Base.Threads.@spawn
include("car.jl")
using Makie
using DataStructures


time = 0

# building street
street_km = 0:30
lineA(x) = 0
lineB(x) = 5
lineC(x) = 10

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)


#=
function linearly_brake(car::CarModule.Car, end_velocity::Float64)
	vec = first(car.drive_path) # current state-vec

end
=#

function paint_car(obs_car::Array{Observable{Float64},1})

	obs1 = lift(x -> [x], obs_car[1])
	obs2 = lift(x -> [x], obs_car[2])
	obs3 = lift(x -> [x], obs_car[3])
	obs4 = lift(x -> [x], obs_car[4])
	scene = arrows!(obs1, obs2, obs3, obs4, linewidth = 40, arrowsize = 0)

end


# build car_1
ll_drivepath_c1 = CarModule.LL_State_Vec()
CarModule.calc_straight_drive((5.0, 0.0, 1.0, 0.0, 0.3, CarModule.ω, 38.0), ll_drivepath_c1)
obs_car1 = [Node(5.0), Node(0.0), Node(1.0), Node(0.0)]
car_1 = CarModule.Car(ll_drivepath_c1)
paint_car(obs_car1)

# build car_2
ll_drivepath_c2 = CarModule.LL_State_Vec()
CarModule.calc_straight_drive((5.0, 6.0, 1.0, 0.0, 0.0001, 0.0, 15.0), ll_drivepath_c2)
obs_car2 = [Node(5.0), Node(6.0), Node(1.0), Node(0.0)]
car_2 = CarModule.Car(ll_drivepath_c2)
paint_car(obs_car2)

display(scene)

CarModule.init(car_1, scene)
CarModule.init(car_2, scene)
#@spawn CarModule.observe_new_car(car_1, car_2, scene)
@spawn CarModule.observe_new_car(car_2, car_1, scene)

@spawn CarModule.drive_car(car_1, obs_car1, true)
CarModule.drive_car(car_2, obs_car2, false)



