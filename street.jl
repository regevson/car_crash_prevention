import Base.Threads.@spawn
include("car.jl")
#using Main.CarModule
using Makie
using DataStructures


time = 0

# building street
street_km = 0:500
lineA(x) = 0
lineB(x) = 250
lineC(x) = 500

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)

function calc_normal_drive(p_x::Float64, p_y::Float64, a_x::Float64, a_y::Float64, θ::Float64, v::Float64, q::CarModule.Queue_State_Vec)

	x_add = v/10

	for i = 1:0.1:13
		yaw_angle = 0.0
		if i >= 5
			yaw_angle = 0.5
		end
		enqueue!(q, (p_x+x_add, p_y, a_x, a_y, θ, v))
		x_add = x_add + v/10
	end

end

function paint_car(obs_car::Array{Observable{Float64},1})

	obs1 = lift(x -> [x], obs_car[1])
	obs2 = lift(x -> [x], obs_car[2])
	obs3 = lift(x -> [x], obs_car[3])
	obs4 = lift(x -> [x], obs_car[4])
	scene = arrows!(obs1, obs2, obs3, obs4, linewidth = 44, arrowsize = 60)

end

function drive_car(car::CarModule.Car, obs_car::Array{Observable{Float64},1}, cause_accident::Bool)

	for i = 1:0.1:10
		if i == 5 && cause_accident == true
			q = CarModule.predict_trajectory(car, 6)
			car.drive_path = q
		end
		position_vector = CarModule.get_drive_path(car)
		println(position_vector)
		obs_car[1][] = position_vector[1]
		obs_car[2][] = position_vector[2]
		obs_car[3][] = position_vector[3]
		obs_car[4][] = position_vector[4]
		sleep(0.1)
	end

end

# build car_1
q_drivepath_c1 = Queue{CarModule.State_Vec}()
calc_normal_drive(25.0, 100.0, 30.0, 0.0, 0.5, 50.0, q_drivepath_c1)
obs_car1 = [Node(25.0), Node(100.0), Node(30.0), Node(0.0)]
car_1 = CarModule.Car(q_drivepath_c1)
paint_car(obs_car1)

# build car_2
q_drivepath_c2 = Queue{CarModule.State_Vec}()
calc_normal_drive(25.0, 400.0, 30.0, 0.0, 0.0, 40.0, q_drivepath_c2)
obs_car2 = [Node(500.0), Node(400.0), Node(-30.0), Node(0.0)]
car_2 = CarModule.Car(q_drivepath_c2)
paint_car(obs_car2)

display(scene)

@spawn CarModule.observe_new_car(car_1, car_2)
CarModule.observe_new_car(car_2, car_1)

@spawn drive_car(car_1, obs_car1, true)
drive_car(car_2, obs_car2, false)




