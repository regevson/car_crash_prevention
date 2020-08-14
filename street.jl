import Base.Threads.@spawn
using Makie
using DataStructures

include("car.jl")

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

function calc_normal_drive(p_x, p_y, a_x, a_y, θ, v, q)
	x_add = v/10
	for i = 1:0.1:8
		yaw_angle = 0.0
		if i >= 5
			yaw_angle = 0.5
		end
		enqueue!(q, (p_x+x_add, p_y, a_x, a_y, θ, v))
		x_add = x_add + v/10
	end
end

function paint_car(obs_car)

	obs1 = lift(x -> [x], obs_car[1])
	obs2 = lift(x -> [x], obs_car[2])
	obs3 = lift(x -> [x], obs_car[3])
	obs4 = lift(x -> [x], obs_car[4])
	scene = arrows!(obs1, obs2, obs3, obs4, linewidth = 44, arrowsize = 60)

end

function drive_car(car::Car, obs_car)

	for i = 1:0.1:10
		if i == 5 
			predict_trajectory(car, 3)
		end
		position_vector = get_drive_path(car)
		println(position_vector)
		obs_car[1][] = position_vector[1]
		obs_car[2][] = position_vector[2]
		obs_car[3][] = position_vector[3]
		obs_car[4][] = position_vector[4]
		sleep(0.1)
	end

end

# build car_1
q_drivepath_c1 = Queue{NTuple{6, Float64}}()
calc_normal_drive(25.0, 100.0, 30.0, 0.0, 0.5, 50, q_drivepath_c1)
obs_car1 = [Node(25.0), Node(100.0), Node(30.0), Node(0.0)]
car_1 = Car(q_drivepath_c1)
paint_car(obs_car1)

# build car_2
q_drivepath_c2 = Queue{NTuple{6, Float64}}()
calc_normal_drive(25.0, 400.0, 30.0, 0.0, 0.001, 30, q_drivepath_c2)
obs_car2 = [Node(500.0), Node(400.0), Node(-30.0), Node(0.0)]
car_2 = Car(q_drivepath_c2)
paint_car(obs_car2)

display(scene)
@spawn drive_car(car_1, obs_car1)
#drive_car(car_1, obs_car1)
drive_car(car_2, obs_car2)




