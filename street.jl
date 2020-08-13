import Base.Threads.@spawn
using Makie
using DataStructures

include("car.jl")

time = 0

# building street
street_km = 0:100
lineA(x) = 0
lineB(x) = 10
lineC(x) = 20

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)

function calc_normal_drive(q)
	for i = 1:0.1:100
		enqueue!(q, (25.0+i, 5.0, 10.0, 0.0, 0.0, 50.0))
	end
end

function paint_car(obs_car)

	obs1 = lift(x -> [x], obs_car[1])
	obs2 = lift(x -> [x], obs_car[2])
	obs3 = lift(x -> [x], obs_car[3])
	obs4 = lift(x -> [x], obs_car[4])
	scene = arrows!(obs1, obs2, obs3, obs4, linewidth = 64, arrowsize = 7)

end

function drive_car(car::Car, obs_car)

	for i = 1:0.1:120
		position_vector = get_drive_path(car)
		obs_car[1][] = position_vector[1]
		obs_car[2][] = position_vector[2]
		obs_car[3][] = position_vector[3]
		obs_car[4][] = position_vector[4]
		sleep(0.01)
	end

end

# build car_1
q_drivepath_c1 = Queue{NTuple{6, Float64}}()
calc_normal_drive(q_drivepath_c1)
obs_car1 = [Node(25.0), Node(5.0), Node(10.0), Node(0.0)]
car_1 = Car(q_drivepath_c1)
paint_car(obs_car1)


display(scene)
#@spawn drive_car()
drive_car(car_1, obs_car1)




