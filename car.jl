module CarModule

import Base.Threads.@spawn
using Makie
using DataStructures

export Float_Tuple, State_Vec, Queue_State_Vec, Car, init, get_drive_path, predict_trajectory, observe_new_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{7, Float64}
Queue_State_Vec = Queue{State_Vec}

ω = 0.1 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

mutable struct Car

	obs_x::Observable{Array{Float64,1}}
	obs_y::Observable{Array{Float64,1}}
	drive_path::Queue_State_Vec
	no_traffic::Bool # @traffic is empty
	traffic::Array{Car, 1}
	Car(drive_path::Queue_State_Vec) = new(Node(zeros(301)), Node(zeros(301)), drive_path, true)

end

function init(car::Car, scene::Scene)
	init_trajectory(car, scene)
end


function get_drive_path(car::Car)

	if(isempty(car.drive_path))
	   return
	end

	return dequeue!(car.drive_path)

end

function predict_trajectory(car::Car, sec_ahead::Int64)

	q_n = Queue{State_Vec}()
	enqueue!(q_n, first(car.drive_path))

	for i = 0:τ:sec_ahead
		(p_x, p_y, tmp1, tmp2, θ, ω, vel) = last(q_n)
		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (cos(ω) - cos(ω+θ*τ))
		θ_n= ω*τ+θ
		enqueue!(q_n, (p_x_n, p_y_n, tmp1, tmp2, θ_n, ω, vel))
		#println("PREDICTION: ", p_x_n, " (p_x_n), ", p_y_n, " (p_y_n), ", θ_n, " (new_angle), ", vel, " vel ") 
	end

	return q_n

end

function observe_new_car(car::Car, obs_car::Car, scene::Scene)

	if car.no_traffic == true
		car.traffic = [obs_car]
		car.no_traffic = false
	else
		append!(car.traffic, obs_car)
	end

	@spawn check_traffic(car, scene)

end

function check_traffic(car::Car, scene::Scene)

	for x = 1:30
		for i = 1:length(car.traffic)
			signal = check_car(car, car.traffic[i], scene::Scene)
			#=if signal == -1
				return
			end=#
		end
		sleep(0.1)
	end
	
	#check_traffic(car, scene) #-> should eventually become recursive
	
end

function check_car(car::Car, obs_car::Car, scene::Scene)

	println("PREDICT!!!!!!!!!!!!!!")
	radius = 200 # attention-radius
	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)
	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= radius^2 # then it is inside of circle
		predicted_traj = predict_trajectory(obs_car, 5)
		@spawn draw_trajectory(obs_car, deepcopy(predicted_traj), scene)
		return analyze_trajectory(car, deepcopy(predicted_traj)) # no need for deepcopy here
	end

end


# begin monitoring threat
function analyze_trajectory(car::Car, obs_q::Queue_State_Vec)

	q = deepcopy(car.drive_path)
	(pos_x_now, pos_y_now) = first(q)
	millis = 0
	radius = 100 # danger-radius
	while isempty(obs_q) == false && isempty(q) == false
		millis = millis + 1
		(obs_pos_x, obs_pos_y) = dequeue!(obs_q)
		(pos_x, pos_y, _, _, _, _, vel) = dequeue!(q)
		if (obs_pos_x - pos_x)^2 + (obs_pos_y - pos_y)^2 <= radius^2 # then it is inside of circle
			try_braking(millis, (pos_x_now, pos_y_now, vel), (pos_x, pos_y)) #pass time to potential crash
			println("DANGER!!!!!!!!!!!!!!")
			return -1 # danger-signal
		end
	end

end

function try_braking(millis::Int64, (pos_x, pos_y, vel), (pos_x_crash, pos_y_crash))
	braking_distance = (vel^2)/100
	pos_x_break = pos_x + braking_distance
	if pos_x_break < pos_x_crash
		println("SUCCESS IN BREAKING")
	else
		println("NO SUCCESS IN BREAKING")
	end
end


function init_trajectory(car::Car, scene::Scene)
	pos_x =  lift(x -> x, car.obs_x)
	pos_y =  lift(y -> y, car.obs_y)
	lines!(scene, pos_x, pos_y, color = :orange)
end

function draw_trajectory(obs_car::Car, traj::Queue_State_Vec, scene::Scene)
	
	pos_x = zeros(length(obs_car.obs_x[]))
	pos_y = zeros(length(obs_car.obs_y[]))

	for i = 1:length(obs_car.obs_x[])
		if isempty(traj) == true 
			break
		end
		(x, y) = dequeue!(traj)
		setindex!(pos_x, x+20, i) # x + 20 to get the line to come out of front of vehicle
		setindex!(pos_y, y, i)
	end

	println("\nDRAWING")
	obs_car.obs_x[] = pos_x
	obs_car.obs_y[] = pos_y

end
















end

