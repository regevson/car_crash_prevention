module CarModule

import Base.Threads.@spawn
using DataStructures

export Float_Tuple, State_Vec, Queue_State_Vec, Car, get_drive_path, predict_trajectory, observe_new_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{6, Float64}
Queue_State_Vec = Queue{State_Vec}

ω = 0.1 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

mutable struct Car

	drive_path::Queue_State_Vec
	no_traffic::Bool # @traffic is empty
	traffic::Array{Car, 1}
	Car(drive_path::Queue_State_Vec) = new(drive_path, true)

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
		(p_x, p_y, tmp1, tmp2, θ, vel) = last(q_n)
		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (cos(ω) - cos(ω+θ*τ))
		θ_n= ω*τ+θ
		enqueue!(q_n, (p_x_n, p_y_n, tmp1, tmp2, θ_n, vel))
		println("PREDICTION: ", p_x_n, " (p_x_n), ", p_y_n, " (p_y_n), ", θ_n, " (new_angle), ", vel, " vel ") 
	end

	return q_n

end

function observe_new_car(car::Car, obs_car::Car)
	if car.no_traffic == true
		car.traffic = [obs_car]
		car.no_traffic = false
	else
		append!(car.traffic, obs_car)
	end

	@spawn check_traffic(car)

end

function check_traffic(car::Car)

	for x = 1:30
		for i = 1:length(car.traffic)
			signal = check_car(car, car.traffic[i])
			if signal == -1
				return
			end
		end
		sleep(0.5)
	end
	
	#check_traffic(car) -> should eventually become recursive
	
end

function check_car(car::Car, obs_car::Car)

	radius = 150 # attention-radius
	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)
	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= radius^2 # then it is inside of circle
		predicted_traj = predict_trajectory(obs_car, 5)
		return analyze_trajectory((my_pos_x, my_pos_y), deepcopy(predicted_traj))
	end

end


function analyze_trajectory((pos_x, pos_y)::Tuple{Float64, Float64}, q::Queue_State_Vec)

	radius = 100 # danger-radius
	while isempty(q) == false
		(obs_pos_x, obs_pos_y) = dequeue!(q)
		if (obs_pos_x - pos_x)^2 + (obs_pos_y - pos_y)^2 <= radius^2 # then it is inside of circle
			println("DANGER!!!!!!!!!!!!!!")
			return -1 # danger-signal
		end
	end

end

end

