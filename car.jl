module CarModule

import Base.Threads.@spawn
using Makie
using DataStructures

export Float_Tuple, State_Vec, LL_State_Vec, Car, init, get_drive_path, predict_trajectory, observe_new_car, calc_straight_drive, drive_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{7, Float64}
LL_State_Vec = MutableLinkedList{State_Vec}

ω = 0.1 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

mutable struct Car

	obs_x::Observable{Array{Float64,1}}
	obs_y::Observable{Array{Float64,1}}
	drive_path::LL_State_Vec
	no_traffic::Bool # @traffic is empty
	traffic::Array{Car, 1}
	Car(drive_path::LL_State_Vec) = new(Node(zeros(301)), Node(zeros(301)), drive_path, true)

end


function init(car::Car, scene::Scene)
	init_trajectory(car, scene)
end


function init_trajectory(car::Car, scene::Scene)

	pos_x =  lift(x -> x, car.obs_x)
	pos_y =  lift(y -> y, car.obs_y)
	lines!(scene, pos_x, pos_y, color = :orange)

end


function get_drive_path(car::Car)

	if(isempty(car.drive_path))
	   return
	end

	return popfirst!(car.drive_path)

end


function predict_trajectory(car::Car, sec_ahead::Int64)

	pred_traj = LL_State_Vec()
	push!(pred_traj, first(car.drive_path))

	for i = 0:τ:sec_ahead
		(p_x, p_y, tmp1, tmp2, θ, ω, vel) = last(pred_traj)
		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (-cos(θ + ω*τ) + cos(θ))
		θ_n= θ+ω*τ
		push!(pred_traj, (p_x_n, p_y_n, tmp1, tmp2, θ_n, ω, vel))
		#println("PREDICTION: ", p_x_n, " (p_x_n), ", p_y_n, " (p_y_n), ", θ_n, " (new_angle), ", vel, " vel ") 
	end

	return pred_traj

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

	for x = 1:50
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

	radius = 5 # attention-radius
	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)
	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= radius^2 # then it is inside of circle
		#println("Attention!!!!!!!!!!!!!!")
		pred_traj = predict_trajectory(obs_car, 3)
		#println("draw")
		@spawn draw_trajectory(obs_car, pred_traj, scene)
		return analyze_trajectory(car, pred_traj) # no need for deepcopy here
	end

end


# begin monitoring threat
function analyze_trajectory(car::Car, obs_traj::LL_State_Vec)

	println("Analyzing!!!!!!!!!!!!!!")
	(pos_x_cur, pos_y_cur) = first(car.drive_path) # our current position
	millis = 0
	radius = 3 # danger-radius
	for obs_vec in obs_traj
		millis = millis + 1
		(obs_pos_x, obs_pos_y) = obs_vec
		(pos_x_fut, pos_y_fut, _, _, _, _, vel) = getindex(car.drive_path, millis) # get our pos after @millis
		if (obs_pos_x - pos_x_fut)^2 + (obs_pos_y - pos_y_fut)^2 <= radius^2 # car will be in our danger-radius (in the future)
			@spawn try_braking(car, millis, (pos_x_cur, pos_y_cur, vel), (pos_x_fut, pos_y_fut)) #pass time to potential crash
			println("DANGER!!!!!!!!!!!!!!")
			return -1 # danger-signal
		end
	end

end


function try_braking(car::Car, millis::Int64, (pos_x, pos_y, vel), (pos_x_crash, pos_y_crash))

	braking_distance = (vel^2)/100
	pos_x_break = pos_x + braking_distance
	crash_t = millis
	safe_dist = 2
	# comment pending
	new_vel = (abs(pos_x_crash-pos_x) - safe_dist) / crash_t # @new_vel in m/0.1s
	new_vel = new_vel*10*60*60/1000 # from m/0.1s to km/h
	println(new_vel)
	new_traj = LL_State_Vec()
	calc_straight_drive((pos_x, pos_y, 3.0, 0.0, 0.00001, 0.00001, new_vel), new_traj)
	car.drive_path = new_traj
	
	if pos_x_break < pos_x_crash
		println("SUCCESS IN BREAKING")
	else
		println("NO SUCCESS IN BREAKING")
	end

end


function draw_trajectory(obs_car::Car, pred_traj::LL_State_Vec, scene::Scene)
	
	#println("repaint")
	pos_x = zeros(length(obs_car.obs_x[]))
	pos_y = zeros(length(obs_car.obs_y[]))

	for i = 1:min(length(obs_car.obs_x[]), length(pred_traj))
		if isempty(pred_traj) == true 
			break
		end
		(x, y) = getindex(pred_traj, i)
		setindex!(pos_x, x+2, i) # x + 20 to get the line to come out of front of vehicle
		setindex!(pos_y, y, i)
	end

	obs_car.obs_x[] = pos_x
	obs_car.obs_y[] = pos_y

end







function calc_straight_drive((p_x, p_y, a_x, a_y, θ, ω, v)::State_Vec, traj::LL_State_Vec)
    println("incalc")

    x_add = v*0.1 # x m/s -> we paint after 0.1s -> x*0.1 = m/0.1s

    for i = 1:0.1:5
        yaw_angle = 0.0
        if i == 2
            yaw_angle = θ # so there is a starting-angle for calculating curve
        end
        push!(traj, (p_x+x_add, p_y, a_x, a_y, yaw_angle, ω, v/10))
        x_add = x_add + v/100
    end

end


function drive_car(car::Car, obs_car::Array{Observable{Float64},1}, cause_accident::Bool)

    for i = 1:0.1:5
        if i == 2 && cause_accident == true
            pred_traj = predict_trajectory(car, 6)
            corrupt_trajectory(pred_traj)
            car.drive_path = pred_traj
        end
        position_vector = get_drive_path(car)
        #println(position_vector)
        obs_car[1][] = position_vector[1]
        obs_car[2][] = position_vector[2]
        obs_car[3][] = position_vector[3]
        obs_car[4][] = position_vector[4]
        sleep(0.1)
    end

end


function corrupt_trajectory(traj::CarModule.LL_State_Vec)

    for i = 1:length(traj)
        vec = getindex(traj, i)
        setindex!(traj, (vec[1], vec[2], vec[3], vec[4], vec[5], vec[6]+rand((0.1:0.8)), vec[7]+rand((0.5:1))), i)
    end

end











end

