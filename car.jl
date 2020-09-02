module CarModule

import Base.Threads.@spawn
using Makie
using DataStructures
using AbstractPlotting.MakieLayout

export Float_Tuple, State_Vec, LL_State_Vec, Car, init, observe_new_car, calc_drive, drive_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{7, Float64}
LL_State_Vec = MutableLinkedList{State_Vec}

ω = 0.4 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

max_braking_rate = 0.3

mutable struct Car

	obs_x::Observable{Array{Float64,1}}
	obs_y::Observable{Array{Float64,1}}
	no_traffic::Bool # when @traffic is empty
	drive_path::LL_State_Vec
	traffic::Array{Car, 1}
	Car() = new(Node(zeros(301)), Node(zeros(301)), true)

end


function init(car::Car, scene::LAxis)
	init_trajectory(car, scene)
end


# set up observables and trajectory-line
function init_trajectory(car::Car, scene::LAxis)

	pos_x =  lift(x -> x, car.obs_x)
	pos_y =  lift(y -> y, car.obs_y)
	lines!(scene, pos_x, pos_y, color = :orange, linewidth = 3, linestyle = :solid)

end


# return most recent state-vector
function get_state_vec(car::Car)

	if(isempty(car.drive_path))
	   return
	end

	return popfirst!(car.drive_path)

end


# predict trajectory @sec_ahead sec. in the future
# with CTRV-Model
function predict_trajectory(car::Car, starting_vec::State_Vec, sec_ahead::Float64, braking_rate::Float64)

	pred_traj = LL_State_Vec(starting_vec)
	#push!(pred_traj, first(car.drive_path))

	for i = 0:τ:sec_ahead
		(p_x, p_y, tmp1, tmp2, θ, ω, vel) = last(pred_traj)
		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (-cos(θ + ω*τ) + cos(θ))
		θ_n= θ+ω*τ

    if vel >= 0 && braking_rate != 0.0
			vel = vel_after_braking(i, vel, braking_rate)
		elseif vel <= 0
			return pred_traj
    end

		push!(pred_traj, (p_x_n, p_y_n, tmp1, tmp2, θ_n, ω, vel))
		#println("PREDICTION: ", p_x_n, " (p_x_n), ", p_y_n, " (p_y_n), ", θ_n, " (new_angle), ", vel, " vel ") 
	end

	return pred_traj

end


# observe new car
function observe_new_car(car::Car, obs_car::Car, scene::LAxis)

	if car.no_traffic == true
		car.traffic = [obs_car]
		car.no_traffic = false
	else
		append!(car.traffic, obs_car)
	end

	@spawn check_traffic(car, scene)

end


# iterate through traffic
function check_traffic(car::Car, scene::LAxis)

	for x = 1:50
		for i = 1:length(car.traffic)
			signal = check_car(car, car.traffic[i], scene::LAxis)
			if signal == -1
				return
			end
		end
		sleep(0.1)
	end
	
	#check_traffic(car, scene) #-> should eventually become recursive
	
end


# check if @obs_car reaches @attention-radius
function check_car(car::Car, obs_car::Car, scene::LAxis)

	attention_radius = 6 # meters

	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)

	# check if @obs_car is inside @car's attention-radius
	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= attention_radius^2
		println("attention-radius invaded!")
		pred_traj = predict_trajectory(obs_car, first(obs_car.drive_path), 0.9, 0.0) # react to events that are max. 0.9 sec. in the future
		@spawn draw_trajectory(obs_car, pred_traj, scene)
		return analyze_trajectory(car, pred_traj, scene)
	end

end


# begin monitoring threat
function analyze_trajectory(car::Car, obs_traj::LL_State_Vec, scene::LAxis)

	println("analyzing threat...")
	(pos_x_cur, pos_y_cur) = first(car.drive_path) # @car's current position
	millis = 0 # time passed (in ms)
	danger_radius = 3 # meters

	# check if @obs_car (belongs @obs_traj) will be in @car's danger_radius in the future
	for obs_vec in obs_traj
		millis += 1
		(obs_pos_x, obs_pos_y) = obs_vec
		(pos_x_fut, pos_y_fut, _, _, _, _, vel) = getindex(car.drive_path, millis) # get our pos after @millis (should be predicted eventually)

		if (obs_pos_x - pos_x_fut)^2 + (obs_pos_y - pos_y_fut)^2 <= danger_radius^2 # car will be in our danger-radius (in the future)
			println("detected car in danger-radius!")
			straight_braking_feedback = try_straight_braking(car, millis, (pos_x_cur, pos_y_cur, kph_from_miles_ph(vel)*5), (obs_pos_x, obs_pos_y), scene)

			if straight_braking_feedback == -1
				try_braking_to_side(car, millis, (pos_x_cur, pos_y_cur, kph_from_miles_ph(vel)*5, vel), (obs_pos_x, obs_pos_y), scene)
			end

			# eventually add alternative-functions for when @try_straight_braking doesn't work
			return -1 # danger-signal
		end

	end

end


# compute velocity (in mps) after @t_ms of braking down from starting-velocity @vel:
vel_after_braking(t_ms::Float64, vel::Float64, braking_rate::Float64) = vel - braking_rate*t_ms

function try_braking_to_side(car::Car, ms_to_crash::Int64, (pos_x, pos_y, vel, vel_pred_format)::NTuple{4, Float64}, (pos_x_crash, pos_y_crash)::NTuple{2, Float64}, scene::LAxis)

	#= Explanation =#

	println("initiating emergency braking...")

	# trajectory with max steering angle and max braking
	optimal_traj_ll = predict_trajectory(car, (pos_x, pos_y, 1.0, 0.0, 0.5, 0.0001, vel_pred_format), 1.0, 0.1)
	last_opt_traj_vec = last(optimal_traj_ll)
	calc_drive(car, 7.0, last_opt_traj_vec, (0.0, 0.0), optimal_traj_ll, false, false, -1.0)

	# empty @drive_path as new one is created (emergency-braking-path)
	delete!(car.drive_path, 1:length(car.drive_path))

	for i = 1:length(optimal_traj_ll)

		# * 4 at the end so deviations on y-axis are more costly and get compensated more
		diff(arg, opt_x, opt_y, weight_x, weight_y) = sqrt((pos_x + (vel_pred_format+arg[2])/ω * (sin(arg[1] + ω*τ) - sin(arg[1])) - opt_x)^2)*weight_x + 
							sqrt((pos_y + (vel_pred_format+arg[2])/ω * (-cos(arg[1] + ω*τ) + cos(arg[1])) - opt_y)^2)*weight_y

		println("finding min deviaton...")
		sol_coords = find_min_deviation_configuration(diff, (-0.3,0.3), (-max_braking_rate, max_braking_rate), optimal_traj_ll)
		
		new_vel = vel_pred_format + sol_coords[2]
		if new_vel < 0
			new_vel = 0.0
		end
		new_angle = sol_coords[1]
		new_start_vec = (pos_x, pos_y, 1.0, 0.0, new_angle, ω, new_vel)
		# no braking as it is already applied by the computed reduced velocity
		new_vec = (pos_x, pos_y, tmp1, tmp2, angle, rate, vel_pred_format) = last(predict_trajectory(car, new_start_vec, 0.1, 0.0))
		# add newly calculated position to @drive_path
		push!(car.drive_path, new_vec)
		
		#println((pos_x, pos_y), " actual pos", " and angle: ", new_angle, " and new vel-addition: ", sol_coords[2], " with vel: ", new_vel, " at iteration: ", i)
		scatter!(scene, [pos_x], [pos_y], color = :purple, markersize = 5)
		scatter!(scene, [optimal_traj_ll[i][1]], [optimal_traj_ll[i][2]], color = :yellow, markersize = 5)
		#println((optimal_traj_ll[i][1], optimal_traj_ll[i][2]), " optimal pos")

	end

end

function find_min_deviation_configuration(f::Function, (x_start, x_end)::NTuple{2, Float64}, (y_start, y_end)::NTuple{2, Float64}, opt_traj::LL_State_Vec)

	min_diff = Inf
	min_x = 0.0
	min_y = 0.0
	min_o_index = 0.0

	for x = x_start:0.1:x_end
		for y = y_start:0.1:y_end
			for o = 1:length(opt_traj)-2

					weight_x = 1/abs(opt_traj[o][1]-opt_traj[o+1][1])
					weight_y = 1/abs(opt_traj[o][2]-opt_traj[o+1][2])
					# add straight line detection
					diff = f([x,y], opt_traj[o][1], opt_traj[o][2], weight_x, weight_y)

					if(diff < min_diff)
							min_diff = diff
							min_x = x
							min_y = y
							min_o_index = o
					end

			end
		end
	end

	if abs(opt_traj[min_o_index+1][2]-opt_traj[min_o_index+2][2]) < 0.02
		min_x = 0.001
		min_y = y_start
	end


	return (min_x, min_y)
		
end


# lin_braking_function, computes new velocity to brake down to
function new_vel(vel, ms_to_crash, pos_x, safe_pos_x) 

	discriminant = 28*(ms_to_crash^2) - 80*ms_to_crash*vel + 28*ms_to_crash- 800*pos_x + 800*safe_pos_x + 7
	if discriminant < 0
		return -1 # no solution -> safety-distance cannot be kept (car goes too fast)
	end

	return (-14*ms_to_crash + 20*vel + sqrt(discriminant) * sqrt(7) - 7)/20

end

function try_straight_braking(car::Car, ms_to_crash::Int64, (pos_x, pos_y, vel)::NTuple{3, Float64}, (pos_x_crash, pos_y_crash)::NTuple{2, Float64}, scene::LAxis)

	#= 

	in @millis ms @obs_car will be at position @pos_x_crash and in @millis ms @car will be there too -> crash
	-> new velocity @new_vel has to be found with which @car will be at a safe distance @safe_dist from @pos_x_crash
	
	goal is to compute new velocity @new_vel that has the following properties:
	pos_x_______________________pos_new_vel__________________________safe_pos_x|__->safe_dist__|pos_x_crash______________________________
                -> lin_break_t                    -> rest_dist_t

	 - time_to_break_down_to_new_vel_linearly (lin_break_t) + time_to_drive_rest_distance_with_new_vel (rest_dist_t) = ms_to_crash
	 - = lin_break_t + rest_dist_t = ms_to_crash
	 -> solve equation to @new_vel
	 -> with @new_vel @car is at @safe_pos_x when @obs_car is at @pos_x_crash -> crash avoided

	=#
	  	
	#=	

	assuming braking-rate: 7 m/s/s -> 0.7 m/s/0.1s
	assuming all velocities are in mps

	compute velocity (in mps) after @t_ms of braking down from starting-velocity @vel:
	-> velocity_after_t_ms_of_braking(t_ms::Float64, vel::Float64) = vel - 0.7*t_ms

	lin_break_t = (new_vel - vel) / -0.7:
		velocity_after_t_ms_of_braking->(new_vel) = vel->(vel) - 0.7 * t_ms | - vel
		new_vel - vel = -0.7 * t_ms | : -0.7
		(new_vel - vel) / -0.7 = t_ms
	-> after @t_ms of braking, @car reaches velocity @new_vel -> @t_ms = lin_break_t

	rest_dist = (safe_pos_x - pos_x) - lin_break_dist:
		lin_break_dist ... distance covered while linearly braking
		lin_break_dist = vel/10 * lin_break_t + -0.07 * ((lin_break_t * (lin_break_t+1)) / 2):
			vel_after_t_ms (in mps) = vel - 0.7*t
			vel_after_t_ms (in mps) / 10 = x m/ms -> in the next ms, x meters are driven
			-> e.g. after 1 ms of braking a velocity of v m/ms is reached -> so in the next ms @car will drive v meters
			lin_break_dist = (vel-0.7*1)/10 + (vel-0.7*2)/10 + (vel-0.7*3)/10 + ... + (vel-0.7*lin_break_t)/10
						   = vel/10 + (-0.7*1)/10 + vel/10 + (-0.7*2)/10 + ... + vel/10 + (-0.7*lin_break_t)/10
						   = vel/10*lin_break_t + (-0.7*1)/10 + (-0.7*2)/10 + ... + (-0.7*lin_break_t)/10
						   = vel/10*lin_break_t + -0.07*(1 + 2 + ... + lin_break_t)
						   = vel/10*lin_break_t + -0.07*((lin_break_t*(lin_break_t+1))/2)
		->lin_break_dist = vel/10*lin_break_t + -0.07*((lin_break_t*(lin_break_t+1))/2)
	-> rest_dist = (safe_pos_x - pos_x) - (vel/10*lin_break_t + -0.07*((lin_break_t*(lin_break_t+1))/2))

	rest_dist_t = rest_dist*(10/new_vel):
		rest_dist = new_vel/10 * rest_dist_t | : new_vel/10
	-> rest_dist_t = rest_dist*(10/new_vel)
	
	=#

	println("try braking...")

	safe_dist = 1.0 # safety-distance away from crash-location
	safe_pos_x = pos_x_crash-safe_dist # safety-position away from crash-location

	# mark safety-location and crash-location
	scatter!(scene, [safe_pos_x], [6], color = :green)	
	scatter!(scene, [pos_x_crash], [6], color = :red)	

	# compute new velocity to brake down to
	nv = new_vel(mps(vel), ms_to_crash, pos_x, safe_pos_x) # in mps

	# safety-distance cannot be kept by straight linear braking
	if nv == -1
		println("NO SUCCESS IN KEEPING SAFETY-DISTANCE!")
		return -1
	end

	nv = kph(nv)
	println(nv, ", new-vel")

	# compute lin_brake_trajectory
	new_traj = linearly_brake(car, vel, nv, pos_x, scene)
	car.drive_path = new_traj

		
	println("SUCCESS IN KEEPING SAFETY-DISTANCE!")

end

function linearly_brake(car::Car, vel::Float64, end_vel::Float64, pos_x, scene)

	lin_break_t = mps(end_vel - vel) / -max_braking_rate	# in ms
	# compute new positions reached while linearly braking down
	positions = [pos_x]
	for i = 1:lin_break_t
		next_driven_meters = vel_after_braking(i, mps(vel), max_braking_rate) / 10
		new_pos = last(positions) + next_driven_meters
		append!(positions, new_pos)
	end

	# display braking-distance
	linesegments!(scene, [positions[1], positions[end]], [6, 6], linestyle = :dot, linewidth = 3, color = :blue)
	scatter!(scene, [positions[1]], [6], color = :blue, markersize = 6)
	scatter!(scene, [positions[length(positions)]], [6], color = :blue, markersize = 6)

	# set-up trajectory with computed @positions
	vec = first(car.drive_path) # current position
	new_traj = LL_State_Vec()
	for i = 1:length(positions)
		push!(new_traj, (positions[i], vec[2], vec[3], vec[4], 0.0, 0.0001, end_vel))
	end

	# continue with straigt drive (append to @new_traj)
	last_new_traj = last(new_traj)
	calc_drive(car, 7.0, last_new_traj, (0.0, 0.0), new_traj, false, false, -1.0)

	return new_traj

end


# velocity-conversions
mps(kph::Float64) = kph/60/60*1000
mpms(kph::Float64) = kph/60/60/10*1000
kph(mps::Float64) = mps*60*60/1000
milesph(kph::Float64) = kph/1.609
kph_from_miles_ph(milesph::Float64) = milesph*1.609


function draw_trajectory(obs_car::Car, pred_traj::LL_State_Vec, scene::LAxis)
	
	#println("repaint")
	pos_x = zeros(length(obs_car.obs_x[]))
	pos_y = zeros(length(obs_car.obs_y[]))

	for i = 1:length(pred_traj)
		(x, y) = getindex(pred_traj, i)
		setindex!(pos_x, x+1, i) # x + 1 to get the line to come out of front of vehicle
		setindex!(pos_y, y, i)
	end

	obs_car.obs_x[] = pos_x
	obs_car.obs_y[] = pos_y

end

function appendLL(ll1::LL_State_Vec, ll2::LL_State_Vec)
	for elem in ll2
		push!(ll1, elem)
	end
end


function calc_drive(car::Car, t_till_crash::Float64, (p_x, p_y, a_x, a_y, _, _, v)::State_Vec, (θ, ω)::NTuple{2, Float64}, 
										traj::LL_State_Vec, cut_left::Bool, emergency_brake::Bool, t_after_crash::Float64) 

	try
		appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, milesph(v)/5), t_till_crash, 0.0))
	
		if cut_left == true
			(p_x, p_y) = last(traj)
			cut_left_traj = predict_trajectory(car, (p_x, p_y, a_x, a_y, θ, ω, milesph(v)/5), t_after_crash, 0.0)
			appendLL(traj, cut_left_traj)
		elseif emergency_brake == true
			(p_x, p_y) = last(traj)
			appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, milesph(v)/5), t_after_crash, 0.1))
		end
	catch e
    bt = backtrace()
    msg = sprint(showerror, e, bt)
    println(msg)
  end
end


function drive_car(car::Car, obs_car::Array{Observable{Float64},1})

	try
		i = 1.0
		while isempty(car.drive_path) == false && i <= 9 # duration of drive
			i += 0.1
			position_vector = get_state_vec(car)
			#println(position_vector, " pos_vec")
			obs_car[1][] = position_vector[1]
			obs_car[2][] = position_vector[2]
			obs_car[3][] = position_vector[3]
			obs_car[4][] = position_vector[4]
			sleep(0.1)
		end
	catch e
    bt = backtrace()
    msg = sprint(showerror, e, bt)
    println(msg)
  end


end


function corrupt_trajectory(traj::LL_State_Vec)

    for i = 1:length(traj)
        vec = getindex(traj, i)
        setindex!(traj, (vec[1], vec[2], vec[3], vec[4], vec[5], vec[6]+rand((0.1:0.8)), vec[7]+rand((0.5:1))), i)
    end

end



end

