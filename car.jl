module CarModule

import Base.Threads.@spawn
using Makie
using DataStructures
using AbstractPlotting.MakieLayout

export Float_Tuple, State_Vec, LL_State_Vec, Car, init, observe_new_car, calc_drive, drive_car, paint_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{7, Float64}
LL_State_Vec = MutableLinkedList{State_Vec}

ω = 0.4 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

max_braking_rate = 0.7
max_acceleration_rate = 0.3


mutable struct Car

	# observables (visualization)
	obs_x::Observable{Array{Float64,1}}
	obs_y::Observable{Array{Float64,1}}
	safety_pos::Observable{Tuple{Float64,Float64}}
	crash_pos::Observable{Tuple{Float64,Float64}}
	car_color::Observable{String}

	no_traffic::Bool # when @traffic is empty
	drive_path::LL_State_Vec
	traffic::Array{Car, 1}
	Car(car_color::Observable{String}) = new(Node(zeros(301)), Node(zeros(301)), Node((0.0,5.0)), Node((0.0,5.0)), car_color, true)

end


function init(car::Car, scene::LAxis)
	init_trajectory(car, scene)
end

# speedometer observables
vel_ego_car = Node(0)
vel_threat_car = Node(0)
braking_force_speedometer = Node(0.0)

# set up observables and trajectory-line
function init_trajectory(car::Car, scene::LAxis)

	# init trajectory
	pos_x = lift(x -> x, car.obs_x)
	pos_y = lift(y -> y, car.obs_y)
	lines!(scene, pos_x, pos_y, color = :orange, linewidth = 3, linestyle = :solid)

	# init safety/crash - location
	safety_pos = lift(x -> x, car.safety_pos)
	crash_pos = lift(x -> x, car.crash_pos)
	scatter!(scene, safety_pos, color = :green)	
	scatter!(scene, crash_pos, color = :red)	

	# car-color

	# speedometer
	vel = lift(vel -> "$vel km/h (speed)   |", vel_ego_car)
	text!(scene, vel, textsize = 0.6, position = (0.0,10.0))
	vel = lift(vel -> "$vel km/h (speed)", vel_threat_car)
	text!(scene, vel, textsize = 0.6, position = (0.0,-1.0), color = :gray53)
	bf = lift(force -> "$force m/s/s (braking_force)", braking_force_speedometer)
	text!(scene, bf, textsize = 0.6, position = (6.0,10.0))

end


# return most recent state-vector
function get_state_vec(car::Car)

	if isempty(car.drive_path)
		return
	elseif length(car.drive_path) == 1
		return last(car.drive_path)
	end

	return popfirst!(car.drive_path)

end


# predict trajectory @sec_ahead sec. in the future
# with CTRV-Model
function predict_trajectory(car::Car, starting_vec::State_Vec, sec_ahead::Float64, braking_rate::Float64)

	pred_traj = LL_State_Vec(starting_vec)

	for i = 0:τ:sec_ahead
		(p_x, p_y, tmp1, tmp2, θ, ω, vel) = last(pred_traj)
		vel = milesph(vel)/2.18
		if vel > 0 && braking_rate != 0.0
			vel = vel_after_braking(i, vel, braking_rate)
		elseif vel <= 0
			vel = 0.0
    end

		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (-cos(θ + ω*τ) + cos(θ))
		θ_n= θ+ω*τ
    
		push!(pred_traj, (p_x_n, p_y_n, tmp1, tmp2, θ_n, ω, pred_to_kph(vel)))
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

	for x = 1:70
		for i = 1:length(car.traffic)
			signal = check_car(car, car.traffic[i], scene::LAxis)
			sleep(0.1)

			if signal == -1
				return
			end

		end
	end
	println("\n\n\n 70 reached \n\n\n")
	
	#check_traffic(car, scene) #-> should eventually become recursive
	
end


# check if @obs_car reaches @attention-radius
function check_car(car::Car, obs_car::Car, scene::LAxis)

	attention_radius = 8 # meters

	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)

	# check if @obs_car is inside @car's attention-radius
	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= attention_radius^2
		println("attention-radius invaded!")
		pred_traj = predict_trajectory(obs_car, first(obs_car.drive_path), 1.5, 0.0) # react to events that are max. 0.9 sec. in the future
		@spawn draw_trajectory(obs_car, pred_traj, scene)
		return analyze_trajectory(car, obs_car, pred_traj, scene)
	end

end


prev_vel = -1.0 # velocity we are currently accelerating to
# begin monitoring threat
function analyze_trajectory(car::Car, obs_car::Car, obs_traj::LL_State_Vec, scene::LAxis)

	println("analyzing threat...")
	(cur_pos_x, cur_pos_y, _, _, _, _, cur_vel) = first(car.drive_path) # @car's cur position
	cur_vec_obs = first(obs_car.drive_path) # @obs_car's cur position
	millis = 0 # time passed (in ms)
	danger_radius = 3.0 # meter

	# check if @obs_car (belongs @obs_traj) will be in @car's danger_radius in the fut
	for obs_vec in obs_traj
		millis += 1
		(obs_pos_x, obs_pos_y, _, _, _, _, obs_vel) = obs_vec
		(fut_pos_x, fut_pos_y, _, _, _, _, fut_vel) = getindex(car.drive_path, millis) # get our pos after @millis (should be predicted eventually)

		if (obs_pos_x - fut_pos_x)^2 + (obs_pos_y - fut_pos_y)^2 <= danger_radius^2 # car will be in our danger-radius (in the fut)
			println("detected car in danger-radius!")
			obs_car.car_color[] = "red2"
			straight_braking_feedback = 0
			straight_braking_feedback = try_straight_braking(car, millis, (cur_pos_x, cur_pos_y, cur_vel), (obs_pos_x, obs_pos_y, obs_vel), scene)

			if straight_braking_feedback == -1
				#try_braking_to_side(car, millis, (cur_pos_x, cur_pos_y, vel), (obs_pos_x, obs_pos_y), scene)
				println("BRAKING TO SIDE")
				return -1
			end

			return 0 

		end
		
	end

	# if @obs_car is out of @danger_radius, paint it gray
	obs_car.car_color[] = "gray53"

	global prev_vel
	# as @obs_car is out of @danger_radius, adapt speed to the one of @obs_car (car in front) (if not already adapted)
	if cur_vec_obs[7] > cur_vel && cur_vec_obs[7] != prev_vel 			
		println("accelerate")
		linearly_brake_accelerate(car, cur_vel, cur_vec_obs[7], cur_pos_x, max_acceleration_rate, vel_after_accelerating, "green", scene)
		prev_vel = cur_vec_obs[7]
		return 0
	end

end


# compute velocity (in mps) after @t_ms of braking down from starting-velocity @vel:
vel_after_braking(t_ms::Float64, vel::Float64, braking_rate::Float64) = vel - braking_rate*t_ms
vel_after_accelerating(t_ms::Float64, vel::Float64, accel_rate::Float64) = vel + accel_rate*t_ms

function try_braking_to_side(car::Car, ms_to_crash::Int64, (pos_x, pos_y, vel)::NTuple{3, Float64}, (pos_x_crash, pos_y_crash)::NTuple{2, Float64}, scene::LAxis)

	#= Explanation =#

	println("initiating emergency braking...")

	# trajectory with max steering angle and max braking
	optimal_traj_ll = predict_trajectory(car, (pos_x, pos_y, 1.0, 0.0, 0.5, 0.0001, vel), 0.5, 0.7)
	last_opt_traj_vec = last(optimal_traj_ll)
	calc_drive(car, 1.0, last_opt_traj_vec, (0.0, 0.0), optimal_traj_ll, false, false, -1.0)

	# empty @drive_path as new one is created (emergency-braking-path)
	delete!(car.drive_path, 2:length(car.drive_path)) # first state-vec has yet to be painted (otherwise gap opens)

	for i = 1:length(optimal_traj_ll)

		# * 4 at the end so deviations on y-axis are more costly and get compensated more
		vel_pred_format = milesph(vel)/2.18
		println(vel, " vel")
		diff(arg, opt_x, opt_y, weight_x, weight_y) = sqrt((pos_x + (vel_pred_format+arg[2])/ω * (sin(arg[1] + ω*τ) - sin(arg[1])) - opt_x)^2)*weight_x + 
							sqrt((pos_y + (vel_pred_format+arg[2])/ω * (-cos(arg[1] + ω*τ) + cos(arg[1])) - opt_y)^2)*weight_y

		println("finding min deviaton...")
		sol_coords = find_min_deviation_configuration(diff, (-0.3,0.3), (-0.3, 0.5), optimal_traj_ll)
		
		new_vel = vel_pred_format + sol_coords[2]
		if new_vel < 0
			new_vel = 0.0
		end
		new_angle = sol_coords[1]
		new_start_vec = (pos_x, pos_y, 1.0, 0.0, new_angle, ω, pred_to_kph(new_vel))
		println("start_vec: ", new_start_vec)
		# no braking as it is already applied by the computed reduced velocity
		new_vec = (pos_x, pos_y, tmp1, tmp2, angle, rate, vel) = last(predict_trajectory(car, new_start_vec, 0.0, 0.0))
		println("new_vec: ", new_vec)
		println("----------------------------------------")
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

function new_vel_braking(vel, ms_to_crash, pos_x, safe_pos_x, braking_force) 

	discriminant = 4*braking_force^2*ms_to_crash^2+4*braking_force^2*ms_to_crash+braking_force^2-8*braking_force*ms_to_crash*vel-80*braking_force*pos_x+80*braking_force*safe_pos_x
	if discriminant < 0
		return -1.0 # no solution -> safety-distance cannot be kept (car goes too fast)
	end

	return (-2*braking_force*ms_to_crash-braking_force+2*vel - sqrt(discriminant))/2

end

# lin_braking_function, computes new velocity to brake down to
function new_vel_braking(vel, ms_to_crash, pos_x, safe_pos_x, braking_force) 

	discriminant = 4*braking_force^2*ms_to_crash^2+4*braking_force^2*ms_to_crash+braking_force^2-8*braking_force*ms_to_crash*vel-80*braking_force*pos_x+80*braking_force*safe_pos_x
	if discriminant < 0
		return -1.0 # no solution -> safety-distance cannot be kept (car goes too fast)
	end

	return (-2*braking_force*ms_to_crash-braking_force+2*vel - sqrt(discriminant))/2

end

function try_straight_braking(car::Car, ms_to_crash::Int64, (pos_x, pos_y, vel)::NTuple{3, Float64}, (pos_x_crash, pos_y_crash, obs_vel)::NTuple{3, Float64}, scene::LAxis)

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

	try

		println("----------------------------------------------------------------------")
		println("try braking...")

		# mark crash-location
		car.crash_pos[] = (pos_x_crash, 6.0)

		# compute new velocity and braking_force to avoid crash by a distance of @safe_dist
		braking_force = 0.0
		nv = -1.0
		safe_dist = 0.0
		# sd = 1.0:0.1:4.0 and let for run until end with nv safed globally -> so best bf and sd come out at the end
		for bf = 0.7:-0.1:0.1 # braking-force
			for sd = 4.0:-0.1:1.5 # safety-distance
				sp = pos_x_crash-sd # safety-position
				nv = new_vel_braking(mps(vel), ms_to_crash, pos_x, sp, bf) # compute velocity to brake down to (in mps)
				if nv >= 0
					braking_force = bf
					braking_force_speedometer[] = bf*10
					safe_dist = sd
					car.safety_pos[] = (sp, 6.0) # mark safety-location
					@goto outer
				end
			end
		end
		@label outer

		if nv < 0 
			car.safety_pos[] = (pos_x_crash-2.5, 6.0) # mark safety-location

			if nv == -1.0 # safety-distance cannot be kept by straight linear braking
				println("NO SUCCESS IN KEEPING SAFETY-DISTANCE!")
				println("Driving with crash dist: : ", (pos_x_crash-pos_x), " with vel: ", vel, " and crash_t: ", ms_to_crash, " gives newvel: ", kph(nv))
				println("----------------------------------------------------------------------")
				return -1

			# @ms_to_crash is too big -> nv is negative bc. small speed is needed to cover small distance with a lot of time (@ms_to_crash)
			# so just try braking as hard as possible
			elseif vel > obs_vel
				linearly_brake_accelerate(car, vel, obs_vel, pos_x, -0.7, vel_after_braking, "blue", scene)
				braking_force_speedometer[] = 0.7*10
				println("\nDriving with crash dist: : ", (pos_x_crash-pos_x), " with vel: ", vel, " and crash_t: ", ms_to_crash, " gives newvel: ", 0.0, "\n")
			end

			println("Going too slow")
			println("----------------------------------------------------------------------")
			return 0
		end
		
		if kph(nv) > vel
			# adapt velocity to velocity of threat-car
			nv = mps(obs_vel) 
			println("Driving with crash dist: : ", (pos_x_crash-pos_x), " with vel: ", vel, 
							" and crash_t: ", ms_to_crash, " gives newvel: ", kph(nv), " with braking_force: ", braking_force, " and safe_dist: ", safe_dist)
		end


		nv = kph(nv)
		#nv = 5.0

		# compute lin_brake_trajectory
		linearly_brake_accelerate(car, vel, nv, pos_x, -braking_force, vel_after_braking, "blue", scene)
			
		println("SUCCESS IN KEEPING SAFETY-DISTANCE!")
		println("Driving with crash dist: : ", (pos_x_crash-pos_x), " with vel: ", vel, 
						" and crash_t: ", ms_to_crash, " gives newvel: ", nv, " with braking_force: ", braking_force, " and safe_dist: ", safe_dist)
		println("----------------------------------------------------------------------")
		return 0

	catch e
    bt = backtrace()
    msg = sprint(showerror, e, bt)
    println(msg)
  end

end


function linearly_brake_accelerate(car::Car, start_vel::Float64, end_vel::Float64, pos_x_start::Float64, force::Float64, 
																	 vel_after_braking_accel::Function, col::String, scene::LAxis)

	try

		lin_break_t = mps(end_vel-start_vel) / force # in ms
		println("linbreakt: ", lin_break_t, " with start_vel: ", start_vel, " and end_vel: ", end_vel)
		if lin_break_t <= 0
			return
		end

		# compute new positions reached while linearly braking down
		vec = first(car.drive_path) # current position
		delete!(car.drive_path, 2:length(car.drive_path)) # first pos_vec has to be drawn still
		pos_x = pos_x_start
		for i = 1:lin_break_t
			new_vel = vel_after_braking_accel(i, mps(start_vel), sign(force)*force) # mps # sign to make force positive in any case
			next_driven_meters = new_vel / 10
			pos_x = pos_x + next_driven_meters
			push!(car.drive_path, (pos_x, vec[2], vec[3], vec[4], 0.0, 0.0001, kph(new_vel)))
		end
		if lin_break_t < 1
			pos_x = pos_x + sign(force)*mps(end_vel)/10
			push!(car.drive_path, (pos_x, vec[2], vec[3], vec[4], 0.0, 0.0001, end_vel))
		end

		
		# continue with straigt drive (append to @new_traj)
		last_new_traj = last(car.drive_path)
		delete!(car.drive_path, length(car.drive_path))
		calc_drive(car, 7.0, last_new_traj, (0.0, 0.0), car.drive_path, false, false, -1.0)

		# display braking-distance
		linesegments!(scene, [pos_x_start, pos_x], [6, 6], linestyle = :dot, linewidth = 3, color = col)
		scatter!(scene, [pos_x_start], [6], color = col, markersize = 6)
		#scatter!(scene, [pos_x], [6], color = :blue, markersize = 6)

	catch e
    bt = backtrace()
    msg = sprint(showerror, e, bt)
    println(msg)
  end
	
end


# velocity-conversions
mps(kph::Float64) = kph/60/60*1000
mpms(kph::Float64) = kph/60/60/10*1000
kph(mps::Float64) = mps*60*60/1000
milesph(kph::Float64) = kph/1.609
kph_from_miles_ph(milesph::Float64) = milesph*1.609
pred_to_kph(pred::Float64) = kph_from_miles_ph(pred)*2.18


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
		appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, v), t_till_crash, 0.0))
	
		if cut_left == true
			#appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, v), 2.0, 0.0))
			(p_x, p_y) = last(traj)
			cut_left_traj = predict_trajectory(car, (p_x, p_y, a_x, a_y, θ, ω, v), t_after_crash, 0.0)
			appendLL(traj, cut_left_traj)
		elseif emergency_brake == true # stop and go
			#=
			v = mps(v) 
			pos_x = 0.0
			for i = 0:0.1:2.0
				push!(traj, (pos_x, 6.0, 1.0, 0.0, 0.0, 0.0001, kph(v)))
				next_driven_meters = v/10
				pos_x = pos_x + next_driven_meters
			end
			=#
			(p_x, p_y) = last(traj)
			appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, v), t_after_crash, 0.05))
			(p_x, p_y) = last(traj)
			appendLL(traj, predict_trajectory(car, (p_x, p_y, a_x, a_y, 0.001, 0.001, 20.0), 8.5, 0.0))
		end
	catch e
    bt = backtrace()
    msg = sprint(showerror, e, bt)
    println(msg)
  end
end


function drive_car(car::Car, obs_car::Array{Observable{Float64},1}, crash, scene::LAxis)

	try
		i = 1.0
		old_s_main_car = 0.0
		old_s_threat_car = 0.0
		while isempty(car.drive_path) == false && i <= 8 # duration of drive
			i += 0.1
			position_vector = get_state_vec(car)
			if crash == false 
				new_s = kph((position_vector[1] + position_vector[2])*10)
				vel_ego_car[] = ceil(Int, position_vector[7])
				old_s_main_car = new_s
				new_x = position_vector[1]
				#println("removed x-axis: ", new_x) 
			else
				new_s = kph((position_vector[1] + position_vector[2])*10)
				vel_threat_car[] = ceil(Int, abs(old_s_threat_car-new_s))
				old_s_threat_car = new_s
			end
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


function paint_car(obs_car::Array{Observable{Float64},1}, obs_car_col::Observable{String}, scene::LAxis)

  obs1 = lift(x -> [x], obs_car[1])
  obs2 = lift(x -> [x], obs_car[2])
  obs3 = lift(x -> [x], obs_car[3])
  obs4 = lift(x -> [x], obs_car[4])
	obs_col = lift(x -> x, obs_car_col)
  arrows!(scene, obs1, obs2, obs3, obs4, linewidth = 30, arrowsize = 0, linecolor = obs_col)

end


function corrupt_trajectory(traj::LL_State_Vec)

    for i = 1:length(traj)
        vec = getindex(traj, i)
        setindex!(traj, (vec[1], vec[2], vec[3], vec[4], vec[5], vec[6]+rand((0.1:0.8)), vec[7]+rand((0.5:1))), i)
    end

end




end

