module CarModule

import Base.Threads.@spawn
using Makie
using DataStructures

export Float_Tuple, State_Vec, LL_State_Vec, Car, init, get_drive_path, predict_trajectory, observe_new_car, calc_straight_drive, drive_car

Float_Tuple = Tuple{Float64, Float64}
State_Vec = NTuple{7, Float64}
LL_State_Vec = MutableLinkedList{State_Vec}

ω = 0.4 # yaw_rate
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


function predict_trajectory(car::Car, sec_ahead::Float64)

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
			if signal == -1
				return
			end
		end
		sleep(0.1)
	end
	
	#check_traffic(car, scene) #-> should eventually become recursive
	
end


function check_car(car::Car, obs_car::Car, scene::Scene)

	radius = 4.1 # attention-radius
	(my_pos_x, my_pos_y) = first(car.drive_path)
	(obs_pos_x, obs_pos_y) = first(obs_car.drive_path)

	if (obs_pos_x - my_pos_x)^2 + (obs_pos_y - my_pos_y)^2 <= radius^2 # then it is inside of circle
		println("Attention!!!!!!!!!!!!!!")
		pred_traj = predict_trajectory(obs_car, 1.5) # react to events that are max. 1.5 sec. in the future
		@spawn draw_trajectory(obs_car, pred_traj, scene)
		return analyze_trajectory(car, pred_traj) # no need for deepcopy here
	end

end


# begin monitoring threat
function analyze_trajectory(car::Car, obs_traj::LL_State_Vec)

	println("Analyzing!!!!!!!!!!!!!!")
	(pos_x_cur, pos_y_cur) = first(car.drive_path) # our current position
	millis = 0 # time passed (in ms)
	radius = 2 # danger-radius

	for obs_vec in obs_traj
		millis += 1
		(obs_pos_x, obs_pos_y) = obs_vec
		(pos_x_fut, pos_y_fut, _, _, _, _, vel) = getindex(car.drive_path, millis) # get our pos after @millis

		if (obs_pos_x - pos_x_fut)^2 + (obs_pos_y - pos_y_fut)^2 <= radius^2 # car will be in our danger-radius (in the future)
			println("DANGER!!!!!!!!!!!!!!")
			@spawn try_braking(car, millis, (pos_x_cur, pos_y_cur, vel*10), (pos_x_fut, pos_y_fut)) # try to avoid crash by braking
			return -1 # danger-signal
		end

	end

end


#new_vel1(vel, ct, cd) = (648*ct+35*vel-9*sqrt(-5600*cd+5184*(ct^2)+560*ct*vel-1008*ct+49)-64) / 35
#new_vel2(vel, ct, cd) = 18.51428571429*ct+1*vel-0.2571428571429*sqrt(-5600*cd+5184*(ct^2)+560*ct*vel-1008*ct+49)-1.8
#new_vel(vel, ms_to_crash, pos_x, safe_pos_x) = (-sqrt(5600*ms_to_crash + 5600*pos_x - 5600*safe_pos_x + 400*(vel^2) - 8280*vel + 42849) + 207) / 20
new_vel(vel, ms_to_crash, pos_x, safe_pos_x) = (-14*ms_to_crash + 20*vel + sqrt(28*(ms_to_crash^2) - 80*ms_to_crash*vel + 28*ms_to_crash- 800*pos_x + 800*safe_pos_x + 7)*sqrt(7) - 7)/20


function try_braking(car::Car, ms_to_crash::Int64, (pos_x, pos_y, vel), (pos_x_crash, pos_y_crash))

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

	rest_dist = (safe_pos_x - pos_x) - lin_break_dist
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
		rest_dist = (safe_pos_x - pos_x) - (vel/10*lin_break_t + -0.07*((lin_break_t*(lin_break_t+1))/2))

	rest_dist_t = rest_dist*(10/new_vel):
		rest_dist = new_vel/10 * rest_dist_t | : new_vel/10
	-> rest_dist_t = rest_dist*(10/new_vel)

	=> new_vel = (-sqrt(5600*ms_to_crash + 5600*pos_x - 5600*safe_pos_x + 400*(vel^2) - 8280*vel + 42849) + 207) / 20
	
	=#

	safe_dist = 0.0138
	println(ms_to_crash, ", ms_to_crash")
	println(pos_x_crash-pos_x, ", crash_distance")

	safe_pos_x = pos_x_crash-safe_dist

	nv = new_vel(mps(vel), ms_to_crash, pos_x, safe_pos_x) 
	println("hello")
	nv = kph(nv)
	println(nv, ", new-vel")

	new_traj = linearly_brake(car, vel, nv, safe_pos_x, pos_x)
	car.drive_path = new_traj

		
	lin_break_t = mps(nv - vel) / -0.7	# in ms
	println(lin_break_t, ", lin_break_t")
	if lin_break_t < ms_to_crash
		println("SUCCESS IN KEEPING SAFE-DISTANCE")
	else
		println("NO SUCCESS IN KEEPING SAFE-DISTANCE")
	end

end

function linearly_brake(car::Car, vel::Float64, end_vel::Float64, safe_pos_x, pos_x)

	lin_break_t = mps(end_vel - vel) / -0.7	# in ms
	println(lin_break_t, ", lin_break_t")
	rest_dist_t = (safe_pos_x - pos_x) - (vel/10*lin_break_t + -0.07*((lin_break_t*(lin_break_t+1))/2))
	println(rest_dist_t, ", rest_dist_t")
	vec = first(car.drive_path) # current position
	positions = [vec[1]]
	for i = 1:lin_break_t
		next_driven_meters = (mps(vel) - 0.7*i) / 10
		new_pos = last(positions) + next_driven_meters
		append!(positions, new_pos)
	end

	new_traj = LL_State_Vec()
	for i = 1:length(positions)
		push!(new_traj, (positions[i], vec[2], vec[3], vec[4], 0.0, 0.0001, end_vel))
	end
	println("hello")

	# continue with straigt drive
	calc_straight_drive(last(new_traj), new_traj)

	return new_traj

end


mps(kph::Float64) = kph/60/60*1000
mpms(kph::Float64) = kph/60/60/10*1000
kph(mps::Float64) = mps*60*60/1000


function draw_trajectory(obs_car::Car, pred_traj::LL_State_Vec, scene::Scene)
	
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







function calc_straight_drive((p_x, p_y, a_x, a_y, θ, ω, v)::State_Vec, traj::LL_State_Vec)
    #println("incalc")

	i_pos_x = p_x

    for i = 1:0.1:5
        yaw_angle = 0.0
        if i >= 1.3
            yaw_angle = θ # so there is a starting-angle for calculating curve
        end
		push!(traj, (i_pos_x, p_y, a_x, a_y, yaw_angle, ω, v/10))
		i_pos_x += mpms(v)
    end

end


function drive_car(car::Car, obs_car::Array{Observable{Float64},1}, cause_accident::Bool)

    for i = 1:0.1:5
        if i == 2 && cause_accident == true
            pred_traj = predict_trajectory(car, 6.0)
            #corrupt_trajectory(pred_traj)
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

