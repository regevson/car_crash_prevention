using Makie
using DataStructures

#Vector = Array{Float64, 1}
Float_Tuple = Tuple{Float64, Float64}
Queue_Tuple = Queue{NTuple{6, Float64}}
ω = 0.1 # yaw_rate
τ = 0.1 # time_step

# θ -> theta = angle
# ω -> omega = yaw_rate

mutable struct Car

	drive_path::Queue_Tuple
	Car(drive_path::Queue_Tuple) = new(drive_path)

end

function get_drive_path(car::Car)
	if(isempty(car.drive_path))
	   return
   end
	return dequeue!(car.drive_path)
end

function yaw_rate(deg)
	return deg*1.1
end

function predict_trajectory(car::Car, sec_ahead)

	new_q = Queue{NTuple{6, Float64}}()
	enqueue!(new_q, first(car.drive_path))

	for i = 0:τ:sec_ahead
		(p_x, p_y, tmp1, tmp2, θ, vel) = last(new_q)
		p_x_n = p_x + vel/ω * (sin(θ + ω*τ) - sin(θ))
		p_y_n = p_y + vel/ω * (cos(ω) - cos(ω+θ*τ))
		θ_n= ω*τ+θ
		enqueue!(new_q, (p_x_n, p_y_n, tmp1, tmp2, θ_n, vel))
		println("PREDICTION: ", p_x_n, " (p_x_n), ", p_y_n, " (p_y_n), ", θ_n, " (new_angle), ", vel, " vel ") 
	end

	car.drive_path = new_q	

end





