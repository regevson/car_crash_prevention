
Vector = Array{Float64, 1}

mutable struct Car

	pos_x::Vector
	pos_y::Vector
	yaw_angle::Vector
	velocity::Vector

	Car(pos_x::Vector, pos_y::Vector, yaw_angle::Vector, velocity::Vector) = new(pos_x, pos_y, yaw_angle, velocity)

end
