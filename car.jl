using Makie
using DataStructures

#Vector = Array{Float64, 1}
Float_Tuple = Tuple{Float64, Float64}
Queue_Tuple = Queue{NTuple{6, Float64}}

mutable struct Car

	drive_path::Queue_Tuple
	Car(drive_path::Queue_Tuple) = new(drive_path)

end

function get_drive_path(car::Car)
	return dequeue!(car.drive_path)
end

