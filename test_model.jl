using Makie

#=
O = angle
w = rate


=#

street_km = 0:30
lineA(x) = 0
lineB(x) = 5 
lineC(x) = 10

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)

t = 0.1



# velocity-conversions
mps(kph::Float64) = kph/60/60*1000
mpms(kph::Float64) = kph/60/60/10*1000
kph(mps::Float64) = mps*60*60/1000
milesph(kph::Float64) = kph/1.609
kph_from_miles_ph(milesph::Float64) = milesph*1.609

vel_func(t_ms::Float64, vel::Float64) = vel - 0.7*t_ms

function predict_trajectory(deg, vel)
	vel = milesph(vel)/5
	sensors = (0.0, 0.0, deg, 0.00001, vel) # (p_x, p_y, yaw_angle, yaw_rate, vel)
	for i = 0:0.1:3
		O = sensors[3]
		v = sensors[5]
		w = sensors[4]
		old_x = sensors[1]
		old_y = sensors[2]
		new_x = old_x + v/w * (sin(O + w*t) - sin(O))
		new_y = old_y + v/w * (-cos(O+w*t) + cos(O))
		new_angle = O+w*t
		vel = vel_func(i, vel)
		if vel < 0
			return
		end
		sensors = (new_x, new_y, new_angle, w, vel)
		scene = scatter!([new_x], [new_y], markersize = 1)
		println("PREDICTION: ", new_x, " (x), ", new_y, " (y), ", O, " (yaw_angle), ", vel, ", (vel)")
		sleep(0.1)
	end 

end 
display(scene)
#predict_trajectory(0.0007, 50/10)
#
predict_trajectory(0.5, 80.0)

