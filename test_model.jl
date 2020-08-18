using Makie

#=
O = angle
w = rate


=#

street_km = 0:100
lineA(x) = 0
lineB(x) = 5 
lineC(x) = 10

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)

t = 0.1

function predict_trajectory(deg, vel)
	#vel = vel/10
	sensors = (0.0, 0.0, deg, 0.008, vel) # (p_x, p_y, yaw_angle, yaw_rate, vel)
    for i = 0:0.1:10
		O = sensors[3]
        v = sensors[5]
		w = sensors[4]
        old_x = sensors[1]
        old_y = sensors[2]
		new_x = old_x + v/w * (sin(O + w*t) - sin(O))
		new_y = old_y + v/w * (-cos(O+w*t) + cos(O))
        new_angle = O+w*t
		sensors = (new_x, new_y, new_angle, w, sensors[5])
		scene = scatter!([new_x], [new_y], markersize = 1)
        println("PREDICTION: ", new_x, " (x), ", new_y, " (y), ", O, " (yaw_angle)")
		sleep(1)

    end 
end 
display(scene)
#predict_trajectory(0.0007, 50/10)
predict_trajectory(0.07, 80)

