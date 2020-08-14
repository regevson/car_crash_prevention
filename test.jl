using Makie

#=
O = angle
w = rate


=#

street_km = 0:500
lineA(x) = 0
lineB(x) = 250 
lineC(x) = 500

scene = Scene(resolution = (1500, 500))

lines!(scene, street_km, lineA, color = :green, linewidth = 4)
lines!(scene, street_km, lineB, color = :red, linewidth = 4)
lines!(scene, street_km, lineC, color = :green, linewidth = 4)


function predict_trajectory(vel)
	sensors = (0.0, 0.0, 0.5, 0.1, vel) # (p_x, p_y, yaw_angle, yaw_rate, vel)
    for i = 0:0.1:10
		O = sensors[3]
        v = sensors[5]
		w = sensors[4]
        old_x = sensors[1]
        old_y = sensors[2]
		new_x = old_x + v/w * (sin(O + w*0.1) - sin(O))
		new_y = old_y + v/w * (cos(w) - cos(w+O*0.1))
        new_angle = O+w*0.1
		sensors = (new_x, new_y, new_angle, w, sensors[5])
		scene = scatter!([new_x], [new_y], markersize = 8)
        println("PREDICTION: ", O, " (yaw_angle), ", sensors[3], " (vel), ", w, " (yaw_rate), ", new_x, " (x), ", new_y, " (y), ", 5.0, 10.0, new_angle, v)

    end 
end 
display(scene)
predict_trajectory(50)

