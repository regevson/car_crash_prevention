import Base.Threads.@spawn
using Makie


time = 0

x = 0:100
lineA(x) = 0
lineB(x) = 10

lineC(x) = 20

scene = Scene(resolution = (1500, 500))

lines!(scene, x, lineA, color = :green, linewidth = 4)
lines!(scene, x, lineB, color = :red, linewidth = 4)
lines!(scene, x, lineC, color = :green, linewidth = 4)

obs = Node(25.0)
x_obs = lift(x -> [x], obs)
scene = arrows!(x_obs, [5], [10], [0], linewidth = 64, arrowsize = 7)
display(scene)

function drive_car()
	for i = 1:120
		global obs[] = obs[]+0.1
		sleep(0.01)
	end
end


@spawn drive_car()
drive_car()



