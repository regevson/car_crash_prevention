using Makie

ran = 4:10
f(x) = x^2


obs_x = Node(1:10)
obs_y = Node(1:10)

scene = lines(ran, f)
display(scene)



function init(scene)
	init_traj(scene)
end

function init_traj(scene)
	parx = lift(x -> x, obs_x)
	pary = lift(x -> x, obs_y)
	lines!(scene, parx, pary)

end


init(scene)

sleep(1)

obs_x[] = 4:13
obs_y[] = 4:13




