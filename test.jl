using Makie

"'Time' - an Observable that controls the animation"
t = Node(0)

"The colour of the line"
c = lift(t) do t
        RGBf0(t/255, (255 - t)/255, 0)
    end

scene = lines(rand(10); linewidth=10, color = c)

record(scene, "line_changing_colour_with_observables.mp4", 1:255; framerate = 60) do i
    t[] = i # update `t`'s value
end
