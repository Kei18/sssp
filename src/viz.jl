export plot_res!

using Plots

COLORS = ["royalblue", "orange", "lime", "gray", "coral", "brown4"]

function plot_circle!(x::Float64, y::Float64, r::Float64,
                      color::String="black", lw::Float64=1.0; fillalpha::Float64=0.0)
    Xs = cos.(collect(0:0.1:pi+0.1)) * r
    Ys = sin.(collect(0:0.1:pi+0.1)) * r
    offset_x = fill(x, length(Xs))
    offset_y = fill(y, length(Ys))
    plot!(Xs + offset_x, Ys + offset_y, fillrange=-Ys + offset_y,
          color=color, label=nothing, lw=lw, fillalpha=fillalpha)
    plot!(Xs + offset_x, - Ys + offset_y, color=color, lw=lw, label=nothing)
end

function plot_obs!(obstacles::Vector{CircleObstacle2D})
    for o in obstacles; plot_circle!(o.x, o.y, o.r; fillalpha=1.0); end
end

function plot_roadmap!(V::Vector{Vector{Node{StatePoint2D}}})
    for i = 1:length(V)
        for v in V[i]
            for u_id in v.neighbors
                u = V[i][u_id]
                plot!([u.q.x, v.q.x], [u.q.y, v.q.y], color=COLORS[i],
                      lw=0.2, markersize=2, markershape=:circle, markerstrokecolor=COLORS[i], legend=nothing)
            end
        end
    end
end

function plot_start_goal!(config_init::Vector{StatePoint2D}, config_goal::Vector{StatePoint2D})
    for (i, (q_init, q_goal)) in enumerate(zip(config_init, config_goal))
        plot!([q_init.x], [q_init.y],
              seriestype=:scatter, markershape=:hex, markersize=5, label=nothing, color=COLORS[i])
        plot!([q_goal.x], [q_goal.y],
              seriestype=:scatter, markershape=:star, markersize=5, label=nothing, color=COLORS[i])
    end
end

function plot_traj!(S_fin::SuperNode{StatePoint2D},
                    VISITED::Dict{String, SuperNode{StatePoint2D}},
                    lw::Float64=3.0)
    N = length(S_fin.Q)
    x_arr = [ [v.q.x] for v in S_fin.Q ]
    y_arr = [ [v.q.y] for v in S_fin.Q ]

    S = S_fin
    while S.parent_id != nothing
        S = VISITED[S.parent_id]
        for (i, v) in enumerate(S.Q)
            if x_arr[i][1] == v.q.x && y_arr[i][1] == v.q.y; continue; end
            pushfirst!(x_arr[i], v.q.x)
            pushfirst!(y_arr[i], v.q.y)
        end
    end

    for i = 1:N
        plot!(x_arr[i], y_arr[i], color=COLORS[i], lw=lw, label=nothing)
    end
end

function plot_res!(
    config_init::Vector{StatePoint2D},
    config_goal::Vector{StatePoint2D},
    obstacles::Vector{CircleObstacle2D},
    V::Vector{Vector{Node{StatePoint2D}}},
    S_fin::Union{Nothing, SuperNode{StatePoint2D}},
    VISITED::Union{Nothing, Dict{String, SuperNode{StatePoint2D}}})

    plot()
    plot_obs!(obstacles)
    plot_roadmap!(V)
    if S_fin != nothing && VISITED != nothing; plot_traj!(S_fin, VISITED); end
    plot_start_goal!(config_init, config_goal)
    plot!(size=(400,400), xlim=(0, 1), ylim=(0, 1), framestyle=:box)
end

function plot_anim!(
    config_init::Vector{StatePoint2D},
    config_goal::Vector{StatePoint2D},
    obstacles::Vector{CircleObstacle2D},
    S_fin::SuperNode{StatePoint2D},
    VISITED::Dict{String, SuperNode{StatePoint2D}},
    rads=Vector{Float64};
    filename::String="tmp.gif",
    fps::Int64=10,)

    S = S_fin
    S_id_list = []
    while S.parent_id != nothing
        pushfirst!(S_id_list, S.id)
        S = VISITED[S.parent_id]
    end
    pushfirst!(S_id_list, S.id)

    plot_agent! = (i, v) -> begin
        plot_circle!(v.q.x, v.q.y, rads[i], COLORS[i], 3.0, fillalpha=0.1)
    end

    anim = @animate for S_id in S_id_list
        S = VISITED[S_id]
        plot(size=(400,400), xlim=(0, 1), ylim=(0, 1), framestyle=:box)
        plot_obs!(obstacles)
        plot_traj!(S_fin, VISITED, 1.0)
        plot_start_goal!(config_init, config_goal)
        for (i, v) in enumerate(S.Q); plot_agent!(i, v); end
    end
    gif(anim, filename, fps=fps)
end
