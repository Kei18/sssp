export plot_res!, plot_anim!

using Plots
using Plots: savefig

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

function plot_sphere!(x::Float64, y::Float64, z::Float64, r::Float64, color="black", step::Int64=20)
    for a = -r:r/step:r
        b = sqrt(r^2-a^2)
        Xs = cos.(collect(0:0.1:2π+0.1))* b
        Ys = sin.(collect(0:0.1:2π+0.1))* b
        Plots.plot!(Xs + fill(x, size(Xs)),
                    Ys + fill(y, size(Ys)),
                    fill(z + a, size(Xs)),
                    legend=false, color=color)
    end
end

function plot_obs!(obstacles::Vector{Obs}) where Obs<:Obstacle
    for o in obstacles
        if Obs == CircleObstacle2D
            plot_circle!(o.x, o.y, o.r; fillalpha=1.0)
        elseif Obs == CircleObstacle3D
            plot_sphere!(o.x, o.y, o.z, o.r, "black")
        end
    end
end

function plot_roadmap!(V::Vector{Vector{Node{State}}};
                       rads::Union{Vector{Float64}, Nothing}=nothing) where State<:AbsState
    for i = 1:length(V)
        params = Dict(:color => COLORS[i], :lw => 0.2, :markersize => 2,
                      :markerstrokecolor => COLORS[i], :legend => nothing)
        for v in V[i]
            for u_id in v.neighbors
                u = V[i][u_id]
                if State in [StatePoint2D, StateLine2D]
                    plot!([u.q.x, v.q.x], [u.q.y, v.q.y]; markershape=:circle, params...)
                elseif State == StatePoint3D
                    plot!([u.q.x, v.q.x], [u.q.y, v.q.y], [u.q.z, v.q.z];
                          markershape=:circle, params...)
                elseif State == StateArm2
                    v_pos= get_arm_tip_point(v.q, rads[i])
                    u_pos = get_arm_tip_point(u.q, rads[i])
                    plot!([v_pos[1], u_pos[1]], [v_pos[2], u_pos[2]];
                          markershape=:circle, params...)
                end
            end
        end
    end
end

function plot_start_goal!(
    config_init::Vector{State}, config_goal::Vector{State};
    rads::Union{Nothing, Vector{Float64}}=nothing) where State<:AbsState
    for (i, (q_init, q_goal)) in enumerate(zip(config_init, config_goal))
        params = Dict(:seriestype => :scatter, :markersize => 5, :label => nothing, :color => COLORS[i])
        if State in [StatePoint2D, StateLine2D]
            plot!([q_init.x], [q_init.y]; markershape=:hex, params...)
            plot!([q_goal.x], [q_goal.y]; markershape=:star, params...)
        elseif State == StatePoint3D
            plot!([q_init.x], [q_init.y], [q_init.z]; markershape=:hex, params...)
            plot!([q_goal.x], [q_goal.y], [q_goal.z]; markershape=:star, params...)
        elseif State == StateArm2
            pos_init = get_arm_tip_point(q_init, rads[i])
            pos_goal = get_arm_tip_point(q_goal, rads[i])
            plot!([pos_init[1]], [pos_init[2]]; markershape=:hex, params...)
            plot!([pos_goal[1]], [pos_goal[2]]; markershape=:star, params...)
        end
    end
end

function plot_traj!(
    S_fin::SuperNode{State},
    VISITED::Dict{String, SuperNode{State}},
    lw::Float64=3.0;
    rads::Union{Nothing, Vector{Float64}}=nothing
    ) where State<:AbsState

    N = length(S_fin.Q)
    x_arr = [ [v.q.x] for v in S_fin.Q ]
    y_arr = [ [v.q.y] for v in S_fin.Q ]
    z_arr = (State == StatePoint3D) ? [ [v.q.z] for v in S_fin.Q ] : []

    if State == StateArm2
        for (i, v) in enumerate(S_fin.Q)
            pos = get_arm_tip_point(v.q, rads[i])
            x_arr[i][1] = pos[1]
            y_arr[i][1] = pos[2]
        end
    end

    S = S_fin
    while S.parent_id != nothing
        S = VISITED[S.parent_id]
        for (i, v) in enumerate(S.Q)
            if State in [StatePoint2D, StatePoint3D, StateLine2D]
                if x_arr[i][1] == v.q.x && y_arr[i][1] == v.q.y; continue; end
                pushfirst!(x_arr[i], v.q.x)
                pushfirst!(y_arr[i], v.q.y)
                if State == StatePoint3D
                    pushfirst!(z_arr[i], v.q.z)
                end
            elseif State == StateArm2
                pos = get_arm_tip_point(v.q, rads[i])
                pushfirst!(x_arr[i], pos[1])
                pushfirst!(y_arr[i], pos[2])
            end
        end
    end

    for i = 1:N
        if State == StatePoint3D
            plot!(x_arr[i], y_arr[i], z_arr[i], color=COLORS[i], lw=lw, label=nothing)
        else
            plot!(x_arr[i], y_arr[i], color=COLORS[i], lw=lw, label=nothing)
        end
    end
end

function plot_init!(State::DataType)
    if State != StatePoint3D
        plot(size=(400,400), xlim=(0, 1), ylim=(0, 1), framestyle=:box)
    else
        plot3d(xlim=(0, 1), ylim=(0, 1), zlim=(0, 1))
    end
end

function plot_res!(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where Obs<:Obstacle,
    V::Vector{Vector{Node{State}}},
    S_fin::Union{Nothing, SuperNode{State}},
    VISITED::Union{Nothing, Dict{String, SuperNode{State}}};
    rads::Union{Vector{Float64}, Nothing}=nothing,
    filename::Union{Nothing, String}=nothing
    ) where State<:AbsState

    plot_init!(State)
    plot_obs!(obstacles)
    plot_roadmap!(V, rads=rads)
    if S_fin != nothing && VISITED != nothing; plot_traj!(S_fin, VISITED; rads=rads); end
    plot_start_goal!(config_init, config_goal; rads=rads)
    if filename != nothing; savefig(filename); end
    return plot!()
end

function plot_anim!(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where Obs<:Obstacle,
    S_fin::SuperNode{State},
    VISITED::Dict{String, SuperNode{State}},
    rads=Vector{Float64};
    filename::String="tmp.gif",
    fps::Int64=10
    ) where State<:AbsState

    S = S_fin
    S_id_list = []
    while S.parent_id != nothing
        pushfirst!(S_id_list, S.id)
        S = VISITED[S.parent_id]
    end
    pushfirst!(S_id_list, S.id)

    plot_agent! = (i, v) -> begin
        if State == StatePoint2D
            plot_circle!(v.q.x, v.q.y, rads[i], COLORS[i], 3.0, fillalpha=0.1)
        elseif State == StatePoint3D
            plot_sphere!(v.q.x, v.q.y, v.q.z, rads[i], COLORS[i])
        elseif State == StateLine2D
            plot!([v.q.x, rads[i] * cos(v.q.theta) + v.q.x],
                  [v.q.y, rads[i] * sin(v.q.theta) + v.q.y],
                  color=COLORS[i], lw=5, label=nothing)
        elseif State == StateArm2
            pos1 = get_arm_intermediate_point(v.q, rads[i])
            pos2 = get_arm_tip_point(v.q, rads[i])
            plot!([v.q.x, pos1[1], pos2[1]], [v.q.y, pos1[2], pos2[2]], color=COLORS[i],
                  lw=5, markershape=:circle, markersize=3, label=nothing)
            scatter!([v.q.x], [v.q.y], markershpa=10, markershape=:hex, color=COLORS[i], label=nothing)
        end
    end

    anim = @animate for S_id in S_id_list
        plot_init!(State)
        S = VISITED[S_id]
        plot_obs!(obstacles)
        plot_traj!(S_fin, VISITED, 1.0; rads=rads)
        plot_start_goal!(config_init, config_goal; rads=rads)
        for (i, v) in enumerate(S.Q); plot_agent!(i, v); end
    end
    gif(anim, filename, fps=fps)
end
