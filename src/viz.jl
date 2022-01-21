using Plots

COLORS = ["royalblue", "orange", "lime", "gray", "coral", "brown4", "aquamarine4"]

function get_color(i::Int64)::String
    return COLORS[mod1(i, length(COLORS))]
end

function plot_circle!(
    x::Float64,
    y::Float64,
    r::Float64,
    color::String = "black",
    lw::Float64 = 1.0;
    fillalpha::Float64 = 0.0,
)
    Xs = cos.(collect(0:0.1:pi+0.1)) * r
    Ys = sin.(collect(0:0.1:pi+0.1)) * r
    offset_x = fill(x, length(Xs))
    offset_y = fill(y, length(Ys))
    plot!(
        Xs + offset_x,
        Ys + offset_y,
        fillrange = -Ys + offset_y,
        color = color,
        label = nothing,
        lw = lw,
        fillalpha = fillalpha,
    )
    plot!(Xs + offset_x, -Ys + offset_y, color = color, lw = lw, label = nothing)
end

function plot_sphere!(
    x::Float64,
    y::Float64,
    z::Float64,
    r::Float64,
    color = "black",
    step::Int64 = 20,
)
    for a = -r:r/step:r
        b = sqrt(r^2 - a^2)
        Xs = cos.(collect(0:0.1:2π+0.1)) * b
        Ys = sin.(collect(0:0.1:2π+0.1)) * b
        Plots.plot!(
            Xs + fill(x, size(Xs)),
            Ys + fill(y, size(Ys)),
            fill(z + a, size(Xs)),
            legend = false,
            color = color,
        )
    end
end

function plot_obs!(obstacles::Vector{Obs}) where {Obs<:Obstacle}
    for o in obstacles
        if Obs == CircleObstacle2D
            plot_circle!(o.x, o.y, o.r; fillalpha = 1.0)
        elseif Obs == CircleObstacle3D
            plot_sphere!(o.x, o.y, o.z, o.r, "black")
        end
    end
end

function plot_roadmap!(
    V::Vector{Vector{Node{State}}},
    ins_params...,
) where {State<:AbsState}
    for i = 1:length(V)
        params = Dict(
            :lw => 0.2,
            :markersize => 2,
            :markershape => :circle,
            :legend => nothing,
            :color => get_color(i),
            :markerstrokecolor => get_color(i),
        )
        for v in V[i]
            for u_id in v.neighbors
                u = V[i][u_id]
                plot_motion!(v.q, u.q, map(arr -> arr[i], ins_params)..., params)
            end
            if isempty(v.neighbors)
                plot_motion!(v.q, v.q, map(arr -> arr[i], ins_params)..., params)
            end
        end
    end
end

function plot_start_goal!(
    config_init::Vector{State},
    config_goal::Vector{State},
    ins_params...;
) where {State<:AbsState}

    for (i, (q_init, q_goal)) in enumerate(zip(config_init, config_goal))
        params = Dict(
            :seriestype => :scatter,
            :markersize => 5,
            :label => nothing,
            :color => get_color(i),
        )
        plot_start_goal!(q_init, q_goal, map(arr -> arr[i], ins_params)..., params)
    end
end

function plot_traj!(
    solution::Union{Nothing,Vector{Vector{Node{State}}}},
    ins_params...;
    lw::Float64 = 3.0,
) where {State<:AbsState}

    isnothing(solution) && return

    N = length(solution[1])
    for (t, Q_to) in enumerate(solution[2:end])
        Q_from = solution[t]
        for i = 1:N
            params = Dict(:color => get_color(i), :lw => lw, :label => nothing)
            plot_motion!(Q_from[i].q, Q_to[i].q, map(arr -> arr[i], ins_params)..., params)
        end
    end
end

function plot_init!(State::DataType)
    if State != StatePoint3D
        plot(
            size = (400, 400),
            xlim = (0, 1),
            ylim = (0, 1),
            framestyle = :box,
            yflip = true,
            xmirror = true,
        )
    else
        plot3d(size = (400, 400), xlim = (0, 1), ylim = (0, 1), zlim = (0, 1))
    end
end

function safe_savefig!(filename::Union{Nothing,String} = nothing)
    isnothing(filename) && return
    dirname = join(split(filename, "/")[1:end-1], "/")
    !isdir(dirname) && mkpath(dirname)
    savefig(filename)
end

function plot_instance!(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...;
    filename::Union{Nothing,String} = nothing,
) where {State<:AbsState}

    N = length(config_init)
    plot_init!(State)
    plot_obs!(obstacles)
    foreach(
        i -> plot_agent!(config_init[i], map(arr -> arr[i], ins_params)..., get_color(i)),
        1:N,
    )
    plot_start_goal!(config_init, config_goal, ins_params...)
    safe_savefig!(filename)
    return plot!()
end

function plot_res!(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...;
    roadmaps::Union{Nothing,Vector{Vector{Node{State}}}} = nothing,
    solution::Union{Nothing,Vector{Vector{Node{State}}}} = nothing,
    filename::Union{Nothing,String} = nothing,
) where {State<:AbsState}

    plot_init!(State)
    plot_obs!(obstacles)
    plot_roadmap!(roadmaps, ins_params...)
    plot_traj!(solution, ins_params...)
    plot_start_goal!(config_init, config_goal, ins_params...)
    safe_savefig!(filename)
    return plot!()
end

function plot_anim!(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...;
    solution::Union{Nothing,Vector{Vector{Node{State}}}} = nothing,
    filename::String = "tmp.gif",
    fps::Int64 = 10,
    interpolate_depth::Union{Nothing,Int64} = nothing,
) where {State<:AbsState}

    if isnothing(solution)
        @warn "solution has not computed yet!"
        return
    end

    T = length(solution)
    anim = @animate for (t, Q) in enumerate(vcat(solution, [solution[end]]))
        plot_init!(State)
        plot_obs!(obstacles)
        plot_traj!(solution, ins_params...; lw = 1.0)
        plot_start_goal!(config_init, config_goal, ins_params...)

        if !isnothing(interpolate_depth) && interpolate_depth > 0 && 1 < t <= T
            C_arr = Array{Any}(undef, 2 + sum(map(k -> 2^k, 0:interpolate_depth-1)))
            add_mid_config(d, ind1, ind2) = begin
                if d > 0
                    C1 = C_arr[ind1]
                    C2 = C_arr[ind2]
                    C = map(e -> get_mid_status(e...), zip(C1, C2))
                    ind = Int64((ind1 + ind2) / 2)
                    C_arr[ind] = C
                    add_mid_config(d - 1, ind1, ind)
                    add_mid_config(d - 1, ind, ind2)
                end
            end

            C_arr[1] = map(v -> v.q, solution[t-1])
            C_arr[end] = map(v -> v.q, solution[t])
            add_mid_config(interpolate_depth, 1, length(C_arr))
            for Q_tmp in C_arr
                for (i, q) in enumerate(Q_tmp)
                    plot_agent!(q, map(arr -> arr[i], ins_params)..., get_color(i))
                end
            end
        else
            for (i, v) in enumerate(Q)
                plot_agent!(v.q, map(arr -> arr[i], ins_params)..., get_color(i))
            end
        end
    end

    dirname = join(split(filename, "/")[1:end-1], "/")
    !isdir(dirname) && mkpath(dirname)

    return gif(anim, filename, fps = fps)
end

function plot_tpg!(
    TPG;
    init::Bool = true,
    offset::Int64 = 0,
    filename::Union{Nothing,String} = nothing,
)
    N = length(TPG)
    if init
        plot(size = (200, 400), xlim = (0.5, N + 0.5), xticks = 1:N)
    end
    params =
        Dict(:label => nothing, :color => :black, :markersize => 5, :markershape => :circle)
    for (i, actions) in enumerate(TPG)
        for a in actions
            for (j, b_id) in a.successors
                k = findfirst(a -> a.id == b_id, TPG[j])
                if k != nothing
                    b = TPG[j][k]
                    plot!([i, j], [a.t + offset, b.t + offset]; params...)
                end
            end
            annotate!(i - 0.1, a.t + offset, text(a.id, :black, :right, 3))
        end
        if length(actions) == 1
            scatter!([i], [actions[1].t + offset]; params...)
        end
    end
    safe_savefig!(filename)
    return plot!()
end

function plot_tpg!(TPG1, TPG2; filename::Union{Nothing,String} = nothing)
    # compute offset
    maxval_tpg1 = (
        !all(map(e -> isempty(e), TPG1)) ?
        maximum(map(e -> e[end].t, filter(e -> !isempty(e), TPG1))) : 1
    )
    minval_tpg2 = (
        !all(map(e -> isempty(e), TPG2)) ?
        minimum(map(e -> e[1].t, filter(e -> !isempty(e), TPG2))) : 0
    )
    offset = maxval_tpg1 - minval_tpg2 + 1
    plot_tpg!(TPG1)
    plot_tpg!(TPG2; init = false, offset = offset)
    for (i, actions) in enumerate(TPG1)
        if !isempty(actions)
            if !isempty(TPG2[i])
                plot!(
                    [i, i],
                    [TPG1[i][end].t, TPG2[i][1].t + offset];
                    label = nothing,
                    color = :black,
                )
            end
            scatter!(
                [i],
                [TPG1[i][end].t];
                color = :red,
                label = nothing,
                markersize = 5,
                markershape = :circle,
            )
        end
    end

    safe_savefig!(filename)
    return plot!()
end
