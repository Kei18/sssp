using Dubins

const DUBINS_TURN_RADIUS = 0.05

struct StateDubins <: AbsState
    x::Float64
    y::Float64
    θ::Float64
end

to_string(s::StateDubins) = @sprintf("(x: %.4f, y: %.4f, θ: %.4f)", s.x, s.y, s.θ)

function get_mid_status(p::StateDubins, q::StateDubins)::StateDubins
    errcode, path = dubins_shortest_path([p.x, p.y, p.θ], [q.x, q.y, q.θ], DUBINS_TURN_RADIUS)
    errcode != 0 && return StateDubins(
        (p.x + q.x) / 2,
        (p.y + q.y) / 2,
        diff_angles(q.θ, p.θ) / 2 + p.θ,
    )
    StateDubins(dubins_path_sample(path, dubins_path_length(path) / 2)[end]...)
end

function dist(q1::StateDubins, q2::StateDubins)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y, diff_angles(q1.θ, q2.θ) / π])
end

function gen_connect(
    q::StateDubins,  # to identify type
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D};
    step_dist::Float64 = 0.01,
    max_dist::Union{Nothing,Float64} = nothing,
)::Function

    # check: q \in C_free
    f(q::StateDubins, i::Int64)::Bool = begin
        # within field
        any(x -> (x < rads[i] || 1 - rads[i] < x), [q.x, q.y]) && return false

        # obstacles
        any([dist([q.x, q.y], [o.x, o.y]) < o.r + rads[i] for o in obstacles]) && return false

        return true
    end

    f(q_from::StateDubins, q_to::StateDubins, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        errcode, path = dubins_shortest_path(
            [q_from.x, q_from.y, q_from.θ],
            [q_to.x, q_to.y, q_to.θ],
            DUBINS_TURN_RADIUS,
        )
        errcode != 0 && return false
        points = dubins_path_sample_many(path, step_dist)[end]
        if !isnothing(points)
            for (x, y, θ) in points
                # outside
                if (x < rads[i] || 1 - rads[i] < x || y < rads[i] || 1 - rads[i] < y)
                    return false
                end

                # obstacles
                any(o -> dist([x, y], [o.x, o.y]) < o.r + rads[i], obstacles) && return false
            end
        end

        return true
    end

    return f
end

function gen_collide(
    q::StateDubins,
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST_LINE2D,
    safety_dist::Float64 = 0.01,
    )::Function

    N = length(rads)

    f(
        q_i_from::StateDubins,
        q_i_to::StateDubins,
        q_j_from::StateDubins,
        q_j_to::StateDubins,
        i::Int64,
        j::Int64,
    ) = begin
        return false
    end

    f(q_i::StateDubins, q_j::StateDubins, i::Int64, j::Int64) = begin
        return false
    end

    f(Q_from::Vector{Node{StateDubins}}, Q_to::Vector{Node{StateDubins}}) = begin
        for i = 1:N, j = i+1:N
            if f(Q_from[i].q, Q_to[i].q, Q_from[j].q, Q_to[j].q, i, j)
                return true
            end
        end
        return false
    end

    f(Q::Vector{Node{StateDubins}}, q_i_to::StateDubins, i::Int64) = begin
        return false
    end

    return f
end

function gen_uniform_sampling(q::StateDubins)::Function
    () -> begin
        return StateDubins(rand(2)..., rand() * 2π)
    end
end

function plot_motion!(
    q_from::StateDubins,
    q_to::StateDubins,
    rad::Float64,
    params;
    step_dist::Float64=0.01,
    )

    q_from == q_to && return

    path = dubins_shortest_path(
        [q_from.x, q_from.y, q_from.θ],
        [q_to.x, q_to.y, q_to.θ],
        DUBINS_TURN_RADIUS,
    )[2]
    points = dubins_path_sample_many(path, step_dist)[end]
    p = copy(params)
    if haskey(p, :markershape)
        !isnothing(points) && scatter!([points[end][1]], [points[end][2]]; params...)
        delete!(p, :markershape)
    end
    !isnothing(points) && plot!(map(e -> e[1], points), map(e -> e[2], points); p...)
end

function plot_start_goal!(q_init::StateDubins, q_goal::StateDubins, rad::Float64, params)
    plot!([q_init.x], [q_init.y]; markershape = :hex, params...)
    plot!([q_goal.x], [q_goal.y]; markershape = :star, params...)
end

function plot_agent!(q::StateDubins, rad::Float64, color::String)
    plot_circle!(q.x, q.y, rad, color, 3.0, fillalpha = 0.1)
    p_from_x = rad * cos(q.θ) + q.x
    p_from_y = rad * sin(q.θ) + q.y
    plot!([q.x, p_from_x], [q.y, p_from_y], lw = 5, color = color, legend = nothing)
end

function gen_random_instance_StateDubins(; params...)
    return gen_random_instance(StateDubins(0, 0, 0); params...)
end

function plot_anim!(
    config_init::Vector{StateDubins},
    config_goal::Vector{StateDubins},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...;
    solution::Union{Nothing,Vector{Vector{Node{StateDubins}}}} = nothing,
    filename::String = "tmp.gif",
    fps::Int64 = 10,
    interpolate_depth::Union{Nothing,Int64} = nothing
)

    if isnothing(solution)
        @warn "solution has not computed yet!"
        return
    end

    T = length(solution)
    N = length(config_init)
    anim = @animate for (t, Q) in enumerate(vcat(solution, [solution[end]]))
        plot_init!(StateDubins)
        plot_obs!(obstacles)
        plot_traj!(solution, ins_params...; lw = 1.0)
        plot_start_goal!(config_init, config_goal, ins_params...)


        if !isnothing(interpolate_depth) && interpolate_depth > 0 && 1 < t <= T
            depth = sum(map(k -> 2^k, 0:interpolate_depth-1)) + 2
            for i in 1:N
                q_from = solution[t-1][i].q
                q_to = solution[t][i].q
                path = dubins_shortest_path(
                    [q_from.x, q_from.y, q_from.θ],
                    [q_to.x, q_to.y, q_to.θ],
                    DUBINS_TURN_RADIUS
                )[end]
                points = dubins_path_sample_many(path, dubins_path_length(path) / depth)[end]
                for (x, y, θ) in vcat(points, [[q_to.x, q_to.y, q_to.θ]])
                    plot_agent!(StateDubins(x, y, θ), map(arr -> arr[i], ins_params)..., get_color(i))
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
