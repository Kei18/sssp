using Dubins

struct StateDubins <: AbsState
    x::Float64
    y::Float64
    θ::Float64
end

function get_mid_status(p::StateDubins, q::StateDubins)::StateDubins
    return StateDubins((p.x + q.x) / 2, (p.y + q.y) / 2, diff_angles(q.θ, p.θ) / 2 + p.θ)
end

function dist(q1::StateDubins, q2::StateDubins)::Float64
    return norm([q1.x - q2.x, q1.y - q2.y, diff_angles(q1.θ, q2.θ) / π])
end

function get_dubins_points(
    q_i_from::State,
    q_i_to::State,
    rad::Float64;
    step_dist::Float64 = STEP_DIST,
    n_dividing::Union{Nothing,Int64} = nothing,
)::Union{Nothing,Vector{Vector{Float64}}} where {State<:AbsState}
    # same -> return one point
    q_i_from == q_i_to && return [[q_i_from.x, q_i_from.y, q_i_from.θ]]

    # obtain Dubins shortest path
    (errcode, path_i) = dubins_shortest_path(
        [q_i_from.x, q_i_from.y, q_i_from.θ],
        [q_i_to.x, q_i_to.y, q_i_to.θ],
        DUBINS_TURN_RADIUS * rad,
    )
    errcode != 0 && return nothing

    # sampling
    samples = dubins_path_sample_many(
        path_i,
        isnothing(n_dividing) ? step_dist : dubins_path_length(path_i) / n_dividing,
    )[end]

    if !isnothing(samples)
        # add goal location
        samples = vcat(samples, [[q_i_to.x, q_i_to.y, q_i_to.θ]])
    else
        # when failing to get samples -> add points of q_i_from and q_i_to
        samples = [[q_i_from.x, q_i_from.y, q_i_from.θ], [q_i_to.x, q_i_to.y, q_i_to.θ]]
    end

    return samples
end


function gen_connect(
    q::StateDubins,  # to identify type
    obstacles::Vector{CircleObstacle2D},
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
    max_dist::Union{Nothing,Float64} = nothing,
)::Function

    # check: q \in C_free
    f(q::StateDubins, i::Int64)::Bool = begin
        # within field
        any(x -> (x < rads[i] || 1 - rads[i] < x), [q.x, q.y]) && return false

        # obstacles
        any([dist([q.x, q.y], o) < o.r + rads[i] for o in obstacles]) && return false

        return true
    end

    f(q_from::StateDubins, q_to::StateDubins, i::Int64)::Bool = begin
        D = dist(q_from, q_to)
        !isnothing(max_dist) && D > max_dist && return false

        P = get_dubins_points(q_from, q_to, rads[i]; step_dist = STEP_DIST)
        isnothing(P) && return false
        for p in P
            # outside
            any(e -> e < rads[i] || 1 - rads[i] < e, p[1:2]) && return false

            # obstacles
            any(o -> dist(p[1:2], o) < o.r + rads[i], obstacles) && return false
        end

        return true
    end

    return f
end

function gen_collide(
    q::StateDubins,
    rads::Vector{Float64};
    step_dist::Float64 = STEP_DIST,
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
        P_i = get_dubins_points(q_i_from, q_i_to, rads[i]; step_dist = step_dist)
        P_j = get_dubins_points(q_j_from, q_j_to, rads[j]; step_dist = step_dist)
        for p_i in P_i, p_j in P_j
            dist(p_i[1:2], p_j[1:2]) < rads[i] + rads[j] && return true
        end

        return false
    end

    f(q_i::StateDubins, q_j::StateDubins, i::Int64, j::Int64) = begin
        return dist([q_i.x, q_i.y], [q_j.x, q_j.y]) < rads[i] + rads[j]
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
        P_i = get_dubins_points(Q[i].q, q_i_to, rads[i]; step_dist = step_dist)
        for j = 1:N
            j == i && continue
            p_j = [Q[j].q.x, Q[j].q.y]
            for p_i in P_i
                dist(p_i[1:2], p_j) < rads[i] + rads[j] && return true
            end
        end
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
    step_dist::Float64 = STEP_DIST,
)
    q_from == q_to && return
    points = get_dubins_points(q_from, q_to, rad; step_dist = step_dist)
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

function gen_random_instance_StateDubins(;
    N_min::Int64 = 2,
    N_max::Int64 = 8,
    N::Int64 = rand(N_min:N_max),
    num_obs_min::Int64 = 0,
    num_obs_max::Int64 = 10,
    num_obs::Int64 = rand(num_obs_min:num_obs_max),
    rad::Float64 = 0.025,
    rad_obs::Float64 = 0.05,
    rad_min::Float64 = rad,
    rad_max::Float64 = rad,
    rad_obs_min::Float64 = rad_obs,
    rad_obs_max::Float64 = rad_obs,
    TIME_LIMIT::Float64 = 0.5,
)::Tuple{Vector{StateDubins},Vector{StateDubins},Vector{CircleObstacle2D},Vector{Float64}}
    """This model do not consider 'back' operation,
    states too close to walls/obstacles cannot be reached.
    Hence this func inserts 'additional buffer'."""

    _q = StateDubins(0, 0, 0)
    while true
        t_s = now()
        timeover() = elapsed_sec(t_s) > TIME_LIMIT

        # generate obstacles
        obstacles = gen_obstacles(2, num_obs, rad_obs_min, rad_obs_max)

        # determine rads
        rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        connect(q::StateDubins, i::Int64) = begin
            r = rads[i] + rads[i] * DUBINS_TURN_RADIUS
            p = [q.x, q.y]
            return all(x -> r < x < 1 - r, p) && all(o -> dist(p, o) > r + o.r, obstacles)
        end

        # avoiding deadlock
        collide = gen_collide(_q, rads + rads * DUBINS_TURN_RADIUS)

        # generate starts & goals
        config_init, config_goal = gen_config_init_goal(_q, N, connect, collide, timeover)

        timeover() && continue
        return (config_init, config_goal, obstacles, rads)
    end
end

# function gen_random_instance_StateDubins(; params...)
#     return gen_random_instance(StateDubins(0, 0, 0); params...)
# end

function plot_anim!(
    config_init::Vector{StateDubins},
    config_goal::Vector{StateDubins},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...;
    solution::Union{Nothing,Vector{Vector{Node{StateDubins}}}} = nothing,
    filename::String = "tmp.gif",
    fps::Int64 = 10,
    interpolate_depth::Union{Nothing,Int64} = nothing,
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
            for i = 1:N
                P = get_dubins_points(
                    solution[t-1][i].q,
                    solution[t][i].q,
                    ins_params[1][i];
                    n_dividing = depth,
                )
                for (x, y, θ) in P
                    plot_agent!(
                        StateDubins(x, y, θ),
                        map(arr -> arr[i], ins_params)...,
                        get_color(i),
                    )
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
