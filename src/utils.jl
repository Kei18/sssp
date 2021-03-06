"""utilities of multi-robot motion planning"""

# ------------------------------------------------
# time
# ------------------------------------------------
function now()
    return Base.time_ns()
end

function elapsed_sec(t_s::UInt64)
    return (now() - t_s) / 1.0e9
end

# ------------------------------------------------
# distance
# ------------------------------------------------
function dist(a::Vector{Float64}, b::Vector{Float64})::Float64
    norm(a - b)
end

function dist(a::Vector{Float64}, o::CircleObstacle2D)::Float64
    dist(a, [o.x, o.y])
end

function dist(a::Vector{Float64}, o::CircleObstacle3D)::Float64
    dist(a, [o.x, o.y, o.z])
end

function dist(v1::Node{State}, v2::Node{State})::Float64 where {State<:AbsState}
    dist(v1.q, v2.q)
end

function dist(C1::Vector{State}, C2::Vector{State})::Float64 where {State<:AbsState}
    sum(e -> dist(e...), zip(C1, C2)) / length(C1)
end

function dist(
    Q1::Vector{Node{State}},
    Q2::Vector{Node{State}},
)::Float64 where {State<:AbsState}
    sum(e -> dist(e...), zip(Q1, Q2)) / length(Q1)
end

function dist(a::Vector{Float64}, b::Vector{Float64}, c::Vector{Float64})::Float64
    """from line [a, b] to point [c]"""
    df_0 = dot(a - b, b - c)
    df_1 = dot(a - b, a - c)

    e = 1
    if df_0 * df_1 < 0
        e = -dot(a - b, b - c) / dot(a - b, a - b)
    elseif df_0 >= 0
        e = 0
    end

    return norm(e * a + (1 - e) * b - c)
end

function dist(
    p1_1::Vector{Float64},
    p1_2::Vector{Float64},
    p2_1::Vector{Float64},
    p2_2::Vector{Float64},
)::Float64
    """from line [p1_1, p1_2] to line [p2_1, p2_2]"""

    # c.f., http://marupeke296.com/COL_3D_No19_LinesDistAndPos.html
    v1 = p1_2 - p1_1
    v2 = p2_2 - p2_1
    D1 = dot(p2_1 - p1_1, v1)
    D2 = dot(p2_1 - p1_1, v2)
    Dv = dot(v1, v2)
    V1 = dot(v1, v1)
    V2 = dot(v2, v2)
    D = V1 * V2 - Dv * Dv
    if D > 0
        t1 = (D1 * V2 - D2 * Dv) / D
        t2 = (D1 * Dv - D2 * V1) / D
        if 0 <= t1 <= 1 && 0 <= t2 <= 1
            Q1 = p1_1 + t1 * v1
            Q2 = p2_1 + t2 * v2
            return norm(Q1 - Q2)
        end
    end

    return minimum(
        [
            dist(p1_1, p1_2, p2_1)
            dist(p1_1, p1_2, p2_2)
            dist(p2_1, p2_2, p1_1)
            dist(p2_1, p2_2, p1_2)
        ],
    )
end

"""compute difference of two angles"""
function diff_angles(t1::Float64, t2::Float64)::Float64
    atan(sin(t1 - t2), cos(t1 - t2))
end

# ------------------------------------------------
# functions to define instance
# ------------------------------------------------
"""
    gen_check_goal(
        config_goal::Vector{State};
        goal_rad::Float64 = 0.0,
        goal_rads::Vector{Float64} = fill(goal_rad, length(config_goal)),
    )::Function where {State<:AbsState}

generate goal check function, example:
```jl
check_goal = gen_check_goal(config_goal)
check_goal(config_goal)  # return true
```
"""
function gen_check_goal(
    config_goal::Vector{State};
    goal_rad::Float64 = 0.0,
    goal_rads::Vector{Float64} = fill(goal_rad, length(config_goal)),
)::Function where {State<:AbsState}

    N = length(config_goal)

    f(C::Vector{State}) = begin
        all(i -> dist(C[i], config_goal[i]) <= goal_rads[i], 1:N)
    end

    f(Q::Vector{Node{State}}) = begin
        all(i -> dist(Q[i].q, config_goal[i]) <= goal_rads[i], 1:N)
    end

    f(v::Node{State}, i::Int64) = begin
        return dist(v.q, config_goal[i]) <= goal_rads[i]
    end

    return f
end


# ------------------------------------------------
# instance generation
# ------------------------------------------------

"""generate obstacles randomly"""
function gen_obstacles(
    d::Int64,  # dimension
    num_obs::Int64,
    rad_obs_min::Float64,
    rad_obs_max::Float64,
)::Union{Vector{CircleObstacle2D},Vector{CircleObstacle3D}}
    r() = rand() * (rad_obs_max - rad_obs_min) + rad_obs_min
    return (
        d == 2 ? map(k -> CircleObstacle2D(rand(d)..., r()), 1:num_obs) :
        map(k -> CircleObstacle3D(rand(d)..., r()), 1:num_obs)
    )
end

"""generate initial/goal configurations randomly"""
function gen_config_init_goal(
    q::State,
    N::Int64,
    connect::Function,
    collide::Function,
    timeover::Function,
)::Tuple{Vector{State},Vector{State}} where {State<:AbsState}

    sampler = gen_uniform_sampling(q)
    config_init = Vector{State}()
    config_goal = Vector{State}()

    for i = 1:N
        # add start
        isvalid_init =
            (q::State) -> (
                connect(q, i) &&
                all(e -> !collide(q, e[2], i, e[1]), enumerate(config_init))
            )
        q_init = sampler()
        while !timeover() && !isvalid_init(q_init)
            q_init = sampler()
        end
        push!(config_init, q_init)

        # add goal
        isvalid_goal =
            (q::State) -> (
                connect(q, i) &&
                all(e -> !collide(q, e[2], i, e[1]), enumerate(config_goal))
            )
        q_goal = sampler()
        while !timeover() && !isvalid_goal(q_goal)
            q_goal = sampler()
        end
        push!(config_goal, q_goal)

        timeover() && break
    end

    return (config_init, config_goal)
end

"""generate random instance"""
function gen_random_instance(
    _q::State;  # to specify type
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
)::Tuple{
    Vector{State},
    Vector{State},
    Vector{Obs} where {Obs<:Obstacle},
    Vector{Float64},
} where {State<:AbsState}

    while true
        t_s = now()
        timeover() = elapsed_sec(t_s) > TIME_LIMIT

        # generate obstacles
        obstacles =
            gen_obstacles(State == StatePoint3D ? 3 : 2, num_obs, rad_obs_min, rad_obs_max)

        # determine rads
        rads = map(e -> rand() * (rad_max - rad_min) + rad_min, 1:N)

        # generate starts & goals
        connect = gen_connect(_q, obstacles, rads)
        collide = gen_collide(_q, rads)
        config_init, config_goal = gen_config_init_goal(_q, N, connect, collide, timeover)

        timeover() && continue
        return (config_init, config_goal, obstacles, rads)
    end
end

# ------------------------------------------------
# validation
# ------------------------------------------------

"""
    is_valid_instance(
        config_init::Vector{State},
        config_goal::Vector{State},
        obstacles::Vector{Obs} where {Obs<:Obstacle},
        ins_params...
    )::Bool where {State<:AbsState}

simple check whether instance is valid
"""
function is_valid_instance(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where {Obs<:Obstacle},
    ins_params...,
)::Bool where {State<:AbsState}

    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)

    # check start
    for (i, q) in enumerate(config_init)
        if !connect(q, q, i)
            @warn @sprintf("invalid instance, start of agent-%d %s is strage", i, string(q))
            return false
        end
    end

    # check goal
    for (i, q) in enumerate(config_goal)
        if !connect(q, q, i)
            @warn @sprintf("invalid instance, goal of agent-%d %s is strage", i, string(q))
            return false
        end
    end

    # check collisions (start-start or goal-goal) between two agents
    N = length(config_init)
    for i = 1:N, j = 1+i:N
        # start-start
        q_i = config_init[i]
        q_j = config_init[j]
        if collide(q_i, q_i, q_j, q_j, i, j)
            @warn @sprintf("invalid instance, starts of agent-%d, %d is colliding", i, j)
            return false
        end

        # goal-goal
        q_i = config_goal[i]
        q_j = config_goal[j]
        if collide(q_i, q_i, q_j, q_j, i, j)
            @warn @sprintf("invalid instance, goals of agent-%d, %d is colliding", i, j)
            return false
        end
    end
    return true
end

"""
    validate(
        config_init::Vector{State},
        connect::Function,
        collide::Function,
        check_goal::Function,
        solution::Union{Nothing,Vector{Vector{Node{State}}}},
    )::Bool where {State<:AbsState}

check validity of solution
"""
function validate(
    config_init::Vector{State},
    connect::Function,
    collide::Function,
    check_goal::Function,
    solution::Union{Nothing,Vector{Vector{Node{State}}}},
)::Bool where {State<:AbsState}

    isnothing(solution) && return true

    N = length(config_init)
    T = length(solution)

    # check initial condition
    if !all(i -> solution[1][i].q == config_init[i], 1:N)
        @warn @sprintf("invalid: initial configuration")
        return false
    end

    # check goal
    if !check_goal(last(solution))
        @warn @sprintf("invalid: last configuration")
        return false
    end

    for t = 2:T
        # check connectivity
        for i = 1:N
            if !connect(solution[t-1][i].q, solution[t][i].q, i)
                @warn @sprintf(
                    "invalid: not connected, agent-%d, at t= %d -> %d",
                    i,
                    t - 1,
                    t
                )
                return false
            end
        end

        # check collision
        if collide(solution[t-1], solution[t])
            @warn @sprintf("invalid: colliding at t= %d -> %d", t - 1, t)
            return false
        end
    end

    return true
end
