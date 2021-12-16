using LinearAlgebra: norm, dot, normalize

function direction(p_i::Vector{Float64}, p_j::Vector{Float64}, p_k::Vector{Float64})::Float64
    a = p_k - p_i
    b = p_j - p_i
    return a[1] * b[2] - b[1] * a[2]
end

function segments_intersect(
    p1::Vector{Float64}, p2::Vector{Float64}, p3::Vector{Float64}, p4::Vector{Float64})::Bool
    return (
        (direction(p3, p4, p1)*direction(p3, p4, p2) < 0) &&
        (direction(p1, p2, p3)*direction(p1, p2, p4) < 0)
    )
end

# from line to point
function dist(a::Vector{Float64}, b::Vector{Float64}, c::Vector{Float64})::Float64
    df_0 = dot(a-b, b-c)
    df_1 = dot(a-b, a-c)

    e = 1
    if df_0*df_1 < 0
        e = - dot(a-b, b-c) / dot(a-b, a-b)
    elseif df_0 > 0
        e = 0
    end

    return norm(e*a + (1-e)*b - c)
end

# from line to line
function dist(
    p1_1::Vector{Float64}, p1_2::Vector{Float64},
    p2_1::Vector{Float64}, p2_2::Vector{Float64})::Float64

    # c.f., http://marupeke296.com/COL_3D_No19_LinesDistAndPos.html
    v1 = p1_2 - p1_1
    v2 = p2_2 - p2_1
    D1 = dot(p2_1 - p1_1, v1)
    D2 = dot(p2_1 - p1_1, v2)
    Dv = dot(v1, v2)
    V1 = dot(v1, v1)
    V2 = dot(v2, v2)
    D = V1*V2 - Dv*Dv
    if D > 0
        t1 = (D1*V2 - D2*Dv) / D
        t2 = (D1*Dv - D2*V1) / D
        if 0 <= t1 <= 1 && 0 <= t2 <= 1
            Q1 = p1_1 + t1*v1
            Q2 = p2_1 + t2*v2
            return norm(Q1 - Q2)
        end
    end

    return minimum([dist(p1_1, p1_2, p2_1)
                    dist(p1_1, p1_2, p2_2)
                    dist(p2_1, p2_2, p1_1)
                    dist(p2_1, p2_2, p1_2)])
end


function uniform_ball_sampling(d::Int64)
    f = () -> rand(d) * 2 - fill(1, d)
    s = f()
    while norm(s) > 1; s = f(); end
    return s
end


function gen_check_goal(
    config_goal::Vector{State};
    goal_rad::Float64=0.0,
    goal_rads::Vector{Float64}=fill(goal_rad, length(config_goal))
    )::Function where State<:AbsState

    return (Q::Vector{Node{State}}) -> begin
        return all([dist(v.q, q) <= goal_rads[i]
                    for (i, (v, q)) in enumerate(zip(Q, config_goal))])
    end
end

function gen_h_func(config_goal::Vector{State})::Function where State<:AbsState
    return (Q::Vector{Node{State}}) -> begin
        return sum([dist(v.q, config_goal[i]) for (i, v) in enumerate(Q)])
    end
end

function gen_g_func(;greedy::Bool=false)::Function
    return (Q_from, Q_to) -> begin
        return greedy ? 0 : sum([dist(u.q, v.q) for (u, v) in zip(Q_from, Q_to)])
    end
end

function gen_get_sample_nums(k_init::Int64)::Function
    return (k::Int64) -> k + k_init - 1
end

function simple_search(
    config_init::Vector{State},
    config_goal::Vector{State},
    obstacles::Vector{Obs} where Obs<:Obstacle,
    rads::Vector{Float64}=fill(0.1, length(config_init));
    eps::Float64=0.2,
    sample_num_init::Int64=3,
    goal_rad::Float64=0.0,
    params=Dict()
    ) where State<:AbsState

    q = config_init[1]
    connect = gen_connect(q, rads, obstacles, eps)
    collide = gen_collide(q, rads)
    check_goal = gen_check_goal(config_goal, goal_rad=goal_rad)
    h_func = gen_h_func(config_goal)
    g_func = gen_g_func(greedy=true)
    random_walk = gen_random_walk(q, eps)
    get_sample_nums = gen_get_sample_nums(sample_num_init)
    return search(config_init, config_goal, connect, collide, check_goal,
                  h_func, g_func, random_walk, get_sample_nums; params...)
end

function now()
    return Base.time_ns()
end

function elapsed_sec(t_s::UInt64)
    return (now() - t_s) / 1.0e9
end

# TODO: refactoring
function print_instance(
    config_init::Vector{StatePoint2D},
    config_goal::Vector{StatePoint2D},
    rads::Vector{Float64},
    obstacles::Vector{CircleObstacle2D}
)::Nothing

    println("problem instance:")
    for (i, (q_init, q_goal, rad)) in enumerate(zip(config_init, config_goal, rads))
        @printf("- %02d: (%.4f, %.4f) -> (%.4f, %.4f), rad: %.4f\n",
                i, q_init.x, q_init.y, q_goal.x, q_goal.y, rad)
    end
    if !isempty(obstacles)
        println("obstacles:")
        for (i, o) in enumerate(obstacles)
            @printf("- %02d: (%.4f, %.4f), rad: %.4f\n", i, o.x, o.y, o.r)
        end
    end
    println()
end

function is_valid_instance(
    config_init::Vector{StatePoint2D},
    config_goal::Vector{StatePoint2D},
    rads::Vector{Float64}
    )::Bool

    for (i, (q1, q2)) in enumerate(zip(config_init, config_goal))
        if any([x < rads[i] || 1-rads[i] < x for x in [q1.x, q1.y, q2.x, q2.y]])
            @warn("invalid instance, start/goal of agent-", i, " is out of range")
            return false
        end
    end
    N = length(config_init)
    for i = 1:N, j = 1+i:N
        if MRMP.dist(config_init[i], config_init[j]) < rads[i] + rads[j]
            @warn("invalid instance, starts of agent-", i, ", ", j, " is colliding")
            return false
        end
        if MRMP.dist(config_goal[i], config_goal[j]) < rads[i] + rads[j]
            @warn("invalid instance, goals of agent-", i, ", ", j, " is colliding")
            return false
        end
    end
    return true
end
