using MRMP
import Logging

struct Field
    x::Float64
    y::Float64
    size::Float64
end

@kwdef mutable struct Query
    field = nothing
    config_init = nothing
    config_goal = nothing
    rads = nothing
    obstacles = nothing
    roadmaps = nothing
    connect = nothing
    collide = nothing
    check_goal = nothing
    h_func = nothing
    g_func = nothing
    random_walk = nothing
    num_neighbors = nothing
    neighbors_growing_rate = nothing
    init_search_time_limit = nothing
    skip_init_refinement = false
    skip_online_refinement = false
    params_init = nothing
    params_refine_init = nothing
    params_refine_online = nothing
    committed_indexes = nothing
    start_time = MRMP.now()
    save_fig = false
    hist_TPG = []
    hist_TPG_committed = []
    hist_commited_indexes = []
    drawing_tasks = []
end

scale(x::Real, y::Real, field::Field) = begin
    StatePoint2D((Float64(x) - field.x) / field.size, (Float64(y) - field.y) / field.size)
end
scale(x::Real, y::Real, r::Real, field::Field) = begin
    CircleObstacle2D((x - field.x) / field.size, (y - field.y) / field.size, r / field.size)
end
scale(r::Real, field::Field) = r / field.size
rescale(q::StatePoint2D, field::Field) =
    (q.x * field.size + field.x, q.y * field.size + field.y)
rescale_x(x::Float64, field::Field) = x * field.size + field.x
rescale_y(y::Float64, field::Field) = y * field.size + field.y
get_commit_str(committed_indexes) = begin
    reduce(*, [@sprintf("%02d_", k) for k in committed_indexes])[1:end-1]
end

function get_instructions(TPG, field)
    get_dict(a) = Dict(
        "id" => a.id,
        "x_from" => rescale_x(a.from.q.x, field),
        "y_from" => rescale_y(a.from.q.y, field),
        "x_to" => rescale_x(a.to.q.x, field),
        "y_to" => rescale_y(a.to.q.y, field),
        "pre" => a.predecessors,
        "suc" => a.successors,
    )
    return [map(get_dict, actions) for actions in TPG]
end

function save_planning(query)
    if !query.save_fig
        return
    end
    idx = length(query.hist_TPG)
    dirname = "./local/toio2d/" * string(query.start_time)
    datestr = string(MRMP.now())

    task = @async begin
        TPG1 = query.hist_TPG_committed[idx]
        TPG2 = query.hist_TPG[idx]
        MRMP.plot_tpg!(TPG1, TPG2; filename = "$dirname/tpg/$datestr.pdf")
        solution = MRMP.get_greedy_solution(TPG2; config_goal = query.config_goal)
        config_init = map(v -> v.q, solution[1])
        plot_anim!(
            query.config_init,
            query.config_goal,
            query.obstacles,
            query.rads,
            solution;
            filename = "$dirname/anim/$datestr.gif",
            fps = 3,
        )
    end
    push!(query.drawing_tasks, task)
end

macro fail_to_solve(msg::String)
    return esc(quote
        @warn $msg
        write(ws, JSON.json(Dict("status" => :failure)))
        return
    end)
end

function setup(req, query)
    # setup instance
    query.field = Field(req["field"]["x"], req["field"]["y"], req["field"]["size"])
    query.config_init =
        map(x -> scale(x["x_init"], x["y_init"], query.field), req["agents"])
    query.config_goal =
        map(x -> scale(x["x_goal"], x["y_goal"], query.field), req["agents"])
    query.rads = map(x -> scale(x["rad"], query.field), req["agents"])
    query.obstacles = (
        (req["obstacles"] != nothing) ?
        map(o -> scale(o["x"], o["y"], o["rad"], query.field), req["obstacles"]) :
        Vector{CircleObstacle2D}()
    )

    # setup search details
    q = query.config_init[1]
    eps = get(req["search"], "eps", 0.2)
    query.connect = gen_connect(q, query.rads, query.obstacles, eps)
    query.collide = gen_collide(q, query.rads)
    query.check_goal = gen_check_goal(query.config_goal)
    query.h_func = gen_h_func(query.config_goal)
    query.g_func = gen_g_func(greedy = true)
    query.random_walk = gen_random_walk(q, eps)
    query.init_search_time_limit = get(req["search"], "init_search_time_limit", 10)
    query.skip_init_refinement = get(req["search"], "skip_init_refinement", false)
    query.skip_online_refinement = get(req["search"], "skip_online_refinement", false)
    query.params_init =
        Dict([(Symbol(key), val) for (key, val) in req["search"]["params_init"]])
    query.params_refine_init =
        Dict([(Symbol(key), val) for (key, val) in req["search"]["params_refine_init"]])
    query.params_refine_online =
        Dict([(Symbol(key), val) for (key, val) in req["search"]["params_refine_online"]])
    query.neighbors_growing_rate = get(req["search"], "neighbors_growing_rate", 1.1)
    query.num_neighbors = get(req["search"], "num_neighbors_init", 5)
    query.committed_indexes = fill(0, length(query.config_goal))
    query.save_fig = get(req, "save_fig", false)

    # setup seed
    seed!(get(req, "seed", 0))
end

function setup_tpg!(TPG, TPG_committed, old_committed_indexes, new_committed_indexes)
    N = length(TPG)
    offset = maximum(map(e -> (isempty(e) ? 0 : e[end].t), TPG_committed)) - 1
    for i = 1:N
        k = new_committed_indexes[i] - old_committed_indexes[i]
        if k == 0 || isempty(TPG[i])
            continue
        end

        # save last commit
        l = length(TPG_committed[i])
        append!(TPG_committed[i], TPG[i][1:k])
        if l > 0
            push!(TPG_committed[i][l].successors, (i, TPG_committed[i][l+1].id))
            push!(TPG_committed[i][l+1].predecessors, (i, TPG_committed[i][l].id))
        end
        map(e -> begin
            e.t += offset
        end, TPG_committed[i][l+1:end])

        # cut
        TPG[i] = TPG[i][k+1:end]
    end

    # remove unnecessary relationship
    for i = 1:N
        for action in TPG[i]
            filter!(
                e -> findfirst(a -> a.id == e[2], TPG[e[1]]) != nothing,
                action.predecessors,
            )
        end
    end
end

Base.deepcopy(s::MRMP.Action{StatePoint2D}) = begin
    MRMP.Action(
        s.id,
        s.from,
        s.to,
        s.t,
        s.agent,
        deepcopy(s.predecessors),
        deepcopy(s.successors),
    )
end
Base.deepcopy(s::Vector{MRMP.Action{StatePoint2D}}) = [deepcopy(a) for a in s]
Base.deepcopy(s::Vector{Vector{MRMP.Action{StatePoint2D}}}) =
    [deepcopy(actions) for actions in s]

function store_new_tpg(query, TPG, TPG_committed, committed_indexes)
    push!(query.hist_TPG, deepcopy(TPG))
    push!(query.hist_TPG_committed, deepcopy(TPG_committed))
    push!(query.hist_commited_indexes, deepcopy(committed_indexes))
    save_planning(query)
end

function timed_metafmt(level, _module, group, id, file, line)
    color, prefix, suffix = Logging.default_metafmt(level, _module, group, id, file, line)
    timestamp = MRMP.now()
    return color, "$timestamp $prefix", suffix
end

Logging.global_logger(
    Logging.ConsoleLogger(meta_formatter = timed_metafmt, right_justify = 100),
)
