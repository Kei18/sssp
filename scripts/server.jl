"""planning server for point2d robot"""

using MRMP
import MRMP: StatePoint2D, CircleObstacle2D, Action
import Dates
using JSON
using HTTP
import Random: seed!
import Printf: @printf, @sprintf
import Base: @kwdef
import Sockets

# filed info, assuming square
struct Field
    x::Float64
    y::Float64
    size::Float64
end

# utilities
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

# convert TPG to instructions
function get_instructions(
    TPG::Vector{Vector{Action{StatePoint2D}}},
    field::Field,
)::Vector{Vector{Dict}}

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

function print_instance(
    config_init::Vector{StatePoint2D},
    config_goal::Vector{StatePoint2D},
    obstacles::Vector{CircleObstacle2D},
    rads::Vector{Float64},
)::Nothing

    @info "problem instance:"
    for (i, (q_init, q_goal, rad)) in enumerate(zip(config_init, config_goal, rads))
        @info @sprintf("\t%02d: %s -> %s, rad: %.4f\n", i, q_init, q_goal, rad)
    end
    if !isempty(obstacles)
        @info "obstacles:"
        foreach(o -> @info(@sprintf("\t- %s\n", o)), obstacles)
    end
end

# used when failing to solve problem
macro fail_to_solve(msg::String)
    esc(quote
        @warn $msg
        write(ws, JSON.json(Dict("status" => :failure)))
        return
    end)
end

function planning(req, ws)
    # time utilities
    t_s = MRMP.now()
    elapsed() = MRMP.elapsed_sec(t_s)
    dirname = "./local/toio2d/" * string(Dates.now())

    # setup problem instance
    field = Field(req["field"]["x"], req["field"]["y"], req["field"]["size"])
    config_init = map(x -> scale(x["x_init"], x["y_init"], field), req["agents"])
    config_goal = map(x -> scale(x["x_goal"], x["y_goal"], field), req["agents"])
    obstacles = (
        isnothing(req["obstacles"]) ? Vector{CircleObstacle2D}() :
        map(o -> scale(o["x"], o["y"], o["rad"], field), req["obstacles"])
    )
    rads = map(x -> scale(x["rad"], field), req["agents"])
    connect = gen_connect(config_init[1], obstacles, rads)
    collide = gen_collide(config_init[1], rads)
    check_goal = MRMP.gen_check_goal(config_goal)
    print_instance(config_init, config_goal, obstacles, rads)
    get(req, "save_fig", false) && @async begin
        plot_instance!(config_init, config_goal, obstacles, rads; filename = "$dirname/ins.pdf")
    end
    !is_valid_instance(config_init, config_goal, obstacles, rads) &&
        @fail_to_solve("invalid instance")

    # get initial solution
    @info @sprintf("searching initial solution\n")
    solver_name = req["solver"]["_target_"]
    solver = Meta.parse(solver_name)
    params =
        Dict([(Symbol(k), v) for (k, v) in filter(e -> e[1] != "_target_", req["solver"])])
    solution, roadmaps =
        eval(solver)(config_init, config_goal, connect, collide, check_goal; params...)
    isnothing(solution) && @fail_to_solve("initial solution is not found")

    # smoothing
    (TPG, solution, cost) = smoothing(solution, connect, collide)
    @info @sprintf("get initial solution, elapsed: %f sec,\ncost: %s\n", elapsed(), cost)

    # send initial plan
    write(
        ws,
        JSON.json(
            Dict("status" => :success, "instructions" => get_instructions(TPG, field)),
        ),
    )

    # save result
    get(req, "save_fig", false) && @async begin
        plot_instance!(config_init, config_goal, obstacles, rads; filename = "$dirname/ins.pdf")
        plot_tpg!(TPG; filename = "$dirname/tpg.pdf")
        plot_anim!(
            config_init,
            config_goal,
            obstacles,
            rads;
            solution = solution,
            filename = "$dirname/anim.gif",
            fps = 3,
        )
    end
end

# server setup
function main(; ADDR::String = "127.0.0.1", PORT = UInt16(8081))
    server = Sockets.listen(Sockets.getaddrinfo(ADDR), PORT)
    @info @sprintf("start server %s:%d", ADDR, PORT)
    @async begin
        req_num = 0
        HTTP.WebSockets.listen(ADDR, PORT; server = server) do ws
            @info @sprintf("\n-----\n%02d: receive request", req_num)
            req_num += 1
            while !eof(ws)
                req = JSON.parse(String(readavailable(ws)))
                planning(req, ws)
            end
            @info @sprintf("\n%02d: finish planning\n", req_num)
        end
    end
    return server
end

server = main(; ARGS...)
Sockets.close() = close(server)
