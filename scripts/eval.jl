"""experimental scripts to evaluate solvers"""

using MRMP
import YAML
import Dates
import JLD2
import Random: seed!
import Base.Threads
import Printf: @sprintf
import CSV

function get_solver_args(ins)
    config_init, config_goal, obstacles, ins_params... = ins
    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)
    check_goal = gen_check_goal(config_goal)
    return [config_init, config_goal, connect, collide, check_goal]
end

# read experimental setting
function main(args...)
    # load experimental setting
    config = merge(
        map(
            arg -> begin
                isfile(arg) && return YAML.load_file(arg)
                typeof(arg) == String &&
                    return Dict(first(split(arg, "=")) => last(split(arg, "=")))
                typeof(arg) == Dict && return arg
                Dict()
            end,
            args,
        )...,
    )
    time_limit_sec = get(config, "time_limit_sec", 10)
    typeof(time_limit_sec) != Int && (time_limit_sec = parse(Int, time_limit_sec))
    seed_start = get(config, "seed_start", 1)
    typeof(seed_start) != Int && (seed_start = parse(Int, seed_start))
    seed_end = get(config, "seed_end", seed_start)
    typeof(seed_end) != Int && (seed_end = parse(Int, seed_end))

    # prepare directory
    date_str = replace(string(Dates.now()), ":" => "-")
    root_dir = joinpath(pwd(), "..", "data", "exp", date_str)
    !isdir(root_dir) && mkpath(root_dir)

    # save configuration file
    io = IOBuffer()
    versioninfo(io, verbose = true)
    additional_info = Dict(
        "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
        "date" => date_str,
        "nthreads" => Threads.nthreads(),
        "env" => String(take!(io)),
    )
    YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))

    # load benchmark
    I = JLD2.load(config["benchmark_file"], "instances")
    num_instances = length(I)

    # load solvers
    solvers = []
    for solver_info in config["solvers"]
        target = Meta.parse(solver_info["target"])
        params = Dict()
        foreach(e -> params[Symbol(first(e))] = last(e), solver_info["params"])
        solver = (args..., ; kwargs...) -> eval(target)(args...; params..., kwargs...)
        push!(solvers, solver)
    end
    num_solvers = length(solvers)

    # pre-compile
    args = get_solver_args(first(I))
    println("pre-compiling")
    Threads.@threads for solver in solvers
        solver(args...; TIME_LIMIT = time_limit_sec)
    end

    # generate iterators
    iterators =
        Iterators.product(
            enumerate(get_solver_args.(I)),
            enumerate(solvers),
            seed_start:seed_end,
        ) |>
        enumerate |>
        collect
    num_total_tasks = length(iterators)
    cnt_fin = map(_ -> Threads.Atomic{Int}(0), 1:num_solvers)
    cnt_solved = map(_ -> Threads.Atomic{Int}(0), 1:num_solvers)

    # main loop
    println("done, start exp")
    result = Vector{Any}(undef, num_total_tasks)
    t_start = Base.time_ns()
    Threads.@threads for (k, ((idx_ins, args), (idx_solver, solver), seed)) in iterators
        seed!(seed)

        # solve
        comp_time_planning = @elapsed begin
            solution, _ = solver(args...; TIME_LIMIT = time_limit_sec)
        end
        cost_original = get_solution_cost(solution)

        comp_time_refinement = @elapsed begin
            res_refined = smoothing(solution, args[3], args[4])
        end

        # validate
        row = Dict(
            :seed => seed,
            :benchmark => config["benchmark_file"],
            :instance => idx_ins,
            :solver_index => idx_solver,
            :solver => config["solvers"][idx_solver]["target"],
            :solved => !isnothing(solution),
            :valid => validate(args[1], args[3:5]..., solution),
            :elapsed_planning => comp_time_planning,
            :elapsed_refinement => comp_time_refinement,
            :elapsed_total => comp_time_planning + comp_time_refinement,
            :soc_original => isnothing(cost_original) ? 0 : cost_original[:sum_of_cost],
            :makespan_original =>
                isnothing(cost_original) ? 0 : cost_original[:makespan],
            :sum_of_cost_refined =>
                isnothing(res_refined) ? 0 : res_refined[end][:sum_of_cost],
            :makespan_refined =>
                isnothing(res_refined) ? 0 : res_refined[end][:makespan],
        )
        result[k] = NamedTuple{Tuple(keys(row))}(values(row))

        Threads.atomic_add!(cnt_fin[idx_solver], 1)
        row[:solved] && row[:valid] && (Threads.atomic_add!(cnt_solved[idx_solver], 1))

        cnt_total_fin = sum(map(l -> cnt_fin[l][], 1:num_solvers))
        str_solved = join(
            map(
                l -> begin
                    @sprintf("%1d ", l) *
                    last(split(config["solvers"][l]["target"], ".")) *
                    ":" *
                    @sprintf(
                        "%4d/%4d (%3d%%)",
                        cnt_solved[l][],
                        cnt_fin[l][],
                        cnt_solved[l][] / cnt_fin[l][] * 100
                    )
                end,
                1:num_solvers,
            ),
            "; ",
        )
        print(
            "\r" *
            @sprintf(
                "%6d sec, %4d/%4d (%3d%%) tasks done",
                (Base.time_ns() - t_start) / 1.0e9,
                cnt_total_fin,
                num_total_tasks,
                cnt_total_fin / num_total_tasks * 100
            ) *
            "\t$(str_solved)",
        )
    end

    result_file = joinpath(root_dir, "result.csv")
    CSV.write(result_file, result)
    println("\nresult file was saved in $(result_file)")
end
