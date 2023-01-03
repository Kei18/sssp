"""experimental scripts to evaluate solvers"""

using MRMP
import YAML
import Dates
import JLD2
import Random: seed!
import Base.Threads
import CSV

function get_solver_args(ins)
    config_init, config_goal, obstacles, ins_params... = ins
    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)
    check_goal = gen_check_goal(config_goal)
    return [config_init, config_goal, connect, collide, check_goal]
end

# read experimental setting
function main(config_file::String)
    # load experimental setting
    config = YAML.load_file(config_file)
    time_limit_sec = get(config, "time_limit_sec", 10)
    seed_start = get(config, "seed_start", 1)
    seed_end = get(config, "seed_end", seed_start)

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

    # pre-compile
    args = get_solver_args(first(I))
    foreach(solver -> solver(args...; TIME_LIMIT = time_limit_sec), solvers)

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
    cnt_fin = Threads.Atomic{Int}(0)
    cnt_solved = Threads.Atomic{Int}(0)
    r = (x) -> round(x, digits = 3)  # round
    t_start = Base.time_ns()

    # main loop
    result = Vector{Any}(undef, num_total_tasks)
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

        Threads.atomic_add!(cnt_fin, 1)
        row[:solved] && row[:valid] && (Threads.atomic_add!(cnt_solved, 1))
        print(
            "\r" *
            "$(r((Base.time_ns() - t_start) / 1.0e9)) sec" *
            "\t$(cnt_fin[])/$(num_instances) " *
            "($(r(cnt_fin[]/num_instances*100))%)" *
            " tasks have been finished, " *
            "solved: $(cnt_solved[])/$(cnt_fin[]) ($(r(cnt_solved[]/cnt_fin[]*100))%)",
        )
    end

    result_file = joinpath(root_dir, "result.csv")
    CSV.write(result_file, result)
    println("\nresult file was saved in $(result_file)")
end
